#include "pi_franka_hand.hpp"

#include <chrono>
#include <utility>

#include <franka/exception.h>
#include <franka/gripper.h>

#include "pi_info.hpp"

namespace {

double monotonic_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

struct FrankaHandTransport::Impl {
    std::unique_ptr<franka::Gripper> gripper;
};

FrankaHandTransport::FrankaHandTransport(FrankaHandConfig config)
    : config_(std::move(config)), impl_(std::make_unique<Impl>()) {}

FrankaHandTransport::~FrankaHandTransport() { stop(); }

bool FrankaHandTransport::start() {
    if (config_.address.empty()) return false;
    try {
        impl_->gripper = std::make_unique<franka::Gripper>(config_.address);
        // One synchronous read before any thread exists: it fails fast when the
        // hand is unreachable, and it is where max_width comes from. Homing is
        // not required to read a usable max_width, so it stays opt-in.
        const franka::GripperState initial = impl_->gripper->readOnce();
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = {};
        max_width_ = initial.max_width;
        last_width_ = initial.width;
        last_read_seconds_ = monotonic_seconds();
        state_.connected = true;
        // Nothing has to be activated to command a Franka Hand; homing only
        // recalibrates the stroke, so an unhomed hand is already commandable.
        state_.activated = !config_.homing;
        state_.position = width_to_position(initial.width, max_width_);
        state_.target = state_.position;
        target_position_ = state_.position;
        has_target_ = false;
        command_generation_ = executed_generation_ = 0;
    } catch (const std::exception& error) {
        PI_ERROR("Failed to open the Franka Hand at %s: %s", config_.address.c_str(), error.what());
        impl_->gripper.reset();
        return false;
    }
    running_ = true;
    reader_ = std::thread(&FrankaHandTransport::read_loop, this);
    commander_ = std::thread(&FrankaHandTransport::command_loop, this);
    return true;
}

void FrankaHandTransport::stop() {
    const bool was_running = running_.exchange(false);
    condition_.notify_all();
    if (was_running && impl_->gripper) {
        // Aborts a move that is still travelling so teardown does not wait out
        // a full stroke. Reading and stopping from another thread is what
        // franka::Gripper documents as threadsafe.
        try {
            impl_->gripper->stop();
        } catch (const std::exception& error) {
            PI_WARN("Franka Hand stop failed: %s", error.what());
        }
    }
    if (commander_.joinable()) commander_.join();
    // The reader sits in readOnce(); it returns on the next state packet (about
    // 25 ms at rest) or throws once the stream is gone.
    if (reader_.joinable()) reader_.join();
    impl_->gripper.reset();
    std::lock_guard<std::mutex> lock(mutex_);
    state_.connected = false;
}

bool FrankaHandTransport::activate() {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!running_) return false;
    if (!config_.homing) {
        state_.activated = true;
        return state_.fault == FRANKA_HAND_FAULT_NONE;
    }
    homing_requested_ = true;
    condition_.notify_all();
    const bool finished = condition_.wait_for(lock, std::chrono::seconds(20), [this] {
        return !homing_requested_ || !running_;
    });
    return finished && state_.activated && state_.fault == FRANKA_HAND_FAULT_NONE;
}

void FrankaHandTransport::set_target(float position, float speed, float force) {
    // The hand positions with move(), which takes no force. Grasping force is
    // the hand's own; the argument is accepted for interface parity.
    (void)force;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || !std::isfinite(position) || !std::isfinite(speed)) return;
    const float next_position = std::clamp(position, 0.0f, 1.0f);
    const float next_speed = std::clamp(speed, 0.0f, 1.0f);
    if (has_target_ && next_position == target_position_ && next_speed == target_speed_) return;
    target_position_ = next_position;
    target_speed_ = next_speed;
    has_target_ = true;
    state_.target = target_position_;
    ++command_generation_;
    condition_.notify_all();
}

void FrankaHandTransport::hold() {
    std::lock_guard<std::mutex> lock(mutex_);
    // Abandons a command that has not started; one already travelling runs to
    // its width, because move() cannot be interrupted from here.
    executed_generation_ = command_generation_;
    has_target_ = false;
    homing_requested_ = false;
    condition_.notify_all();
}

EffectorTransportState FrankaHandTransport::effector_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    EffectorTransportState state = state_;
    if (last_read_seconds_ > 0.0) {
        state.frame_age_ms = static_cast<float>(
            std::max(0.0, monotonic_seconds() - last_read_seconds_) * 1000.0);
    }
    return state;
}

bool FrankaHandTransport::has_effector_fault() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.fault != FRANKA_HAND_FAULT_NONE;
}

void FrankaHandTransport::read_loop() {
    while (running_) {
        franka::GripperState reading;
        try {
            reading = impl_->gripper->readOnce();
        } catch (const std::exception& error) {
            if (running_) {
                PI_ERROR("Franka Hand state stream stopped: %s", error.what());
                std::lock_guard<std::mutex> lock(mutex_);
                state_.connected = false;
                state_.fault = FRANKA_HAND_FAULT_DISCONNECTED;
                condition_.notify_all();
            }
            return;
        }
        const double now = monotonic_seconds();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (reading.max_width > 0.0) max_width_ = reading.max_width;
            const float position = width_to_position(reading.width, max_width_);
            const double elapsed = now - last_read_seconds_;
            state_.velocity =
                elapsed > 0.0 ? static_cast<float>((position - state_.position) / elapsed) : 0.0f;
            state_.position = position;
            state_.connected = true;
            last_width_ = reading.width;
            last_read_seconds_ = now;
            condition_.notify_all();
        }
        // The hand paces this loop: readOnce() blocks for the next packet
        // (about 40 Hz at rest, 8 Hz while the fingers travel). The sleep is
        // only a floor so a fast stream cannot spin a core.
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.read_period_ms));
    }
}

void FrankaHandTransport::command_loop() {
    while (true) {
        bool homing = false;
        float position = 0.0f;
        float speed = 0.0f;
        double max_width = 0.0;
        float position_before = 0.0f;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] {
                return !running_ || homing_requested_ ||
                       command_generation_ != executed_generation_;
            });
            if (!running_) return;
            homing = homing_requested_;
            if (!homing) {
                position = target_position_;
                speed = target_speed_;
                executed_generation_ = command_generation_;
            }
            max_width = max_width_;
            position_before = state_.position;
            state_.moving = true;
        }

        if (homing) {
            bool homed = false;
            try {
                homed = impl_->gripper->homing();
            } catch (const std::exception& error) {
                PI_ERROR("Franka Hand homing failed: %s", error.what());
            }
            std::lock_guard<std::mutex> lock(mutex_);
            homing_requested_ = false;
            state_.activated = homed;
            state_.moving = false;
            if (!homed) state_.fault = FRANKA_HAND_FAULT_COMMAND;
            condition_.notify_all();
            continue;
        }

        const double width = position_to_width(position, max_width);
        const double speed_m_s =
            std::max(0.001, static_cast<double>(speed) * static_cast<double>(config_.speed_m_s));
        const bool closing = position < position_before;
        bool reached = false;
        bool threw = false;
        try {
            reached = impl_->gripper->move(width, speed_m_s);
        } catch (const std::exception& error) {
            threw = true;
            PI_ERROR("Franka Hand move to %.4f m failed: %s", width, error.what());
        }
        std::lock_guard<std::mutex> lock(mutex_);
        state_.moving = false;
        // A -> B -> A while A runs cancels B and needs no second move(A).
        // A closing move stopped short is terminal too; periodic targets must
        // not turn contact into an unbounded series of close attempts.
        if (!threw && has_target_ && target_position_ == position && target_speed_ == speed) {
            executed_generation_ = command_generation_;
        }
        // Fingers stopped by an object on the way closed is how a grasp ends,
        // not a fault. An opening that never reached its width, or any thrown
        // command, is one.
        if (threw || (!reached && !closing)) state_.fault = FRANKA_HAND_FAULT_COMMAND;
        condition_.notify_all();
    }
}
