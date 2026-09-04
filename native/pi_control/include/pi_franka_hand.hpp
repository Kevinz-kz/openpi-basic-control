#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "pi_effector_transport.hpp"

/*! @brief Fault codes published in EffectorTransportState::fault. */
enum FrankaHandFault : uint8_t {
    FRANKA_HAND_FAULT_NONE = 0,
    FRANKA_HAND_FAULT_DISCONNECTED = 1,  ///< The state stream stopped.
    FRANKA_HAND_FAULT_COMMAND = 2,       ///< An opening command did not reach its width.
};

struct FrankaHandConfig {
    std::string address;        ///< Hostname or IP of the FR3 the hand is attached to.
    float speed_m_s = 0.05f;    ///< Finger speed at full commanded speed.
    float force_n = 20.0f;      ///< Reserved: move() positions, it does not take a force.
    bool homing = false;        ///< Run homing() on activate() to recalibrate max width.
    int read_period_ms = 10;    ///< Floor between state reads; the hand paces the rest.
};

/*!
 * @class FrankaHandTransport
 * @brief The Franka Hand as an FR3 end effector.
 *
 * One franka::Gripper connection, two threads on it: a reader blocked in
 * readOnce() and a command thread executing the latest requested width. The
 * gripper server accepts exactly ONE client, so nothing else in the system may
 * open a second connection while this transport runs -- the second connection
 * is refused and takes the first one down with it.
 *
 * Reading while a command runs is safe: franka::Gripper documents its members
 * as threadsafe, move()/grasp()/homing() go over TCP while readOnce() reads a
 * separate UDP state stream. The stream slows to roughly 8 Hz while the
 * fingers travel (about 40 Hz at rest), so a mid-stroke width is a frame or
 * two old.
 */
class FrankaHandTransport final : public EffectorTransport {
   public:
    explicit FrankaHandTransport(FrankaHandConfig config);
    ~FrankaHandTransport() override;

    bool start() override;
    void stop() override;
    bool activate() override;
    void set_target(float position, float speed, float force) override;
    void hold() override;
    EffectorTransportState effector_state() const override;
    bool has_effector_fault() const override;

    /*! @brief Normalized opening for a finger width, against a maximum width. */
    static float width_to_position(double width, double max_width) {
        if (!(max_width > 0.0)) return 0.0f;
        return static_cast<float>(std::clamp(width / max_width, 0.0, 1.0));
    }

    /*! @brief Finger width for a normalized opening, against a maximum width. */
    static double position_to_width(float position, double max_width) {
        return std::clamp(static_cast<double>(position), 0.0, 1.0) * std::max(0.0, max_width);
    }

   private:
    struct Impl;
    void read_loop();
    void command_loop();

    FrankaHandConfig config_;
    std::unique_ptr<Impl> impl_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::thread reader_;
    std::thread commander_;
    std::atomic<bool> running_{false};
    EffectorTransportState state_;
    double max_width_ = 0.0;
    double last_width_ = 0.0;
    double last_read_seconds_ = 0.0;
    float target_position_ = 1.0f;
    float target_speed_ = 1.0f;
    uint64_t command_generation_ = 0;
    uint64_t executed_generation_ = 0;
    bool homing_requested_ = false;
};
