#pragma once

#include <array>

// No values here: the law comes from FR3_law.json through FR3Law, so that the
// arm and any simulation of it read one file. A zero-initialised instance is
// not a usable controller.
struct FR3ControllerGains {
    std::array<double, 6> cartesian_stiffness{};
    std::array<double, 6> cartesian_damping{};
    std::array<double, 7> joint_stiffness{};
    std::array<double, 7> joint_damping{};
};

struct FR3ControllerLimits {
    std::array<double, 3> cartesian_lower{};
    std::array<double, 3> cartesian_upper{};
    std::array<double, 7> joint_lower{};
    std::array<double, 7> joint_upper{};
    std::array<double, 7> velocity{};
    std::array<double, 7> torque{};
    double joint_margin = 0.0;
    double velocity_margin = 0.0;
    double cartesian_margin = 0.0;
    double joint_stiffness = 0.0;
    double velocity_stiffness = 0.0;
    double cartesian_stiffness = 0.0;
    // A hard check of its own on the elbow, on top of the joint soft limits.
    double elbow_velocity = 0.0;
};

struct FR3ControllerInput {
    std::array<double, 7> q{};
    std::array<double, 7> dq{};
    std::array<double, 7> coriolis{};
    std::array<double, 42> flange_jacobian{};
    std::array<double, 42> end_effector_jacobian{};
    std::array<double, 3> end_effector_position{};
    double elbow_velocity = 0.0;
};

class FR3Controller {
   public:
    FR3Controller(FR3ControllerGains gains, FR3ControllerLimits limits);

    void set_target(const std::array<double, 7>& target_position);
    void hold(const std::array<double, 7>& measured_position);
    std::array<double, 7> compute(const FR3ControllerInput& input);
    const std::array<double, 7>& commanded_position() const { return target_position_; }

   private:
    static void add_soft_limit(double value, double lower, double upper, double margin,
                               double stiffness, double& output);

    FR3ControllerGains gains_;
    FR3ControllerLimits limits_;
    std::array<double, 7> target_position_{};
    bool initialized_ = false;
    bool position_command_active_ = false;
};
