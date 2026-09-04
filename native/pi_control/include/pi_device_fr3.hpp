#pragma once

#include <memory>

#include "pi_device.hpp"
#include "pi_driver_fr3.hpp"
#include "pi_robotiq.hpp"

class DeviceFR3 final : public Device {
   public:
    explicit DeviceFR3(const CommandLineArgs& cla);
    ~DeviceFR3() override;

    ReturnCode init(const CommandLineArgs& cla, int argc, char** argv,
                    std::shared_ptr<Topic> topic = nullptr,
                    std::shared_ptr<Driver> driver = nullptr) override;
    ReturnCode start(int baud_rate) override;
    ReturnCode stop() override;
    ReturnCode park_safely() override;
    ReturnCode apply_action(const MsgJoints& msg) override;
    ReturnCode get_observation(MsgJoints& msg) override;
    ReturnCode process_follower_msg(const MsgJoints& msg) override;
    ReturnCode read_hardware_values() override;
    ReturnCode write_hardware_values() override;
    ReturnCode move_to_ready_position() override;
    ReturnCode operate_as_leader() override;
    ReturnCode operate_as_follower() override;
    ReturnCode get_servo_ids(std::vector<int>& servo_ids) override;
    ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) override;
    ReturnCode runtime_hold() override;

   protected:
    void reset_ready_state_for_move_to_ready() override { is_ready_ = false; }
    void clear_command_buffers_for_move_to_ready() override;
    float get_ready_move_completion_ratio() const override { return is_ready_ ? 1.0f : 0.0f; }

   private:
    /*!
     * @brief Applies the configured fault policy (--fr3_fault_action).
     *
     * "home" keeps the inherited behaviour: HARDWARE_FAULT sends the device
     * into emergency recovery, which drives the arm to the reset pose.
     * "stop" (the default) parks both transports and ends the session with the
     * arm where its own reflex left it. A joint-space move to the reset pose
     * after a collision is not something the client can interrupt -- the node
     * is blocked inside robot->control for the whole motion -- and it carries
     * whatever is in the gripper along an unplanned path.
     */
    ReturnCode handle_fault();

    std::shared_ptr<DriverFR3> driver_fr3_;
    std::unique_ptr<RobotiqTransport> robotiq_;
    float robotiq_default_speed_ = 1.0f;
    float robotiq_default_force_ = 1.0f;
};
