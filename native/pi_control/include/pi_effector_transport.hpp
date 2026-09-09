#pragma once

#include <cstdint>

/*!
 * @brief One end effector's state, in the units the device layer publishes.
 *
 * ``position`` is normalized: 0 is fully closed and 1 is fully open, whatever
 * the hardware's own units are. Unmeasured fields retain their documented
 * defaults rather than being invented.
 */
struct EffectorTransportState {
    bool connected = false;   ///< The transport is talking to the hardware.
    bool activated = false;   ///< The effector is ready to accept commands.
    bool moving = false;      ///< A command is executing.
    float position = 1.0f;    ///< Normalized opening: 0 closed, 1 open.
    float velocity = 0.0f;    ///< Normalized opening per second.
    float effort = 0.0f;      ///< Effort in the effector's own units, 0 if unmeasured.
    float current = 0.0f;     ///< Motor current (A), 0 if unmeasured.
    float target = 1.0f;      ///< Normalized opening most recently commanded.
    float frame_age_ms = -1.0f; ///< Age of cached feedback; -1 when untracked.
    uint8_t fault = 0;        ///< Hardware fault code, 0 when healthy.
};

/*!
 * @brief The end effector interface DeviceFR3 drives.
 *
 * Both implementations own their hardware connection and a thread that talks
 * to it, so the 1 kHz arm controller never blocks on gripper I/O: commands are
 * handed over and executed asynchronously, and state is published back.
 */
class EffectorTransport {
   public:
    virtual ~EffectorTransport() = default;

    /*! @brief Opens the connection and starts the transport threads. */
    virtual bool start() = 0;

    /*! @brief Stops the threads and closes the connection. */
    virtual void stop() = 0;

    /*! @brief Brings the effector to a commandable state. Blocking. */
    virtual bool activate() = 0;

    /*!
     * @brief Requests a normalized opening.
     * @param position Normalized opening: 0 closed, 1 open.
     * @param speed Normalized speed in [0, 1] of the effector's maximum.
     * @param force Normalized force in [0, 1]; effectors without force control
     *              ignore it.
     */
    virtual void set_target(float position, float speed, float force) = 0;

    /*! @brief Abandons any command that has not started executing yet. */
    virtual void hold() = 0;

    /*! @brief The most recently published state. */
    virtual EffectorTransportState effector_state() const = 0;

    /*! @brief Whether the effector reports a fault that stops the session. */
    virtual bool has_effector_fault() const = 0;
};
