# FR3 gripper and effector control

`openpi-control` owns one Franka Emika FR3 and, optionally, one end effector
through the same `ArmSession` and `FollowerArm` API used by the other follower
arms. One `pi_control_node` process owns every hardware connection it needs.

Three effector configurations are supported:

| `effector_model` | Connection | Hardware |
| --- | --- | --- |
| `"Robotiq"` | `RobotiqConnection.rtu(...)` / `.tcp(...)` | Robotiq 2F over Modbus |
| `"Franka_hand"` | `FrankaHandConnection(...)` | The FR3's own Franka Hand |
| `None` | — | Arm only; the node reports seven joints |

## Requirements

- libfranka is pinned to **0.21.3** and built from source by
  `scripts/build_deps.sh`.
- The FR3 must run Robot System version **5.9.0 or newer** (robot server
  protocol 10).
- The host must be able to reach the FR3 controller over Ethernet.
- The gripper must use either true Modbus RTU over a serial device or true
  Modbus TCP. RTU-over-TCP gateways are not treated as serial devices.

A real-time kernel is not required. The driver always constructs libfranka
with `franka::RealtimeConfig::kIgnore`; the torque callback still runs at the
1 kHz rate owned by libfranka.

## Python configuration

For serial RTU:

```python
from openpi_control import ArmConfig, FR3Connection, RobotiqConnection

config = ArmConfig(
    "follower",
    "FR3",
    FR3Connection("192.168.1.10"),
    effector_model="Robotiq",
    effector_connection=RobotiqConnection.rtu(
        "/dev/serial/by-id/usb-robotiq",
        baud_rate=115200,
        slave_id=9,
    ),
)
```

For Modbus TCP, only the gripper connection changes:

```python
effector_connection=RobotiqConnection.tcp("192.168.1.11", port=502)
```

The public gripper convention is `0.0 = fully closed` and `1.0 = fully open`.
Raw Robotiq register calibration defaults to 3 (open) and 230 (closed).

## Franka Hand

The hand hangs off the FR3's own controller, so its address defaults to the
arm's:

```python
from openpi_control import ArmConfig, FR3Connection, FrankaHandConnection

config = ArmConfig(
    "follower",
    "FR3",
    FR3Connection("192.168.1.10"),
    effector_model="Franka_hand",
    effector_connection=FrankaHandConnection(speed_m_s=0.05),
)
```

The same `0.0 = closed`, `1.0 = open` convention applies; the node divides the
measured finger width by the hand's own `max_width`, so a normalized position
means the same thing whatever fingers are fitted.

One connection, two threads: a reader blocked in `readOnce()` and a command
thread running the latest requested width. **The gripper server accepts exactly
one client**, so no other process may hold a connection to the hand while the
node runs -- a second connection is refused and takes the first one down with
it. The state stream runs at roughly 40 Hz at rest and 8 Hz while the fingers
travel, so a mid-stroke width is a frame or two old.

Fingers stopped by an object on the way closed is how a grasp ends and is not a
fault; an opening command that never reaches its width is. The node positions
with `move()`, which takes no force, so `force_n` is reserved for a future
`grasp()` mode. `homing` is off by default: it recalibrates the stroke but
sweeps the fingers through their full range, which is not safe to do unattended
with long fingers fitted. Without homing, `max_width` still comes from the
hand's own state stream.

## Faults

`fault_action` (`FR3Connection`) decides what a control fault -- a collision
reflex, a soft joint or velocity limit, the elbow velocity check -- does:

- `"stop"` (default): both transports park and the session ends with the arm
  where its own reflex left it. Python sees a `HardwareFaultError`.
- `"home"`: the node drives the arm to `reset_pose_rad` first. That motion
  cannot be interrupted by the client, and it carries whatever is in the
  gripper along an unplanned path.

Connecting recovers a latched reflex in place, so a session can be restarted
after a fault without moving the arm first.

Connecting is passive: the arm holds its measured pose and the gripper is not
activated. `move_to_ready()` performs internal FR3 error recovery, moves to the
configured seven-joint reset pose, activates the gripper, and waits for it to
open fully. There are no
FR3-specific activation, recovery, velocity-command, read-only, or synthetic
backend APIs.

```python
from openpi_control import ArmSession, PositionCommand

with ArmSession() as session:
    follower = session.add_follower(config)
    session.connect()
    follower.move_to_ready()
    follower.command(
        PositionCommand(
            [0.0, -0.6283185307, 0.0, -2.5132741229, 0.0, 1.8849555922, 0.0],
            1.0,
        )
    )
```

## Native controller

Policy targets may arrive at a lower frequency while libfranka continues its
1 kHz torque callback. Position targets remain active until replaced or held.

The law itself lives in one file, `src/openpi_control/models/arms/FR3/FR3_law.json`:
the joint and Cartesian impedance gains, the joint, velocity and Cartesian
limits with their soft-limit margins and push-back stiffnesses, the elbow
velocity check, the torque clamp, libfranka's torque filter cutoff and rate
limit switch, and the collision thresholds. The node loads it at startup from
`--fr3_law`, which the Python layer resolves to the packaged file, or to
`FR3Connection(law_path=...)` for an experiment, and logs the path and joint
stiffness it loaded. Nothing about the law is compiled in: a simulation that
reads the same file runs the same law, and a change to the file needs no
rebuild. The file is read strictly -- every key required, no unknown keys,
exact array lengths, finite values, every lower bound below its upper bound --
and a violation stops the node before it touches the arm.

Joint, velocity, Cartesian, torque, collision, owner-liveness, and command
shape checks remain active. libfranka supplies the robot dynamics and
kinematics, so the FR3 configuration does not require a packaged URDF.

The dependency builder produces static `libfranka.a` and `libmodbus.a`
archives. Both are linked directly into the packaged `pi_control_node`, just
like the existing pinned native dependencies.

## Hardware acceptance

Perform the first actuating checks with the workspace clear and an operator at
the E-stop:

1. Connect and verify seven joint states plus one gripper state without motion.
2. Verify the RTU or TCP gripper endpoint and the open/closed convention.
3. Call `move_to_ready()` and verify the blocking reset motion and activation.
4. Send low-rate position targets and verify the 1 kHz callback remains healthy.
5. Terminate the Python owner and verify that the arm holds and gripper motion stops.
