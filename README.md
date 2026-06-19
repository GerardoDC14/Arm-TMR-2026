# Jaguar + Dicerox Arm Workspace

This repository now keeps both robots in one ROS 2 / MoveIt workspace:

- `Jaguar`: full MoveIt Servo teleop stack plus hardware bridges already wired into the repo
- `Dicerox`: MoveIt + RViz + MoveIt Servo teleop path is now in place, while the low-level mixed-driver hardware bridge is still a work in progress

## Repo layout

```text
src/
  jaguar/
    jaguar_robot_full_description/
    jaguar_full/
    jaguar_teleop/
  dicerox/
    dicerox_arm_urdf/
    dicerox_moveit/
    dicerox_urdf_v1/
    bldc_can_tools/
    arm_dicerox_ws/          # legacy nested workspace artifacts, ignored by colcon
  ginkgo_odrive_bridge/
firmware/
requirements.txt
```

`jaguar_teleop` keeps its legacy package name, but the joystick and keyboard MoveIt Servo nodes are now parameterized and reused by both robots.

## Build

```bash
source /opt/ros/humble/setup.bash
pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
```

Arduino libraries used by the Jaguar ESP32 sketches:

- `ESP32Servo` by Kevin Harrington
- `ArduinoJson` by Benoit Blanchon

## Shared control flow

Both robots now follow the same high-level MoveIt Servo input path:

```text
Joystick / Keyboard / SpaceMouse
        │
        └──→ jaguar_teleop
               ├──→ /servo_node/delta_twist_cmds
               └──→ /servo_node/delta_joint_cmds
                         │
                         └──→ moveit_servo
                                  │
                                  └──→ /<robot>_arm_controller/joint_trajectory
                                           │
                                           └──→ ros2_control JointTrajectoryController
```

Jaguar uses its existing hardware bridges. Dicerox has separate simulation and
hardware launch paths; hardware execution uses a `FollowJointTrajectory` action
server in the Python bridge rather than the mock controller.

## Jaguar

Jaguar is the most complete stack in this repo:

- `jaguar_robot_full_description`: arm URDF, meshes, RViz assets
- `jaguar_full`: MoveIt config, Servo config, launch files, controller config
- `jaguar_teleop`: shared joystick / keyboard servo inputs, plus Jaguar serial bridge
- `ginkgo_odrive_bridge`: ROS 2 bridge from `/joint_states` to ODrive CAN through Ginkgo USB-CAN

### Jaguar launch sequence

```bash
# 1. MoveIt + RViz + mock ros2_control
ros2 launch jaguar_full demo.launch.py

# 2. MoveIt Servo
ros2 launch jaguar_full servo.launch.py

# 3. ODrive CAN bridge for joints 1-3
ros2 launch ginkgo_odrive_bridge joint_state_bridge.launch.py verbose:=true verbose_period_s:=0.5

# 4a. Joystick teleop + Jaguar serial bridge
ros2 launch jaguar_teleop joystick.launch.py

# 4b. Keyboard teleop + Jaguar serial bridge
ros2 launch jaguar_teleop keyboard.launch.py

# 4c. SpaceMouse (Space Explorer) — all-in-one: demo + servo + spacemouse node
ros2 launch jaguar_full spacemouse.launch.py
```

If the ESP32 is on a different port:

```bash
ros2 launch jaguar_teleop joystick.launch.py port:=/dev/ttyUSB1
```

### Jaguar hardware mapping

| Joint | Actuator | Interface |
|-------|----------|-----------|
| Joint 1-3 | ODrive brushless | Ginkgo USB-CAN |
| Joint 4-6 | Servos | ESP32 serial bridge |
| Gripper | Servo | ESP32 serial bridge |

`serial_bridge.py` listens to `/joint_states` and `/joy`, then streams JSON to the ESP32 for joints 4-6 plus the gripper. The ESP32 conversion and calibration live under `firmware/servo_sweep/`.

## Dicerox

Dicerox is now laid out as a normal sibling robot folder instead of a nested workspace inside the workspace:

- `dicerox_arm_urdf`: arm description package
- `dicerox_urdf_v1`: chassis / flipper description package
- `dicerox_moveit`: combined MoveIt config, RViz demo, Servo config, and teleop launches
- `bldc_can_tools`: mixed-driver CAN utilities, ROS nodes, and ESP32 examples for Dicerox hardware bringup

### Dicerox architecture

```text
ROS 2 / MoveIt
  -> Python bridge
  -> USB serial
  -> ESP32
  -> MCP2515 CAN
  -> six motor controllers
```

The six arm joints use three controller families on one CAN bus: ODrive for
joints 1-3, ZE300 for joint 4, and LKTech/MyActuator for joints 5-6. MoveIt sends
the complete trajectory to `/dicerox_arm_controller/follow_joint_trajectory`.
The bridge samples it at 50 Hz, applies per-joint velocity and acceleration
limits, and streams `j6` targets. The ESP32 applies a second ramp before sending
commands to the motor drivers.

### Dicerox simulation

```bash
ros2 launch dicerox_moveit demo.launch.py
```

Simulation continues to use the fake/mock `joint_trajectory_controller`. Do not
run simulation and hardware launches together because both would try to own the
same `FollowJointTrajectory` action name.

### Dicerox hardware

Hardware trusted-open-loop launch:

```bash
ros2 launch dicerox_moveit hardware.launch.py \
  dry_run:=false \
  enable_hardware_motion:=true \
  joint_state_source_mode:=trusted_open_loop
```

The hardware launch does not start the mock controller. Motor initialization is
also disabled by default; launching ROS alone cannot enable or move the motors.

Normal operating sequence:

```bash
ros2 service call /moveit_arm_bridge_6dof/status std_srvs/srv/Trigger "{}"

ros2 service call /moveit_arm_bridge_6dof/initialize_motors \
  std_srvs/srv/Trigger "{}"

ros2 service call /moveit_arm_bridge_6dof/rearm \
  std_srvs/srv/Trigger "{}"
```

After both initialization and rearm succeed, trajectories can be planned and
executed from RViz. To intentionally remove holding torque:

```bash
ros2 service call /moveit_arm_bridge_6dof/disarm std_srvs/srv/Trigger "{}"
```

Disarm invalidates motor readiness. Another initialization and rearm are
required before motion can resume.

### Boot and safety lifecycle

```text
Power ON
-> ESP32 passive boot
-> MOTORS_UNINITIALIZED
-> /initialize_motors
-> MOTORS_READY / REQUIRES_REARM
-> /rearm
-> ACTIVE
-> MoveIt execution allowed
```

The ESP32 `setup()` path configures communication and internal state only. It
does not request ODrive closed-loop control, capture zeros, or command ZE300 or
LKTech motors. Initialization occurs only after explicit `init6`. `/rearm`
fails until `init6` has completed successfully.

Bridge states include `STARTUP`, `MOTORS_UNINITIALIZED`,
`MOTORS_INITIALIZING`, `REQUIRES_REARM`, `ACTIVE`, `FAULT`, and `RECOVERY`.
An ESP32 reset or serial reconnection invalidates readiness and prevents stale
targets from resuming automatically.

Relevant launch parameters and safe defaults:

```text
auto_initialize_motors := false
motor_initialization_timeout_sec := 20.0
firmware_status_timeout_sec := 2.0
trusted_open_loop_hold_after_action := true
trusted_open_loop_hold_rate_hz := 10.0
idle_on_action_cancel := false
can_fault_strict_debug := false
```

### Serial commands

```text
init6
status6
hold6
disarm6
jx6
j6 <j1_deg> <j2_deg> <j3_deg> <j4_deg> <j5_deg> <j6_deg>
cfs6 on|off
testfault6 can_bus_lost
```

- `init6` initializes all motor controllers and captures relative zero references.
- `status6` reports firmware, motor, zero-reference, CAN, and fault state.
- `hold6` stops coordinated updates but retains the existing holding targets.
- `disarm6` disables/stops reachable motors and invalidates motion readiness.
- `jx6` is a disable/loose command, not a hold command.
- `j6 ...` updates the coordinated six-joint target; firmware ramping prevents jumps.
- `cfs6 on|off` enables stricter CAN fault thresholds for bench validation.
- `testfault6 can_bus_lost` injects a fault and requires strict debug mode.

ROS lifecycle services:

```text
/moveit_arm_bridge_6dof/initialize_motors
/moveit_arm_bridge_6dof/status
/moveit_arm_bridge_6dof/rearm
/moveit_arm_bridge_6dof/disarm
/moveit_arm_bridge_6dof/inject_can_fault
```

### Trusted open loop

In `trusted_open_loop`, `/joint_states` are commanded states published for
MoveIt/RViz synchronization. They are not measured encoder feedback. MoveIt
success means the trajectory was validated, sampled, limited, and transmitted;
it does not verify the final physical pose.

After an action, the bridge continues sending the final target at 10 Hz by
default. Action cancellation does not automatically loosen the arm. Use
`/disarm` when removing holding torque is intentional.

### CAN faults

Firmware monitors MCP2515/CAN errors, command-send failures, incomplete motor
bursts, and ODrive heartbeat timeouts. It emits `fault6 ...` and latches
`FAULT_CAN`; the bridge then enters `FAULT`, aborts active execution, and blocks
new motion until explicit initialization and rearm.

If the CAN bus is physically disconnected, the ESP32 cannot guarantee that a
stop or disable command reaches the motor drivers. Mechanical support and
hardware-level emergency protection remain necessary.

`status6` includes decoded MCP2515 EFLG flags, total and in-window send
failures, incomplete burst count, last failed CAN ID/code/name, per-joint
failure counters, and ODrive heartbeat ages. A non-motion propagation test is:

```bash
ros2 launch dicerox_moveit hardware.launch.py \
  dry_run:=false enable_hardware_motion:=true \
  joint_state_source_mode:=trusted_open_loop can_fault_strict_debug:=true

ros2 service call /moveit_arm_bridge_6dof/inject_can_fault \
  std_srvs/srv/Trigger "{}"
```

The injected `fault6 can_bus_lost injected=true` follows the real bridge fault
path and requires another initialization and rearm.

### Dicerox teleoperation

The following commands operate the simulation/Servo path and are separate from
the hardware lifecycle above:

```bash

# 2. MoveIt Servo
ros2 launch dicerox_moveit servo.launch.py

# 3a. Joystick teleop
ros2 launch dicerox_moveit joystick.launch.py

# 3b. Keyboard teleop
ros2 launch dicerox_moveit keyboard.launch.py

# 3c. SpaceMouse (Space Explorer) — all-in-one: demo + servo + spacemouse node
ros2 launch dicerox_moveit spacemouse.launch.py
```

### Dicerox notes

- The MoveIt model includes the arm and the chassis flippers, but MoveIt Servo is configured for the `dicerox_arm` group only.
- `dicerox_moveit` now has its own `servo.launch.py` and `servo_params.yaml`, mirroring the Jaguar workflow.
- The hardware bridge is the only `FollowJointTrajectory` action server in hardware mode; simulation retains mock-controller support.

## Input mapping

### Joystick

| Input | Effect |
|-------|--------|
| Left stick Y | Forward / Back (+X) |
| Left stick X | Strafe (+Y) |
| Right stick Y | Up / Down (+Z) |
| Right stick X | Yaw |
| LB + Left X | Roll |
| RB + Right Y | Pitch |
| Start | Toggle cartesian / joint mode |
| D-pad Up/Down | Select active joint in joint mode |
| Left stick Y | Jog selected joint in joint mode |
| Y | Pause / resume MoveIt Servo |
| Guide (hold) | Emergency stop |
| Back | Publish zero command |

### Keyboard

| Input | Effect |
|-------|--------|
| `w/s` | X translation in cartesian mode or jog active joint in joint mode |
| `a/d` | Y translation |
| `r/f` | Z translation |
| `u/j` | Roll |
| `i/k` | Pitch |
| `o/l` | Yaw |
| `1`-`6` | Select active joint |
| `m` | Toggle cartesian / joint mode |
| `space` or `x` | Stop |
| `+` / `-` | Adjust speed |

### Space Explorer 3D mouse

The SpaceMouse controller (`spacemouse_servo`) provides simultaneous 6-DOF Cartesian control of the end effector. All six axes are live at the same time — no mode switching required.

| Puck gesture | Effect |
|---|---|
| Push forward / back | EEF +X / -X |
| Push left / right | EEF +Y / -Y |
| Push up / down | EEF +Z / -Z |
| Tilt left / right | Roll (angular X) |
| Tilt forward / back | Pitch (angular Y) |
| Twist CW / CCW | Yaw (angular Z) |
| RIGHT button | Pause / resume MoveIt Servo |

**Axis filtering** — three stages prevent mixed commands caused by physical coupling:

1. **Noise floor** (`deadband=0.05`) — per-axis hard cutoff to eliminate sensor noise at rest.
2. **Within-group suppression** (`axis_relative_threshold=0.40`) — inside each group (translation or rotation), axes below 40 % of the group peak are zeroed.
3. **Group dominance** (`dominance_threshold=1.5`) — if one group's magnitude exceeds the other by more than 1.5×, the weaker group is zeroed entirely.

**udev rule** — the Space Explorer ships without world-readable permissions on most distros. Add a rule so any logged-in user can open it:

```bash
echo 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c627", MODE="0660", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-spacemouse.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG plugdev $USER   # log out and back in after this
```

## Firmware

- `firmware/servo_sweep/`: Jaguar ESP32 sketch for joints 4-6 and gripper
- `firmware/joint5_servo/`: Jaguar joint 5 standalone calibration sketch
- `src/dicerox/bldc_can_tools/examples/`: Dicerox ESP32 + MCP2515 experiments, including the 6-joint mixed CAN controller

## Package overview

| Package | Scope | Role |
|---------|-------|------|
| `jaguar_robot_full_description` | Jaguar | Arm URDF, meshes, RViz assets |
| `jaguar_full` | Jaguar | MoveIt config, controllers, Servo launch |
| `jaguar_teleop` | Shared | Shared joystick / keyboard / SpaceMouse MoveIt Servo inputs, plus Jaguar serial bridge |
| `ginkgo_odrive_bridge` | Shared / Jaguar | ODrive CAN bridge for real hardware |
| `dicerox_arm_urdf` | Dicerox | Arm URDF and meshes |
| `dicerox_urdf_v1` | Dicerox | Chassis / flipper URDF and meshes |
| `dicerox_moveit` | Dicerox | Combined MoveIt config, Servo config, joystick / keyboard launches |
| `bldc_can_tools` | Dicerox | Mixed BLDC CAN tooling and bringup examples |
