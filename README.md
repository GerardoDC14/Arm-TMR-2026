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

For Jaguar, that controller output is already bridged to real hardware.
For Dicerox, the current output path is the MoveIt mock controller used in RViz / fake hardware.

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

### Dicerox launch sequence

```bash
# 1. MoveIt + RViz + mock ros2_control
ros2 launch dicerox_moveit demo.launch.py

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
- The Dicerox hardware bridge from MoveIt output to the real mixed CAN motor stack is not finished yet. The repo already contains the lower-level work under `src/dicerox/bldc_can_tools` and its ESP32 examples.

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
