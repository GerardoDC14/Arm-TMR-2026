# `src/bldc_can_tools`

`src/bldc_can_tools` is a Python `ament_python` package for testing multiple BLDC CAN driver families, including LKTech or LingKong motors and ZE300-style drivers.

The code is intentionally split into:

- a dedicated Ginkgo CAN transport layer
- centralized LKTech protocol helpers
- room for additional driver-family protocol helpers such as ZE300-style drivers
- a high-level motor driver with software zero-offset handling
- direct Python CLIs and a practical CAN sniffer
- optional ROS 2 nodes and launch/config files

This package is written so the direct Python tools are useful on Windows even if you are not using ROS 2 right now.

## Current backend status

This workspace already contains a real Ginkgo vendor ctypes binding and Windows DLL bundle under:

- `src/ginkgo_odrive_bridge/Python_USB_CAN_Test_64bits/ControlCAN.py`
- `src/ginkgo_odrive_bridge/Python_USB_CAN_Test_64bits/lib/windows/...`

The new package uses that real SDK when it is available instead of inventing a fake Ginkgo API. The wrapper in this package still stays isolated and clean, so the rest of the code only depends on `ginkgo_can_interface.py`.

Important:

- Ginkgo backend availability still depends on matching Python architecture and vendor DLL availability.
- The workspace copy I found includes Windows and macOS binaries. Linux libraries were not visible in the checkout I inspected.

## Protocol honesty

The LKTech protocol helpers in this package are based on a MyActuator or LingKong style command family that appears consistent with:

- `0x92` read multi-loop angle
- `0x94` read single-loop angle
- `0x9A` read state 1
- `0x9C` read state 2
- `0x9D` read state 3
- `0xA4` multi-loop or absolute position control with speed limit
- `0xA6` single-loop position control with speed limit

Some LKTech-specific details may still differ by motor firmware, manual revision, or CAN ID mapping. Comments and docstrings call out these assumptions where relevant. You should verify them on real hardware before trusting them for anything safety-critical.

## Build

From the workspace root:

```bash
colcon build --packages-select bldc_can_tools
```

## Source And Run

Linux ROS 2 shell:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Windows ROS 2 prompt:

```powershell
call install\setup.bat
```

For direct Python usage on Windows, you can also run the tools from inside `src/bldc_can_tools` without building the ROS package first, as long as Python can see the package folder and the Ginkgo vendor SDK can load.

## Windows-First Direct Python Usage

Read the current angle once:

```powershell
python -m bldc_can_tools.cli_read_angle --channel 0 --bitrate 1000 --motor-id 15
```

Read the current angle and treat startup position as software zero:

```powershell
python -m bldc_can_tools.cli_read_angle --channel 0 --bitrate 1000 --motor-id 15 --capture-boot-offset
```

Send a motor-side target directly:

```powershell
python -m bldc_can_tools.cli_position_test --channel 0 --bitrate 1000 --motor-id 15 --motor-deg 180 --speed-dps-motor 90
```

Send a joint-side target that is converted through the reduction ratio and boot offset:

```powershell
python -m bldc_can_tools.cli_position_test --channel 0 --bitrate 1000 --motor-id 15 --reduction-ratio 6 --joint-deg 30 --speed-dps-joint 20
```

## Boot-Offset Logic

This package does not ask the motor to redefine its zero in RAM.

Instead, the high-level driver uses a pure software offset:

1. On startup, read the current multi-loop motor angle.
2. Store that value as `boot_offset_motor_deg`.
3. Define the current joint angle as:

```text
joint_angle_deg = (current_motor_deg - boot_offset_motor_deg) / reduction_ratio
```

4. When commanding a joint target relative to startup zero, convert it back to a motor target:

```text
motor_target_deg = boot_offset_motor_deg + desired_joint_deg * reduction_ratio
```

That means the position at startup becomes software zero.

## Joint Degrees Versus Motor Degrees

- Joint or output degrees are the user-facing quantity.
- Motor-side degrees are what the protocol command uses.
- The conversion uses `reduction_ratio`.
- Speed limits for joint commands are also scaled up internally to motor-side speed before the `0xA4` command is built.

## Monitor Node

If you later want ROS 2 monitoring again:

```bash
ros2 run bldc_can_tools lktech_monitor --ros-args --params-file \
  $(ros2 pkg prefix bldc_can_tools)/share/bldc_can_tools/config/default_params.yaml
```

Published topics:

- `/lktech/current_motor_deg`
- `/lktech/current_joint_deg`
- `/lktech/boot_offset_motor_deg`

Optional:

- `sensor_msgs/msg/JointState` for one joint

## Publish A Test Target Joint Angle

```bash
ros2 topic pub --once /lktech/target_joint_deg std_msgs/msg/Float64 "{data: 15.0}"
```

## Direct CLI Tools

Read angle:

```powershell
python -m bldc_can_tools.cli_read_angle --help
```

Send one target:

```powershell
python -m bldc_can_tools.cli_position_test --help
```

## Run `cansniffer.py`

Directly from the package root:

```powershell
python scripts/cansniffer.py --channel 0 --bitrate 1000 --decode raw
```

LKTech-aware decode mode:

```powershell
python scripts/cansniffer.py --channel 0 --bitrate 1000 --decode lktech --motor-id 15
```

Log frames to a file:

```powershell
python scripts/cansniffer.py --channel 0 --bitrate 1000 --decode lktech --log-file can_log.txt
```

## Notes You Must Verify On Hardware

- The exact arbitration ID mapping is assumed to be `0x140 + motor_id` for requests and `0x240 + motor_id` for replies.
- `0xA4` is treated as the correct multi-loop angle control with speed limit for this motor family.
- The `0xA6` single-loop control helper is included as a scaffold and needs hardware confirmation before relying on it.
- The meaning of returned angle fields in status replies may vary slightly across firmware revisions.
- Ginkgo adapter access depends on the vendor SDK loading correctly on your machine.

