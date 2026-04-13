# ESP32 + MCP2515 MoveIt 4-DOF Controller

This sketch combines:

- ODrive joints `0x10`, `0x11`, `0x12`
- ZE300 joint4 on CAN address `13`

Use it together with:

- `ros2 run bldc_can_tools moveit_arm_bridge_4dof`

MoveIt command line used by the bridge:

- `j4 <joint1_deg> <joint2_deg> <joint3_deg> <joint4_deg>`

The sketch keeps the proven ODrive motion path for joints `1-3` and adds a
non-blocking ZE300 absolute-position command for joint4.
