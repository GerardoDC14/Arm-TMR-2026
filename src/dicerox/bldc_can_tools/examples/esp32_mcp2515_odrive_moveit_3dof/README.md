# ESP32 + MCP2515 ODrive MoveIt 3-DOF Controller

This sketch is a dedicated copy of the stable ODrive sniffer/controller for
MoveIt-driven joints `1-3`.

Targets:

- `0x10` = physical Joint 1
- `0x11` = physical Joint 2
- `0x12` = physical Joint 3

Use it together with:

- `ros2 run bldc_can_tools moveit_odrive_bridge_3dof`

The ROS 2 bridge reads `/joint_states`, converts MoveIt radians to physical
degrees, applies the flipped joint mappings, and sends `1/2/3` + `g <deg>`
commands over serial to this sketch.
