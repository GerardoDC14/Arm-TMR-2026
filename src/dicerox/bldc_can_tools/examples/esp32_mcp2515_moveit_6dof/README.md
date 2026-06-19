# ESP32 + MCP2515 MoveIt 6-DOF Controller

This sketch combines the working direct-position paths for all six arm joints:

- ODrive joint1 `0x10`
- ODrive joint2 `0x11`
- ODrive joint3 `0x12`
- ZE300 joint4 on CAN address `13`
- LKTech joint5 on CAN ID `0x14E`
- LKTech joint6 on CAN ID `0x14F`

Use it together with:

- `ros2 run bldc_can_tools moveit_arm_bridge_6dof`

MoveIt command line used by the bridge:

- `j6 <joint1_deg> <joint2_deg> <joint3_deg> <joint4_deg> <joint5_deg> <joint6_deg>`

Highlights:

- automatic ODrive bringup at boot
- automatic ZE300 joint4 speed setup and software-zero capture at boot
- automatic LKTech motor-on and zero capture at boot
- synchronized 6-joint packets from ROS 2
- acceleration-limited smoothing in the bridge
- fixed-rate target re-streaming on the ESP32 to reduce chopped motion
- joint4 MoveIt targets are relative to the captured software zero, not the ZE300 hardware zero
- the first joint4 command is limited to +/-30 degrees from captured zero to catch bad startup poses
- heartbeat gaps first become a degraded warning; the ESP32 keeps feeding the last safe ODrive target during that window
- only prolonged heartbeat loss or a confirmed ODrive fault/disarm escalates to a true ESP32 failsafe
