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

- passive boot with no motor enable, zero capture, or position command
- explicit six-motor initialization through `init6`
- `status6`, `hold6`, and `disarm6` lifecycle commands
- latched CAN/heartbeat fault detection reported through `fault6`
- synchronized 6-joint packets from ROS 2
- acceleration-limited smoothing in the bridge
- velocity/acceleration-limited smoothing in the ESP32 firmware
- fixed-rate target re-streaming on the ESP32 to reduce chopped motion
- joint4 MoveIt targets are relative to the captured software zero, not the ZE300 hardware zero
- the first joint4 command is limited to +/-30 degrees from captured zero to catch bad startup poses
- coordinated motion is rejected until all motors and relative zeros are ready

## Passive boot and initialization

At power-up the ESP32 configures serial, SPI, MCP2515 filters, and internal
state only. It reports:

```text
boot6 ready=true motors_initialized=false passive=true
```

It does not request ODrive closed-loop control or communicate motion/zero
commands to ZE300 or LKTech motors. Run initialization explicitly through ROS:

```bash
ros2 service call /moveit_arm_bridge_6dof/initialize_motors std_srvs/srv/Trigger "{}"
ros2 service call /moveit_arm_bridge_6dof/rearm std_srvs/srv/Trigger "{}"
```

## Lifecycle serial commands

```text
init6       initialize all motors and capture six relative zeros
status6     report firmware, CAN, motor-ready, zero-valid, and fault state
hold6       stop coordinated target updates and retain the last driver targets
disarm6     best-effort disable/stop all reachable motors; re-init required
jx6         legacy disable/loose command; re-init required (not hold)
cfs6 on|off enable strict CAN fault thresholds for bench validation
testfault6 can_bus_lost  inject a latched fault; requires cfs6 on
```

CAN faults are latched as `fault6 ...`; new `j6` targets are rejected until a
new explicit `init6` succeeds. If CAN is physically disconnected, firmware can
detect and report the loss but cannot guarantee that stop commands reach the
motor drivers.

`status6` decodes MCP2515 EFLG bits and reports total/windowed send failures,
incomplete bursts, last failed ID and library code, per-joint failure counts,
and ODrive heartbeat ages. With the installed `autowp-mcp2515` library,
`0x80` is `RX1OVR`, return code `2` is `ERROR_ALLTXBUSY`, and return code `4`
is `ERROR_FAILTX`.
