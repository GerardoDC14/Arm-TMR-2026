from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import List, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Int8
from std_srvs.srv import Trigger

from . import spacenavigator_hidapi as spnav

# Physical buttons on the Space Explorer
BTN_RIGHT = 1   # right side button → pause / resume MoveIt Servo

_STATUS = {
    1: 'WARNING  Approaching singularity — slowing down',
    2: 'HALT     Singularity reached — halted',
    3: 'WARNING  Leaving singularity',
    4: 'HALT     Collision detected — halted',
    5: 'WARNING  Near joint limit',
}


@dataclass
class MotionState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    t: float = 0.0
    buttons: List[int] = field(default_factory=list)


class SpaceMouseServoNode(Node):
    def __init__(self):
        super().__init__("spacemouse_servo")

        self.declare_parameter("robot_name",              "Robot")
        self.declare_parameter("twist_topic",             "/servo_node/delta_twist_cmds")
        self.declare_parameter("frame_id",                "base_link")
        self.declare_parameter("publish_rate_hz",         50.0)
        self.declare_parameter("linear_scale",            0.20)
        self.declare_parameter("angular_scale",           0.35)
        self.declare_parameter("deadband",                0.05)   # noise floor per axis
        self.declare_parameter("axis_relative_threshold", 0.40)   # minor-axis suppression within group
        self.declare_parameter("dominance_threshold",     1.5)    # group dominance ratio
        self.declare_parameter("command_timeout",         0.20)
        self.declare_parameter("invert_x",                False)
        self.declare_parameter("invert_y",                False)
        self.declare_parameter("invert_z",                False)
        self.declare_parameter("invert_roll",             False)
        self.declare_parameter("invert_pitch",            False)
        self.declare_parameter("invert_yaw",              False)
        self.declare_parameter("pause_service",           "/servo_node/pause_servo")
        self.declare_parameter("start_service",           "/servo_node/start_servo")
        self.declare_parameter("status_topic",            "/servo_node/status")

        self.robot_name          = str(self.get_parameter("robot_name").value)
        self.twist_topic         = self.get_parameter("twist_topic").value
        self.frame_id            = self.get_parameter("frame_id").value
        self.publish_rate_hz     = float(self.get_parameter("publish_rate_hz").value)
        self.linear_scale        = float(self.get_parameter("linear_scale").value)
        self.angular_scale       = float(self.get_parameter("angular_scale").value)
        self.deadband            = float(self.get_parameter("deadband").value)
        self.axis_relative_threshold = float(self.get_parameter("axis_relative_threshold").value)
        self.dominance_threshold = float(self.get_parameter("dominance_threshold").value)
        self.command_timeout     = float(self.get_parameter("command_timeout").value)
        self.axis_signs = {
            "x":     -1.0 if bool(self.get_parameter("invert_x").value)     else 1.0,
            "y":     -1.0 if bool(self.get_parameter("invert_y").value)     else 1.0,
            "z":     -1.0 if bool(self.get_parameter("invert_z").value)     else 1.0,
            "roll":  -1.0 if bool(self.get_parameter("invert_roll").value)  else 1.0,
            "pitch": -1.0 if bool(self.get_parameter("invert_pitch").value) else 1.0,
            "yaw":   -1.0 if bool(self.get_parameter("invert_yaw").value)   else 1.0,
        }

        pause_svc    = self.get_parameter("pause_service").value
        start_svc    = self.get_parameter("start_service").value
        status_topic = self.get_parameter("status_topic").value

        self.publisher  = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self._pause_cli = self.create_client(Trigger, pause_svc)
        self._start_cli = self.create_client(Trigger, start_svc)
        self.create_subscription(Int8, status_topic, self._status_cb, 10)

        self.last_state: MotionState  = MotionState()
        self.last_publish_nonzero     = False
        self._servo_paused            = False
        self._last_status             = 0
        self._prev_buttons: List[int] = []
        self._tick                    = 0
        self._last_active_axes        = set()
        self._last_was_moving         = False
        self._last_print_tick         = 0

        try:
            spnav.open()
        except PermissionError as exc:
            self.get_logger().error(str(exc))
            raise
        except Exception as exc:
            self.get_logger().error(f"Failed to open SpaceMouse: {exc}")
            raise

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.02
        self.create_timer(period, self._poll_and_publish)
        self._print_help()

    # ── Help ──────────────────────────────────────────────────────────────

    def _print_help(self):
        print('\n' + '=' * 60)
        print(f'  {self.robot_name} — Space Explorer Servo  (6-DOF)')
        print('=' * 60)
        print('  Translation  (puck push)')
        print('    Push fwd / back    → EEF  +X / -X')
        print('    Push left / right  → EEF  +Y / -Y')
        print('    Push up  / down    → EEF  +Z / -Z')
        print()
        print('  Rotation  (puck tilt / twist)')
        print('    Tilt left / right  → Roll  (angular X)')
        print('    Tilt fwd  / back   → Pitch (angular Y)')
        print('    Twist CW  / CCW    → Yaw   (angular Z)')
        print()
        print('  RIGHT button   Pause / Resume MoveIt Servo')
        print('=' * 60 + '\n')

    # ── Servo status ──────────────────────────────────────────────────────

    def _status_cb(self, msg: Int8):
        code = msg.data
        if code == self._last_status:
            return
        self._last_status = code
        text = _STATUS.get(code)
        if text:
            self.get_logger().warn(f'[SERVO] {text}')
        elif code == 0:
            self.get_logger().info('[SERVO] OK')

    # ── Pause / resume ────────────────────────────────────────────────────

    def _call_service(self, client, label: str):
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=0.25):
            self.get_logger().warn(f'{label} service not available')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'{label}: {f.result().message}' if f.result() else f'{label}: no response'
            )
        )

    def _toggle_pause(self):
        if self._servo_paused:
            self._servo_paused = False
            self._call_service(self._start_cli, 'start_servo')
            self.get_logger().info('Servo RESUMED — SpaceMouse active')
        else:
            self._publish_zero()
            self._servo_paused = True
            self._call_service(self._pause_cli, 'pause_servo')
            self.get_logger().info('Servo PAUSED — RViz / move_group active')

    # ── Button edge detection ─────────────────────────────────────────────

    def _handle_buttons(self, buttons: List[int]):
        if not self._prev_buttons:
            self._prev_buttons = [0] * len(buttons)

        def just_pressed(idx: int) -> bool:
            return (
                idx < len(buttons)
                and buttons[idx] == 1
                and self._prev_buttons[idx] == 0
            )

        if just_pressed(BTN_RIGHT):
            threading.Thread(target=self._toggle_pause, daemon=True).start()

        self._prev_buttons = list(buttons)

    # ── Filtering ─────────────────────────────────────────────────────────

    def _noise_floor(self, value: float) -> float:
        """Remove per-axis noise below the absolute deadband."""
        if abs(value) < self.deadband:
            return 0.0
        return max(min(value, 1.0), -1.0)

    @staticmethod
    def _suppress_minor(vals: List[float], threshold: float) -> List[float]:
        """Within a group, zero out axes below `threshold * max_magnitude`."""
        peak = max(abs(v) for v in vals)
        if peak < 1e-6:
            return vals
        return [v if abs(v) >= peak * threshold else 0.0 for v in vals]

    def _filter_command(
        self,
        x: float, y: float, z: float,
        r: float, p: float, yw: float,
    ):
        """Apply within-group minor-axis suppression then group dominance."""
        x, y, z  = self._suppress_minor([x, y, z],  self.axis_relative_threshold)
        r, p, yw = self._suppress_minor([r, p, yw], self.axis_relative_threshold)

        lin_mag = (x**2 + y**2 + z**2) ** 0.5
        ang_mag = (r**2 + p**2 + yw**2) ** 0.5
        thr     = self.dominance_threshold

        if lin_mag > ang_mag * thr:
            r = p = yw = 0.0
        elif ang_mag > lin_mag * thr:
            x = y = z = 0.0

        return x, y, z, r, p, yw

    def _latest_state(self) -> Optional[MotionState]:
        state = spnav.read()
        if state is None:
            return None

        return MotionState(
            x=    self._noise_floor(self.axis_signs["x"]     * state.x),
            y=    self._noise_floor(self.axis_signs["y"]     * state.y),
            z=    self._noise_floor(self.axis_signs["z"]     * state.z),
            roll= self._noise_floor(self.axis_signs["roll"]  * state.roll),
            pitch=self._noise_floor(self.axis_signs["pitch"] * state.pitch),
            yaw=  self._noise_floor(self.axis_signs["yaw"]   * state.yaw),
            t=state.t,
            buttons=list(state.buttons),
        )

    # ── Status line ──────────────────────────────────────────────────────

    def _print_status(self, msg: TwistStamped):
        lx = msg.twist.linear.x
        ly = msg.twist.linear.y
        lz = msg.twist.linear.z
        ax = msg.twist.angular.x
        ay = msg.twist.angular.y
        az = msg.twist.angular.z

        active_axes = {
            name for name, v in [('X', lx), ('Y', ly), ('Z', lz),
                                  ('Roll', ax), ('Pitch', ay), ('Yaw', az)]
            if abs(v) > 1e-6
        }
        moving = bool(active_axes)

        axes_changed    = active_axes != self._last_active_axes
        started_moving  = moving and not self._last_was_moving
        stopped_moving  = not moving and self._last_was_moving
        periodic_update = moving and (self._tick - self._last_print_tick) >= 50  # ~1 Hz

        if started_moving or stopped_moving or axes_changed or periodic_update:
            state_tag = 'PAUSED' if self._servo_paused else 'ACTIVE'
            if moving:
                trans = f'X:{lx:+.2f} Y:{ly:+.2f} Z:{lz:+.2f}'
                rot   = f'R:{ax:+.2f} P:{ay:+.2f} Yw:{az:+.2f}'
                axes  = ' '.join(sorted(active_axes))
                self.get_logger().info(f'[{state_tag}]  {trans}  |  {rot}  → [{axes}]')
            else:
                self.get_logger().info(f'[{state_tag}]  idle')
            self._last_print_tick = self._tick

        self._last_active_axes = active_axes
        self._last_was_moving  = moving

    # ── Publish ───────────────────────────────────────────────────────────

    def _publish_zero(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def _poll_and_publish(self):
        latest = self._latest_state()
        if latest is not None:
            self._handle_buttons(latest.buttons)
            self.last_state = latest

        if self._servo_paused:
            return

        now       = self.get_clock().now().nanoseconds / 1e9
        timed_out = (now - self.last_state.t) > self.command_timeout

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        if not timed_out:
            x, y, z, r, p, yw = self._filter_command(
                self.last_state.x,    self.last_state.y,     self.last_state.z,
                self.last_state.roll, self.last_state.pitch, self.last_state.yaw,
            )
            msg.twist.linear.x  = x  * self.linear_scale
            msg.twist.linear.y  = y  * self.linear_scale
            msg.twist.linear.z  = z  * self.linear_scale
            msg.twist.angular.x = r  * self.angular_scale
            msg.twist.angular.y = p  * self.angular_scale
            msg.twist.angular.z = yw * self.angular_scale

        is_nonzero = any(
            abs(v) > 1e-6
            for v in (
                msg.twist.linear.x,  msg.twist.linear.y,  msg.twist.linear.z,
                msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z,
            )
        )

        if is_nonzero or self.last_publish_nonzero:
            self.publisher.publish(msg)
        self.last_publish_nonzero = is_nonzero

        self._tick += 1
        self._print_status(msg)

    def destroy_node(self):
        try:
            spnav.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpaceMouseServoNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
