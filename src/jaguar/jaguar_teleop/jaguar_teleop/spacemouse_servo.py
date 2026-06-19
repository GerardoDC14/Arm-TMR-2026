from __future__ import annotations

import threading
from dataclasses import dataclass, field
from enum import Enum
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
    3: 'WARNING  Leaving singularity — recovering',
    4: 'HALT     Collision detected — halted',
    5: 'WARNING  Near joint limit',
}


class ServoMode(Enum):
    """Explicit state of the SpaceMouse servo controller."""
    SERVO_ACTIVE       = "SERVO_ACTIVE"        # Normal teleoperation
    PAUSED_USER        = "PAUSED_USER"         # User pressed button to pause
    PAUSED_SINGULARITY = "PAUSED_SINGULARITY"  # Auto-paused on servo HALT status


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

        # ── Core parameters ───────────────────────────────────────────────
        self.declare_parameter("robot_name",              "Robot")
        self.declare_parameter("twist_topic",             "/servo_node/delta_twist_cmds")
        self.declare_parameter("frame_id",                "base_link")
        self.declare_parameter("publish_rate_hz",         50.0)

        # Global velocity scales.
        # With servo_params command_in_type "unitless", set these to 1.0 here and
        # tune max velocity in servo_params scale.linear / scale.rotational.
        # With command_in_type "speed_units", set these to the desired max m/s and rad/s.
        self.declare_parameter("linear_scale",            1.0)
        self.declare_parameter("angular_scale",           1.0)

        # Per-axis fine-tuning multipliers (applied on top of the global scale above).
        # 1.0 = no change. < 1.0 attenuates that axis, > 1.0 amplifies it.
        self.declare_parameter("linear_x_scale",          1.0)
        self.declare_parameter("linear_y_scale",          1.0)
        self.declare_parameter("linear_z_scale",          1.0)
        self.declare_parameter("angular_roll_scale",      1.0)
        self.declare_parameter("angular_pitch_scale",     1.0)
        self.declare_parameter("angular_yaw_scale",       1.0)

        # ── Axis remapping ────────────────────────────────────────────────
        # Space Explorer HID convention:
        #   bytes 1-2 (driver label "x") = lateral / left-right
        #   bytes 3-4 (driver label "y") = fore-aft / forward-backward
        # Robot ROS convention: X = forward, Y = lateral.
        # → swap_xy_translation: True fixes this mismatch.
        # → swap_xy_rotation: True swaps roll↔pitch (usually not needed).
        self.declare_parameter("swap_xy_translation",     False)
        self.declare_parameter("swap_xy_rotation",        False)

        # Per-axis sign inversion.
        #
        # IMPORTANT: these flags are applied AFTER the swap, so they control
        # robot-frame axes, not raw HID device axes.  With swap_xy_translation:
        #   invert_x → flips forward/backward robot motion
        #   invert_y → flips left/right robot motion
        # This is intentional so the parameter names match what the operator sees.
        self.declare_parameter("invert_x",                False)
        self.declare_parameter("invert_y",                False)
        self.declare_parameter("invert_z",                False)
        self.declare_parameter("invert_roll",             False)
        self.declare_parameter("invert_pitch",            False)
        self.declare_parameter("invert_yaw",              False)

        # ── Noise / filtering ─────────────────────────────────────────────
        self.declare_parameter("deadband",                0.05)   # per-axis noise floor [0-1]
        self.declare_parameter("axis_relative_threshold", 0.40)   # suppress minor axes in group
        self.declare_parameter("dominance_threshold",     1.5)    # linear/angular group dominance
        self.declare_parameter("command_timeout",         0.20)   # zero cmd if HID silent > this

        # ── Expo curve ────────────────────────────────────────────────────
        # expo_factor = 0.0 : linear (full stick deflection = full speed)
        # expo_factor = 0.5 : blend of linear + cubic (good precision near zero)
        # expo_factor = 1.0 : pure cubic x^3 (very fine near zero, full speed at max)
        # Formula: f(x) = (1 - k)*x + k*x^3
        # At 10% stick with k=0.5: output = 0.5*0.1 + 0.5*0.001 = 0.051 (vs 0.1 linear)
        # At 50% stick with k=0.5: output = 0.5*0.5 + 0.5*0.125 = 0.3125 (vs 0.5 linear)
        # At 100% stick with k=any: output = 1.0 (unchanged)
        self.declare_parameter("expo_factor",             0.0)

        # ── Low-pass smoothing ────────────────────────────────────────────
        # EMA alpha: 1.0 = no smoothing (raw), 0.65 = light filter.
        # Increase toward 1.0 if the arm feels sluggish.
        self.declare_parameter("smoothing_alpha",         0.7)

        # ── Mode / singularity handling ───────────────────────────────────
        self.declare_parameter("start_paused", True)
        self.declare_parameter("auto_resume_after_singularity", True)
        self.declare_parameter("singularity_resume_delay_sec",  1.0)

        # ── Debug logging ─────────────────────────────────────────────────
        # When True, logs three lines per active command cycle showing:
        #   [DBG hid]    raw HID values (after driver scale, after deadband, before any remap)
        #   [DBG mapped] values after swap and sign inversion (= final robot-frame axes)
        #   [DBG twist]  final published TwistStamped field values
        # Enable this to diagnose axis direction issues.
        self.declare_parameter("debug_input",             False)

        # ── Services / topics ─────────────────────────────────────────────
        self.declare_parameter("pause_service",  "/servo_node/pause_servo")
        self.declare_parameter("start_service",  "/servo_node/start_servo")
        self.declare_parameter("status_topic",   "/servo_node/status")

        # ── Read parameters ───────────────────────────────────────────────
        self.robot_name      = str(self.get_parameter("robot_name").value)
        self.twist_topic     = self.get_parameter("twist_topic").value
        self.frame_id        = self.get_parameter("frame_id").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.linear_scale    = float(self.get_parameter("linear_scale").value)
        self.angular_scale   = float(self.get_parameter("angular_scale").value)
        self.per_axis_lin = {
            "x": float(self.get_parameter("linear_x_scale").value),
            "y": float(self.get_parameter("linear_y_scale").value),
            "z": float(self.get_parameter("linear_z_scale").value),
        }
        self.per_axis_ang = {
            "roll":  float(self.get_parameter("angular_roll_scale").value),
            "pitch": float(self.get_parameter("angular_pitch_scale").value),
            "yaw":   float(self.get_parameter("angular_yaw_scale").value),
        }
        self.swap_xy_translation = bool(self.get_parameter("swap_xy_translation").value)
        self.swap_xy_rotation    = bool(self.get_parameter("swap_xy_rotation").value)
        # Signs applied POST-SWAP (robot-frame axes), see parameter docs above.
        self.axis_signs = {
            "x":     -1.0 if bool(self.get_parameter("invert_x").value)     else 1.0,
            "y":     -1.0 if bool(self.get_parameter("invert_y").value)     else 1.0,
            "z":     -1.0 if bool(self.get_parameter("invert_z").value)     else 1.0,
            "roll":  -1.0 if bool(self.get_parameter("invert_roll").value)  else 1.0,
            "pitch": -1.0 if bool(self.get_parameter("invert_pitch").value) else 1.0,
            "yaw":   -1.0 if bool(self.get_parameter("invert_yaw").value)   else 1.0,
        }
        self.deadband                = float(self.get_parameter("deadband").value)
        self.axis_relative_threshold = float(self.get_parameter("axis_relative_threshold").value)
        self.dominance_threshold     = float(self.get_parameter("dominance_threshold").value)
        self.command_timeout         = float(self.get_parameter("command_timeout").value)
        self.expo_factor             = max(0.0, min(1.0,
                                         float(self.get_parameter("expo_factor").value)))
        self.smoothing_alpha         = max(0.0, min(1.0,
                                         float(self.get_parameter("smoothing_alpha").value)))
        self.start_paused = bool(self.get_parameter("start_paused").value)
        self.auto_resume_after_singularity = bool(
            self.get_parameter("auto_resume_after_singularity").value)
        self.singularity_resume_delay_sec  = float(
            self.get_parameter("singularity_resume_delay_sec").value)
        self.debug_input = bool(self.get_parameter("debug_input").value)

        pause_svc    = self.get_parameter("pause_service").value
        start_svc    = self.get_parameter("start_service").value
        status_topic = self.get_parameter("status_topic").value

        # ── ROS interfaces ────────────────────────────────────────────────
        self.publisher  = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self._pause_cli = self.create_client(Trigger, pause_svc)
        self._start_cli = self.create_client(Trigger, start_svc)
        self.create_subscription(Int8, status_topic, self._status_cb, 10)

        # ── State ─────────────────────────────────────────────────────────
        self.last_state: MotionState          = MotionState()
        self.last_publish_nonzero             = False
        self._mode                            = (
            ServoMode.PAUSED_USER if self.start_paused else ServoMode.SERVO_ACTIVE
        )
        self._last_status                     = 0
        self._prev_buttons: List[int]         = []
        self._tick                            = 0
        self._last_active_axes: set           = set()
        self._last_was_moving                 = False
        self._last_print_tick                 = 0
        # Low-pass smoother state: [x, y, z, roll, pitch, yaw]
        self._smooth: List[float]             = [0.0] * 6
        # Timestamp when servo last entered PAUSED_SINGULARITY
        self._singularity_halt_time: Optional[float] = None

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
        x_src = "fore/aft (Y→X swapped)"  if self.swap_xy_translation else "fore/aft"
        y_src = "lateral (X→Y swapped)"   if self.swap_xy_translation else "lateral"
        inv_x = " [INVERTED]" if self.axis_signs["x"] < 0 else ""
        inv_y = " [INVERTED]" if self.axis_signs["y"] < 0 else ""
        inv_z = " [INVERTED]" if self.axis_signs["z"] < 0 else ""
        print('\n' + '=' * 62)
        print(f'  {self.robot_name} — Space Explorer Servo  (6-DOF)')
        print('=' * 62)
        print('  Translation  (puck push)')
        print(f'    Push fwd  / back   → EEF  +X / -X  [{x_src}]{inv_x}')
        print(f'    Push left / right  → EEF  +Y / -Y  [{y_src}]{inv_y}')
        print(f'    Push up   / down   → EEF  +Z / -Z{inv_z}')
        print()
        print('  Rotation  (puck tilt / twist)')
        print('    Tilt left / right  → Roll  (angular X)')
        print('    Tilt fwd  / back   → Pitch (angular Y)')
        print('    Twist CW  / CCW    → Yaw   (angular Z)')
        print()
        print('  RIGHT button   Pause / Resume MoveIt Servo')
        print('─' * 62)
        print(f'  Mode:            {self._mode.value}')
        print(f'  linear_scale:    {self.linear_scale:.2f}   angular_scale: {self.angular_scale:.2f}')
        print(f'  smoothing_alpha: {self.smoothing_alpha:.2f}   deadband:      {self.deadband:.2f}')
        print(f'  expo_factor:     {self.expo_factor:.2f}')
        print(f'  swap_xy_trans:   {self.swap_xy_translation}   swap_xy_rot: {self.swap_xy_rotation}')
        print(f'  start_paused:    {self.start_paused}')
        print(f'  debug_input:     {self.debug_input}')
        if self.debug_input:
            print('  [DEBUG MODE] Axis diagnostics will be logged while moving.')
        if self._mode == ServoMode.PAUSED_USER:
            print()
            print('  ** Starting in RViz mode (PAUSED_USER).')
            print('  ** Use RViz to move the arm away from any singular pose,')
            print('  ** then press RIGHT button to enable SpaceMouse control.')
        print('=' * 62 + '\n')

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
            self.get_logger().info('[SERVO] OK — singularity / collision cleared')

        # Auto-pause on hard halt so stale smooth state does not fight the servo halt.
        if code in (2, 4) and self._mode == ServoMode.SERVO_ACTIVE:
            self._zero_smooth_state()
            halt_name = "singularity" if code == 2 else "collision"
            self._mode = ServoMode.PAUSED_SINGULARITY
            self._singularity_halt_time = self.get_clock().now().nanoseconds / 1e9
            resume_note = (
                f"auto-resume after {self.singularity_resume_delay_sec:.1f}s once clear"
                if self.auto_resume_after_singularity
                else "press RIGHT button to resume"
            )
            self.get_logger().warn(
                f'[MODE → PAUSED_SINGULARITY]  Auto-paused on {halt_name} halt — '
                f'inputs zeroed; {resume_note}.  '
                f'Use RViz to move the arm to a non-singular pose first.'
            )

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
        if self._mode == ServoMode.PAUSED_USER:
            self._mode = ServoMode.SERVO_ACTIVE
            self._zero_smooth_state()
            self.last_publish_nonzero = True  # force publish so servo_node does not time out
            self._call_service(self._start_cli, 'start_servo')
            self.get_logger().info('[MODE → SERVO_ACTIVE]  SpaceMouse control resumed')

        elif self._mode == ServoMode.SERVO_ACTIVE:
            self._zero_smooth_state()
            self._publish_zero()
            self._mode = ServoMode.PAUSED_USER
            self._call_service(self._pause_cli, 'pause_servo')
            self.get_logger().info(
                '[MODE → PAUSED_USER]  Servo paused — RViz / move_group active; '
                'press RIGHT button to resume'
            )

        elif self._mode == ServoMode.PAUSED_SINGULARITY:
            # User pressed button during a singularity pause → convert to user pause
            # (prevents auto-resume from overriding user's explicit choice)
            self._mode = ServoMode.PAUSED_USER
            self.get_logger().info(
                '[MODE → PAUSED_USER]  Manual pause override (was PAUSED_SINGULARITY); '
                'press RIGHT button to resume'
            )

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

    def _filter_command(self, x, y, z, r, p, yw):
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

    def _expo(self, value: float) -> float:
        """Expo curve for precision near zero.
        f(x) = (1 - k)*x + k*x^3
        k=0.0: linear (no change). k=1.0: pure cubic.
        At 10% stick with k=0.5: output ≈ 5.1% (vs 10% linear).
        At 50% stick with k=0.5: output ≈ 31.3% (vs 50% linear).
        At 100% stick: always 100% regardless of k.
        x^3 preserves sign so negative values are handled correctly."""
        if self.expo_factor < 1e-6:
            return value
        k = self.expo_factor
        return (1.0 - k) * value + k * value * value * value

    def _zero_smooth_state(self):
        """Reset EMA smoother — call on every mode transition so there is no
        command carryover when resuming from a paused state."""
        self._smooth = [0.0] * 6

    def _latest_state(self) -> Optional[MotionState]:
        """Read the SpaceMouse HID device and apply only the deadband.

        Axis sign inversion and swap are NOT applied here.  They are applied
        post-swap in _poll_and_publish() so that the invert_* parameters
        control final robot-frame axes rather than raw HID device axes.
        """
        state = spnav.read()
        if state is None:
            return None

        return MotionState(
            x=    self._noise_floor(state.x),
            y=    self._noise_floor(state.y),
            z=    self._noise_floor(state.z),
            roll= self._noise_floor(state.roll),
            pitch=self._noise_floor(state.pitch),
            yaw=  self._noise_floor(state.yaw),
            t=state.t,
            buttons=list(state.buttons),
        )

    # ── Status line ───────────────────────────────────────────────────────

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
        periodic_update = moving and (self._tick - self._last_print_tick) >= 50  # ~1 Hz at 50Hz

        if started_moving or stopped_moving or axes_changed or periodic_update:
            mode_tag = self._mode.value
            if moving:
                trans = f'X:{lx:+.2f} Y:{ly:+.2f} Z:{lz:+.2f}'
                rot   = f'R:{ax:+.2f} P:{ay:+.2f} Yw:{az:+.2f}'
                axes  = ' '.join(sorted(active_axes))
                self.get_logger().info(f'[{mode_tag}]  {trans}  |  {rot}  → [{axes}]')
            else:
                self.get_logger().info(f'[{mode_tag}]  idle')
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

        now_sec = self.get_clock().now().nanoseconds / 1e9

        # ── Auto-resume from singularity pause ────────────────────────────
        if (
            self._mode == ServoMode.PAUSED_SINGULARITY
            and self.auto_resume_after_singularity
            and self._singularity_halt_time is not None
            and self._last_status in (0, 3)
            and (now_sec - self._singularity_halt_time) >= self.singularity_resume_delay_sec
        ):
            self._mode = ServoMode.SERVO_ACTIVE
            self.last_publish_nonzero = True  # force publish so servo_node does not time out
            self._call_service(self._start_cli, 'start_servo')
            self.get_logger().info(
                f'[MODE → SERVO_ACTIVE]  Auto-resumed after singularity '
                f'({self.singularity_resume_delay_sec:.1f}s delay)'
            )
            self._singularity_halt_time = None

        # ── Paused: keep smooth state zeroed, publish nothing ─────────────
        if self._mode != ServoMode.SERVO_ACTIVE:
            self._zero_smooth_state()
            return

        # ── Build raw command from latest HID state ───────────────────────
        timed_out = (now_sec - self.last_state.t) > self.command_timeout

        if timed_out:
            raw_x = raw_y = raw_z = raw_r = raw_p = raw_yw = 0.0
        else:
            raw_x  = self.last_state.x
            raw_y  = self.last_state.y
            raw_z  = self.last_state.z
            raw_r  = self.last_state.roll
            raw_p  = self.last_state.pitch
            raw_yw = self.last_state.yaw

        rx, ry, rz   = raw_x, raw_y, raw_z
        rr, rp, ryw  = raw_r, raw_p, raw_yw

        # ── Axis swap ─────────────────────────────────────────────────────
        # Swap is applied BEFORE sign so that the invert_* parameters operate
        # on the robot-frame axes (post-swap), not the raw HID device axes.
        # With swap_xy_translation=True and the Space Explorer:
        #   robot +X (forward) ← HID y (fore/aft)
        #   robot +Y (lateral) ← HID x (lateral)
        if self.swap_xy_translation:
            rx, ry = ry, rx
        if self.swap_xy_rotation:
            rr, rp = rp, rr

        # ── Per-axis sign inversion (robot-frame, post-swap) ──────────────
        # invert_x controls forward/backward  (twist.linear.x)
        # invert_y controls left/right         (twist.linear.y)
        # invert_z controls up/down            (twist.linear.z)
        rx  *= self.axis_signs["x"]
        ry  *= self.axis_signs["y"]
        rz  *= self.axis_signs["z"]
        rr  *= self.axis_signs["roll"]
        rp  *= self.axis_signs["pitch"]
        ryw *= self.axis_signs["yaw"]

        # ── Debug log: stage 1 and 2 ──────────────────────────────────────
        # Enable with debug_input: True in launch parameters.
        # Logs raw HID values and post-swap+sign (robot-frame) values.
        if self.debug_input and (
            any(abs(v) > 1e-6 for v in (raw_x, raw_y, raw_z, raw_r, raw_p, raw_yw))
            or self._last_was_moving
        ):
            self.get_logger().info(
                f'[DBG hid]    x={raw_x:+.3f} y={raw_y:+.3f} z={raw_z:+.3f}'
                f' | r={raw_r:+.3f} p={raw_p:+.3f} yw={raw_yw:+.3f}',
                throttle_duration_sec=0.1)
            self.get_logger().info(
                f'[DBG mapped] x={rx:+.3f} y={ry:+.3f} z={rz:+.3f}'
                f' | r={rr:+.3f} p={rp:+.3f} yw={ryw:+.3f}'
                f'  (swap={self.swap_xy_translation}'
                f' inv_x={self.axis_signs["x"]<0}'
                f' inv_y={self.axis_signs["y"]<0})',
                throttle_duration_sec=0.1)

        # ── Expo curve (precision near zero) ─────────────────────────────
        rx  = self._expo(rx)
        ry  = self._expo(ry)
        rz  = self._expo(rz)
        rr  = self._expo(rr)
        rp  = self._expo(rp)
        ryw = self._expo(ryw)

        # ── Group dominance + minor-axis suppression ──────────────────────
        x, y, z, r, p, yw = self._filter_command(rx, ry, rz, rr, rp, ryw)

        # ── Exponential moving average (low-pass) ─────────────────────────
        a = self.smoothing_alpha
        self._smooth[0] = a * x  + (1.0 - a) * self._smooth[0]
        self._smooth[1] = a * y  + (1.0 - a) * self._smooth[1]
        self._smooth[2] = a * z  + (1.0 - a) * self._smooth[2]
        self._smooth[3] = a * r  + (1.0 - a) * self._smooth[3]
        self._smooth[4] = a * p  + (1.0 - a) * self._smooth[4]
        self._smooth[5] = a * yw + (1.0 - a) * self._smooth[5]

        sx, sy, sz, sr, sp, syw = self._smooth

        # ── Build and publish TwistStamped ────────────────────────────────
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x  = sx  * self.linear_scale  * self.per_axis_lin["x"]
        msg.twist.linear.y  = sy  * self.linear_scale  * self.per_axis_lin["y"]
        msg.twist.linear.z  = sz  * self.linear_scale  * self.per_axis_lin["z"]
        msg.twist.angular.x = sr  * self.angular_scale * self.per_axis_ang["roll"]
        msg.twist.angular.y = sp  * self.angular_scale * self.per_axis_ang["pitch"]
        msg.twist.angular.z = syw * self.angular_scale * self.per_axis_ang["yaw"]

        is_nonzero = any(
            abs(v) > 1e-6
            for v in (
                msg.twist.linear.x,  msg.twist.linear.y,  msg.twist.linear.z,
                msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z,
            )
        )

        # ── Debug log: stage 3 (final published twist) ────────────────────
        if self.debug_input and is_nonzero:
            self.get_logger().info(
                f'[DBG twist]  lx={msg.twist.linear.x:+.4f}'
                f' ly={msg.twist.linear.y:+.4f}'
                f' lz={msg.twist.linear.z:+.4f}'
                f' | rx={msg.twist.angular.x:+.4f}'
                f' ry={msg.twist.angular.y:+.4f}'
                f' rz={msg.twist.angular.z:+.4f}',
                throttle_duration_sec=0.1)

        # Only publish if there is something to say, or to send a trailing zero
        # so the servo node sees the command go to zero (not just silence).
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
