from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
import sys
import threading
import time
from typing import Optional


if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


try:
    import rclpy
    from rclpy.action import ActionServer
    from rclpy.action import CancelResponse
    from rclpy.action import GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTrajectoryControllerState
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
except ImportError as exc:  # pragma: no cover
    rclpy = None
    ActionServer = None  # type: ignore[assignment]
    CancelResponse = None  # type: ignore[assignment]
    GoalResponse = None  # type: ignore[assignment]
    ReentrantCallbackGroup = None  # type: ignore[assignment]
    MultiThreadedExecutor = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment]
    QoSProfile = DurabilityPolicy = HistoryPolicy = ReliabilityPolicy = None  # type: ignore[assignment]
    FollowJointTrajectory = None  # type: ignore[assignment]
    JointTrajectoryControllerState = None  # type: ignore[assignment]
    JointState = None  # type: ignore[assignment]
    Trigger = None  # type: ignore[assignment]
    JointTrajectory = None  # type: ignore[assignment]
    JointTrajectoryPoint = None  # type: ignore[assignment]
    _ROS_IMPORT_ERROR = exc
else:
    _ROS_IMPORT_ERROR = None


try:
    import serial
    from serial import Serial, SerialException
except ImportError as exc:  # pragma: no cover
    serial = None  # type: ignore[assignment]
    Serial = object  # type: ignore[assignment]
    SerialException = Exception  # type: ignore[assignment]
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None


class BridgeState(Enum):
    """Explicit lifecycle state of the bridge node.

    Transitions (default require_explicit_rearm=True):
      STARTUP         → REQUIRES_REARM   (startup_delay elapsed — NEVER auto-ACTIVE on boot)
      REQUIRES_REARM  → ACTIVE           (/rearm service called AND fresh joint_state present)
      ACTIVE          → FAULT            (FAILSAFE, serial fault threshold, or bus-off)
      FAULT           → RECOVERY         ("Failsafe cleared" received from ESP32)
      RECOVERY        → REQUIRES_REARM   (fault_recovery_delay elapsed)
      (any)           → FAULT            (/disarm service)

    Why REQUIRES_REARM is between every potentially-stale transition:
      Any event that could invalidate the bridge↔hardware command epoch (boot,
      fault, arm-only power cycle) MUST force a human handshake before motion
      resumes.  Prevents the failure mode where motors re-energise and the
      bridge replays the last MoveIt pose from before the outage.
    """
    STARTUP         = "STARTUP"          # boot hold-off; no commands sent
    REQUIRES_REARM  = "REQUIRES_REARM"   # safe-idle; waiting for explicit /rearm handshake
    ACTIVE          = "ACTIVE"           # normal operation
    FAULT           = "FAULT"            # hardware fault; commands suppressed, jx6 sent
    RECOVERY        = "RECOVERY"         # fault cleared; hold-off before REQUIRES_REARM


@dataclass(frozen=True)
class JointMapping:
    moveit_name: str
    min_deg: float
    max_deg: float
    sign: float

    def moveit_rad_to_physical_deg(self, radians_value: float) -> float:
        return self.sign * math.degrees(radians_value)


JOINT_MAPPINGS: tuple[JointMapping, ...] = (
    JointMapping("Joint1", -90.0,  90.0, -1.0),
    JointMapping("Joint2",   0.0, 180.0, -1.0),
    JointMapping("Joint3", -200.0,  0.0, -1.0),
    JointMapping("Joint4",  -90.0, 90.0, -1.0),
    JointMapping("Joint5",  -90.0, 90.0, -1.0),
    JointMapping("Joint6",  -90.0, 90.0,  1.0),
)

FIXED_JOINT_STATE_RAD: dict[str, float] = {
    "Flipper1J": 0.0,
    "Flipper2J": 0.0,
    "Flipper3J": 0.0,
    "Flipper4J": 0.0,
}


@dataclass(frozen=True)
class TimedJointTarget:
    time_from_start_sec: float
    deg_by_joint: dict[str, float]


@dataclass(frozen=True)
class CurrentStateSnapshot:
    deg_by_joint: dict[str, float]
    source: str


@dataclass(frozen=True)
class TrajectoryValidation:
    ok: bool
    message: str
    points: tuple[TimedJointTarget, ...] = ()
    current_state: Optional[CurrentStateSnapshot] = None


class MoveItArmBridge6DofNode(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError(f"ROS 2 imports unavailable: {_ROS_IMPORT_ERROR}")
        if serial is None:
            raise RuntimeError(f"pyserial unavailable: {_SERIAL_IMPORT_ERROR}")

        super().__init__("moveit_arm_bridge_6dof")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("serial_port",  "/dev/ttyUSB0")
        # 921600 matches the ESP32 firmware constant SERIAL_BAUD_RATE.
        # At 921600 each "j6 ..." command write takes ~0.4 ms vs ~3.3 ms at 115200.
        # If your USB-serial cable is marginal, fall back to 460800.
        self.declare_parameter("baud_rate",    921600)

        # Increased default from 20 → 50 Hz: halves worst-case command latency
        # (50 ms → 20 ms) without stressing the serial link.
        self.declare_parameter("command_rate_hz", 50.0)

        self.declare_parameter("min_delta_deg",             0.2)
        self.declare_parameter("joint_state_timeout_sec",   3.0)
        self.declare_parameter("idle_on_stale",             False)
        self.declare_parameter("max_command_velocity_dps",  90.0)
        self.declare_parameter("max_command_acceleration_dps2", 220.0)
        self.declare_parameter(
            "joint_max_velocity_dps",
            [10.0, 8.0, 8.0, 20.0, 20.0, 20.0],
        )
        self.declare_parameter(
            "joint_max_acceleration_dps2",
            [30.0, 24.0, 24.0, 60.0, 60.0, 60.0],
        )
        self.declare_parameter("joint4_offset_deg",         0.0)

        # Tighter write timeout (0.1s) prevents a single slow write from
        # blocking the timer callback for a full 250 ms.
        self.declare_parameter("write_timeout_sec",  0.1)

        # Longer startup (5 s default) to give the CAN bus and all motor
        # nodes time to come up on a shared power bus.
        self.declare_parameter("startup_delay_sec",  5.0)

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "trajectory_topics",
            ["/jaguar_arm_controller/joint_trajectory", "/dicerox_arm_controller/joint_trajectory"],
        )
        self.declare_parameter("trajectory_hold_sec", 0.5)
        self.declare_parameter("prefer_trajectory_topic", True)
        self.declare_parameter(
            "controller_state_topics",
            ["/jaguar_arm_controller/controller_state", "/dicerox_arm_controller/controller_state"],
        )
        self.declare_parameter("controller_state_timeout_sec", 0.5)
        self.declare_parameter("prefer_controller_state", True)

        # Hold-off after a FAILSAFE clears before re-arming (allow bus to settle).
        self.declare_parameter("fault_recovery_delay_sec", 2.0)

        # Number of consecutive serial-write failures before entering FAULT.
        self.declare_parameter("serial_fault_threshold", 3)

        # Number of "partially applied" ESP32 replies before entering FAULT.
        self.declare_parameter("partial_apply_threshold", 3)

        # Minimum number of serial lines that must arrive from the ESP32 before
        # the STARTUP→ACTIVE transition is allowed.  Useful when the bridge
        # starts BEFORE the main voltage bus (USB powers ESP32 only):  the ESP32
        # will be silent until CAN motors come up, so the bridge waits for activity.
        #
        # Default is 0 (time-only startup) because if the bridge starts AFTER the
        # ESP32 has already booted, the ESP32 may be idle and never send lines
        # during the startup window.  Enable (e.g. 5) only when you explicitly
        # need to guard against the early-USB-connect scenario.
        self.declare_parameter("startup_min_serial_lines", 0)

        # ── Re-arm policy (stale-command-replay hardening) ────────────────
        # If True: every transition that could have left the bridge's command
        # epoch out of sync with the hardware (boot, fault, recovery) routes
        # through REQUIRES_REARM and waits for an explicit /rearm service call
        # before motion resumes.  This is the dummy-proof default.
        #
        # Set False only for bench work where the operator accepts responsibility
        # for any stale-pose replay after arm-only power cycles.
        self.declare_parameter("require_explicit_rearm", True)

        # Send "jx6\n" to the ESP32 on FAULT entry to clear its in-firmware
        # target cache (g_streamLastTarget / g_auxTargets).  Prevents the ESP32
        # from continuing to stream a 20 Hz last-target after motors re-energise.
        self.declare_parameter("send_jx6_on_fault", True)

        # If > 0, auto-rearm after the arm has been physically settled (no
        # joint_state motion greater than this threshold) for the configured
        # dwell time.  Opt-in; default 0.0 disables auto-rearm entirely so the
        # operator must always click /rearm.
        self.declare_parameter("auto_rearm_settle_threshold_deg", 0.0)
        self.declare_parameter("auto_rearm_settle_dwell_sec",     3.0)

        # ── FollowJointTrajectory action safety defaults ──────────────────
        self.declare_parameter(
            "follow_joint_trajectory_action_name",
            "/dicerox_arm_controller/follow_joint_trajectory",
        )
        self.declare_parameter("dry_run", True)
        self.declare_parameter("enable_hardware_motion", False)
        self.declare_parameter("require_trusted_joint_state_for_hardware", True)
        self.declare_parameter("trusted_joint_state_publishers", [])
        self.declare_parameter("allow_fake_joint_states_for_hardware", False)
        self.declare_parameter("joint_state_source_mode", "dry_run_fake")
        self.declare_parameter("allow_dry_run_without_active_hardware", True)
        self.declare_parameter("require_current_joint_state", True)
        self.declare_parameter("allow_dry_run_initial_joint_state", True)
        self.declare_parameter(
            "dry_run_initial_joint_positions_deg",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            "trusted_open_loop_initial_joint_positions_deg",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter("goal_start_tolerance_deg", 5.0)
        self.declare_parameter("first_point_max_delta_deg", 15.0)
        self.declare_parameter("goal_tolerance_deg", 2.0)
        self.declare_parameter("trajectory_tracking_error_abort_deg", 15.0)
        self.declare_parameter("trajectory_tracking_error_abort_sec", 1.0)
        self.declare_parameter("zero_duration_epsilon_sec", 0.001)
        self.declare_parameter("position_epsilon_deg", 0.05)
        self.declare_parameter("allow_single_point_goal", False)
        self.declare_parameter("hardware_snapshot_timeout_sec", 1.0)
        self.declare_parameter("require_pre_motion_hardware_snapshot", True)
        self.declare_parameter("require_post_motion_hardware_snapshot", True)
        self.declare_parameter("publish_hardware_snapshots_for_planning", False)
        self.declare_parameter("periodic_hardware_snapshot_refresh", False)
        self.declare_parameter("republish_cached_hardware_joint_states", True)
        self.declare_parameter("publish_trusted_open_loop_commanded_joint_states", True)
        self.declare_parameter("idle_on_action_cancel", False)
        self.declare_parameter("trusted_open_loop_hold_after_action", True)
        self.declare_parameter("trusted_open_loop_hold_rate_hz", 10.0)
        self.declare_parameter("hardware_snapshot_publish_period_sec", 0.1)
        self.declare_parameter("request_startup_hardware_snapshot", False)
        self.declare_parameter("joint_limit_epsilon_deg", 0.05)

        # ── Read parameters ───────────────────────────────────────────────
        self.serial_port              = str(self.get_parameter("serial_port").value)
        self.baud_rate                = int(self.get_parameter("baud_rate").value)
        self.command_rate_hz          = float(self.get_parameter("command_rate_hz").value)
        self.min_delta_deg            = float(self.get_parameter("min_delta_deg").value)
        self.joint_state_timeout_sec  = float(self.get_parameter("joint_state_timeout_sec").value)
        self.idle_on_stale            = bool(self.get_parameter("idle_on_stale").value)
        self.max_command_velocity_dps = float(self.get_parameter("max_command_velocity_dps").value)
        self.max_command_acceleration_dps2 = float(
            self.get_parameter("max_command_acceleration_dps2").value)
        joint_max_velocity_value = self.get_parameter("joint_max_velocity_dps").value
        joint_max_accel_value = self.get_parameter("joint_max_acceleration_dps2").value
        self.joint_max_velocity_dps = [
            float(value) for value in list(joint_max_velocity_value)
        ]
        self.joint_max_acceleration_dps2 = [
            float(value) for value in list(joint_max_accel_value)
        ]
        self.joint4_offset_deg        = float(self.get_parameter("joint4_offset_deg").value)
        self.write_timeout_sec        = float(self.get_parameter("write_timeout_sec").value)
        self.startup_delay_sec        = float(self.get_parameter("startup_delay_sec").value)
        self.joint_state_topic        = str(self.get_parameter("joint_state_topic").value)
        trajectory_topics_value = self.get_parameter("trajectory_topics").value
        if isinstance(trajectory_topics_value, str):
            trajectory_topics_raw = [trajectory_topics_value]
        else:
            trajectory_topics_raw = list(trajectory_topics_value)
        self.trajectory_topics = [
            str(topic)
            for topic in trajectory_topics_raw
            if str(topic)
        ]
        self.trajectory_hold_sec = float(self.get_parameter("trajectory_hold_sec").value)
        self.prefer_trajectory_topic = bool(
            self.get_parameter("prefer_trajectory_topic").value)
        controller_state_topics_value = self.get_parameter("controller_state_topics").value
        if isinstance(controller_state_topics_value, str):
            controller_state_topics_raw = [controller_state_topics_value]
        else:
            controller_state_topics_raw = list(controller_state_topics_value)
        self.controller_state_topics  = [
            str(topic)
            for topic in controller_state_topics_raw
            if str(topic)
        ]
        self.controller_state_timeout_sec = float(
            self.get_parameter("controller_state_timeout_sec").value)
        self.prefer_controller_state = bool(
            self.get_parameter("prefer_controller_state").value)
        self.fault_recovery_delay_sec = float(self.get_parameter("fault_recovery_delay_sec").value)
        self.serial_fault_threshold   = int(self.get_parameter("serial_fault_threshold").value)
        self.partial_apply_threshold   = int(self.get_parameter("partial_apply_threshold").value)
        self.startup_min_serial_lines  = int(self.get_parameter("startup_min_serial_lines").value)
        self.require_explicit_rearm    = bool(
            self.get_parameter("require_explicit_rearm").value)
        self.send_jx6_on_fault         = bool(
            self.get_parameter("send_jx6_on_fault").value)
        self.auto_rearm_settle_threshold_deg = float(
            self.get_parameter("auto_rearm_settle_threshold_deg").value)
        self.auto_rearm_settle_dwell_sec = float(
            self.get_parameter("auto_rearm_settle_dwell_sec").value)
        self.follow_joint_trajectory_action_name = str(
            self.get_parameter("follow_joint_trajectory_action_name").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.enable_hardware_motion = bool(
            self.get_parameter("enable_hardware_motion").value)
        self.require_trusted_joint_state_for_hardware = bool(
            self.get_parameter("require_trusted_joint_state_for_hardware").value)
        trusted_publishers_value = self.get_parameter("trusted_joint_state_publishers").value
        if isinstance(trusted_publishers_value, str):
            trusted_publishers_raw = [trusted_publishers_value]
        else:
            trusted_publishers_raw = list(trusted_publishers_value)
        self.trusted_joint_state_publishers = [
            str(publisher)
            for publisher in trusted_publishers_raw
            if str(publisher)
        ]
        self.allow_fake_joint_states_for_hardware = bool(
            self.get_parameter("allow_fake_joint_states_for_hardware").value)
        self.joint_state_source_mode = str(
            self.get_parameter("joint_state_source_mode").value)
        self.allow_dry_run_without_active_hardware = bool(
            self.get_parameter("allow_dry_run_without_active_hardware").value)
        self.require_current_joint_state = bool(
            self.get_parameter("require_current_joint_state").value)
        self.allow_dry_run_initial_joint_state = bool(
            self.get_parameter("allow_dry_run_initial_joint_state").value)
        dry_run_initial_positions = self.get_parameter(
            "dry_run_initial_joint_positions_deg").value
        self.dry_run_initial_joint_positions_deg = [
            float(value) for value in list(dry_run_initial_positions)
        ]
        trusted_open_loop_initial_positions = self.get_parameter(
            "trusted_open_loop_initial_joint_positions_deg").value
        self.trusted_open_loop_initial_joint_positions_deg = [
            float(value) for value in list(trusted_open_loop_initial_positions)
        ]
        self.goal_start_tolerance_deg = float(
            self.get_parameter("goal_start_tolerance_deg").value)
        self.first_point_max_delta_deg = float(
            self.get_parameter("first_point_max_delta_deg").value)
        self.goal_tolerance_deg = float(
            self.get_parameter("goal_tolerance_deg").value)
        self.trajectory_tracking_error_abort_deg = float(
            self.get_parameter("trajectory_tracking_error_abort_deg").value)
        self.trajectory_tracking_error_abort_sec = float(
            self.get_parameter("trajectory_tracking_error_abort_sec").value)
        self.zero_duration_epsilon_sec = float(
            self.get_parameter("zero_duration_epsilon_sec").value)
        self.position_epsilon_deg = float(
            self.get_parameter("position_epsilon_deg").value)
        self.allow_single_point_goal = bool(
            self.get_parameter("allow_single_point_goal").value)
        self.hardware_snapshot_timeout_sec = float(
            self.get_parameter("hardware_snapshot_timeout_sec").value)
        self.require_pre_motion_hardware_snapshot = bool(
            self.get_parameter("require_pre_motion_hardware_snapshot").value)
        self.require_post_motion_hardware_snapshot = bool(
            self.get_parameter("require_post_motion_hardware_snapshot").value)
        self.publish_hardware_snapshots_for_planning = bool(
            self.get_parameter("publish_hardware_snapshots_for_planning").value)
        self.periodic_hardware_snapshot_refresh = bool(
            self.get_parameter("periodic_hardware_snapshot_refresh").value)
        self.republish_cached_hardware_joint_states = bool(
            self.get_parameter("republish_cached_hardware_joint_states").value)
        self.publish_trusted_open_loop_commanded_joint_states = bool(
            self.get_parameter("publish_trusted_open_loop_commanded_joint_states").value)
        self.idle_on_action_cancel = bool(
            self.get_parameter("idle_on_action_cancel").value)
        self.trusted_open_loop_hold_after_action = bool(
            self.get_parameter("trusted_open_loop_hold_after_action").value)
        self.trusted_open_loop_hold_rate_hz = float(
            self.get_parameter("trusted_open_loop_hold_rate_hz").value)
        self.hardware_snapshot_publish_period_sec = float(
            self.get_parameter("hardware_snapshot_publish_period_sec").value)
        self.request_startup_hardware_snapshot = bool(
            self.get_parameter("request_startup_hardware_snapshot").value)
        self.joint_limit_epsilon_deg = float(
            self.get_parameter("joint_limit_epsilon_deg").value)

        if self.command_rate_hz <= 0.0:
            raise ValueError("command_rate_hz must be > 0")
        if self.max_command_velocity_dps <= 0.0:
            raise ValueError("max_command_velocity_dps must be > 0")
        if self.max_command_acceleration_dps2 <= 0.0:
            raise ValueError("max_command_acceleration_dps2 must be > 0")
        if len(self.joint_max_velocity_dps) != len(JOINT_MAPPINGS):
            raise ValueError(
                f"joint_max_velocity_dps must contain exactly {len(JOINT_MAPPINGS)} values")
        if len(self.joint_max_acceleration_dps2) != len(JOINT_MAPPINGS):
            raise ValueError(
                f"joint_max_acceleration_dps2 must contain exactly {len(JOINT_MAPPINGS)} values")
        if any(value <= 0.0 for value in self.joint_max_velocity_dps):
            raise ValueError("joint_max_velocity_dps values must be > 0")
        if any(value <= 0.0 for value in self.joint_max_acceleration_dps2):
            raise ValueError("joint_max_acceleration_dps2 values must be > 0")
        if len(self.dry_run_initial_joint_positions_deg) != len(JOINT_MAPPINGS):
            raise ValueError(
                "dry_run_initial_joint_positions_deg must contain exactly "
                f"{len(JOINT_MAPPINGS)} values")
        if len(self.trusted_open_loop_initial_joint_positions_deg) != len(JOINT_MAPPINGS):
            raise ValueError(
                "trusted_open_loop_initial_joint_positions_deg must contain exactly "
                f"{len(JOINT_MAPPINGS)} values")
        if self.trajectory_tracking_error_abort_sec < 0.0:
            raise ValueError("trajectory_tracking_error_abort_sec must be >= 0")
        if self.zero_duration_epsilon_sec < 0.0:
            raise ValueError("zero_duration_epsilon_sec must be >= 0")
        if self.hardware_snapshot_timeout_sec <= 0.0:
            raise ValueError("hardware_snapshot_timeout_sec must be > 0")
        if self.hardware_snapshot_publish_period_sec <= 0.0:
            raise ValueError("hardware_snapshot_publish_period_sec must be > 0")
        if self.trusted_open_loop_hold_rate_hz <= 0.0:
            raise ValueError("trusted_open_loop_hold_rate_hz must be > 0")
        if self.joint_limit_epsilon_deg < 0.0:
            raise ValueError("joint_limit_epsilon_deg must be >= 0")
        valid_joint_state_source_modes = {"dry_run_fake", "hardware_feedback", "trusted_open_loop"}
        if self.joint_state_source_mode not in valid_joint_state_source_modes:
            raise ValueError(
                "joint_state_source_mode must be one of "
                f"{sorted(valid_joint_state_source_modes)}")

        # ── State machine ─────────────────────────────────────────────────
        self._state: BridgeState               = BridgeState.STARTUP
        self._fault_cleared_time: Optional[float] = None
        self._consecutive_serial_failures       = 0
        self._consecutive_partial_applies       = 0
        self._startup_serial_lines              = 0  # lines received from ESP32 during STARTUP

        # ── Command-epoch / re-arm bookkeeping ───────────────────────────
        # Every fault / disarm / (configurable) recovery increments _command_epoch.
        # The operator/UI can log epoch transitions to trace stale-replay events.
        self._command_epoch                    = 0
        self._rearm_requested                  = False
        self._requires_rearm_entered_sec: Optional[float] = None

        # Settle-based auto-rearm tracking (only used if threshold > 0).
        self._settle_last_positions_deg: dict[str, float] = {}
        self._settle_start_sec: Optional[float]           = None

        # ── Motion state ──────────────────────────────────────────────────
        self._serial: Optional[Serial]             = None
        self._latest_deg_by_joint: dict[str, float]    = {}
        self._latest_controller_desired_deg_by_joint: dict[str, float] = {}
        self._latest_controller_desired_vel_dps_by_joint: dict[str, float] = {}
        self._latest_controller_desired_accel_dps2_by_joint: dict[str, float] = {}
        self._active_trajectory_points: list[tuple[float, dict[str, float]]] = []
        self._commanded_deg_by_joint: dict[str, float] = {}
        self._command_velocity_dps_by_joint: dict[str, float] = {}
        self._last_sent_deg_by_joint: dict[str, float] = {}
        self._last_joint_state_time    = self.get_clock().now()
        self._active_trajectory_start_sec: Optional[float] = None
        self._active_trajectory_end_sec: Optional[float] = None
        self._last_controller_state_time = self.get_clock().now()
        self._startup_ready_time       = (
            self.get_clock().now().nanoseconds / 1e9 + self.startup_delay_sec)
        self._last_publish_time_sec    = self.get_clock().now().nanoseconds / 1e9
        self._timeout_warned           = False
        self._joint_state_timeout_disarmed = False
        self._serial_read_buf          = b""
        self._active_command_source: Optional[str] = None
        self._active_action_goal_handle = None
        self._action_execution_active = False
        self._trusted_open_loop_hold_active = False
        self._last_trusted_open_loop_hold_stream_sec: Optional[float] = None
        self._serial_io_lock = threading.RLock()
        self._last_hardware_snapshot: Optional[CurrentStateSnapshot] = None
        self._last_hardware_snapshot_monotonic_sec: Optional[float] = None
        self._last_hardware_snapshot_publish_request_sec: Optional[float] = None
        self._last_cached_joint_state_republish_sec: Optional[float] = None
        self._startup_hardware_snapshot_requested = False

        self._callback_group = ReentrantCallbackGroup()

        self._open_serial_port()

        # BEST_EFFORT + depth=1: on a Jetson-over-LAN path, this ensures we
        # always act on the freshest joint_state and never process a stale queue.
        # The publisher (MoveIt joint_state_publisher) is RELIABLE so it can
        # serve a BEST_EFFORT subscriber without issues.
        joint_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            joint_state_qos,
            callback_group=self._callback_group,
        )
        self._hardware_joint_state_pub = self.create_publisher(
            JointState,
            self.joint_state_topic,
            10,
        )
        trajectory_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._trajectory_subs = [
            self.create_subscription(
                JointTrajectory,
                topic,
                lambda msg, topic=topic: self._on_joint_trajectory(msg, topic),
                trajectory_qos,
                callback_group=self._callback_group,
            )
            for topic in dict.fromkeys(self.trajectory_topics)
        ]
        controller_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._controller_state_subs = [
            self.create_subscription(
                JointTrajectoryControllerState,
                topic,
                self._on_controller_state,
                controller_state_qos,
                callback_group=self._callback_group,
            )
            for topic in dict.fromkeys(self.controller_state_topics)
        ]
        self.create_timer(
            1.0 / self.command_rate_hz,
            self._publish_latest_targets,
            callback_group=self._callback_group,
        )

        # ── Re-arm / disarm services ─────────────────────────────────────
        # /rearm  — operator handshake to leave REQUIRES_REARM and enter ACTIVE.
        # /disarm — forces the bridge into FAULT regardless of current state,
        #           sends jx6 to the ESP32, and waits for explicit re-arm.
        self._rearm_srv = self.create_service(
            Trigger, "~/rearm",  self._handle_rearm_service,
            callback_group=self._callback_group)
        self._disarm_srv = self.create_service(
            Trigger, "~/disarm", self._handle_disarm_service,
            callback_group=self._callback_group)
        self._request_snapshot_srv = self.create_service(
            Trigger, "~/request_hardware_snapshot", self._handle_request_snapshot_service,
            callback_group=self._callback_group)
        self._follow_joint_trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.follow_joint_trajectory_action_name,
            execute_callback=self._execute_follow_joint_trajectory,
            goal_callback=self._handle_follow_joint_trajectory_goal,
            cancel_callback=self._handle_follow_joint_trajectory_cancel,
            callback_group=self._callback_group,
        )

        summary = ", ".join(
            f"{m.moveit_name}[{m.min_deg:.0f},{m.max_deg:.0f}]"
            for m in JOINT_MAPPINGS)
        limit_summary = ", ".join(
            f"{mapping.moveit_name}:v={self._joint_max_velocity_dps(mapping):.1f}dps/"
            f"a={self._joint_max_acceleration_dps2(mapping):.1f}dps²"
            for mapping in JOINT_MAPPINGS
        )
        self.get_logger().info(
            f"[{self._state.value}] Bridge up: {self.joint_state_topic} → "
            f"{self.serial_port} @{self.baud_rate}baud {self.command_rate_hz:.0f}Hz; "
            f"startup_delay={self.startup_delay_sec:.1f}s "
            f"startup_min_lines={self.startup_min_serial_lines} "
            f"joint_topic={self.joint_state_topic} "
            f"trajectory_topics={self.trajectory_topics or ['disabled']} "
            f"prefer_trajectory_topic={self.prefer_trajectory_topic} "
            f"trajectory_hold={self.trajectory_hold_sec:.2f}s "
            f"controller_topics={self.controller_state_topics or ['disabled']} "
            f"prefer_controller_state={self.prefer_controller_state} "
            f"controller_timeout={self.controller_state_timeout_sec:.2f}s "
            f"fault_recovery={self.fault_recovery_delay_sec:.1f}s "
            f"serial_fault_thr={self.serial_fault_threshold} "
            f"partial_thr={self.partial_apply_threshold}; "
            f"require_explicit_rearm={self.require_explicit_rearm} "
            f"jx6_on_fault={self.send_jx6_on_fault} "
            f"auto_rearm_settle={self.auto_rearm_settle_threshold_deg:.2f}°"
            f"/{self.auto_rearm_settle_dwell_sec:.1f}s; "
            f"max_vel={self.max_command_velocity_dps:.1f}dps "
            f"max_accel={self.max_command_acceleration_dps2:.1f}dps²; "
            f"action={self.follow_joint_trajectory_action_name} "
            f"dry_run={self.dry_run} "
            f"enable_hardware_motion={self.enable_hardware_motion} "
            f"joint_state_source_mode={self.joint_state_source_mode} "
            f"require_trusted_joint_state_for_hardware="
            f"{self.require_trusted_joint_state_for_hardware} "
            f"allow_fake_joint_states_for_hardware="
            f"{self.allow_fake_joint_states_for_hardware} "
            f"trusted_joint_state_publishers="
            f"{self.trusted_joint_state_publishers or ['not configured']} "
            f"real_motion_allowed={self._hardware_motion_enabled()} "
            f"hardware_snapshot_timeout={self.hardware_snapshot_timeout_sec:.2f}s "
            f"pre_snapshot_required={self.require_pre_motion_hardware_snapshot} "
            f"post_snapshot_required={self.require_post_motion_hardware_snapshot} "
            f"snapshot_publish_for_planning={self.publish_hardware_snapshots_for_planning} "
            f"periodic_snapshot_refresh={self.periodic_hardware_snapshot_refresh} "
            f"republish_cached_joint_states={self.republish_cached_hardware_joint_states} "
            f"publish_trusted_open_loop_commanded_joint_states="
            f"{self.publish_trusted_open_loop_commanded_joint_states} "
            f"idle_on_action_cancel={self.idle_on_action_cancel} "
            f"trusted_open_loop_hold_after_action="
            f"{self.trusted_open_loop_hold_after_action} "
            f"trusted_open_loop_hold_rate={self.trusted_open_loop_hold_rate_hz:.1f}Hz "
            f"snapshot_publish_period={self.hardware_snapshot_publish_period_sec:.2f}s "
            f"startup_snapshot={self.request_startup_hardware_snapshot} "
            f"joint_limit_epsilon={self.joint_limit_epsilon_deg:.3f}deg "
            f"allow_dry_run_without_active={self.allow_dry_run_without_active_hardware}; "
            f"mappings: {summary}"
        )
        self.get_logger().warn(
            "Hardware snapshot policy: qjs6 is requested only by manual service, "
            "/rearm, pre/post real trajectory snapshots, optional startup snapshot, "
            "or explicit periodic_hardware_snapshot_refresh:=true. Cached "
            "/joint_states republishing does not query the ESP32.")
        self.get_logger().warn(
            "Published hardware /joint_states include fixed flipper joints at 0.0 rad: "
            f"{', '.join(FIXED_JOINT_STATE_RAD)}")
        if self.joint_state_source_mode == "trusted_open_loop":
            self.get_logger().warn(
                "trusted_open_loop /joint_states are commanded open-loop positions "
                "published for MoveIt/RViz synchronization; they are not measured "
                "encoder feedback.")
        self.get_logger().warn(
            f"Hardware-side per-joint command limits: {limit_summary}")
        if self.dry_run:
            self.get_logger().warn(
                "DRY-RUN MODE: FollowJointTrajectory goals will be sampled and logged, "
                "but no serial j6 motion commands will be sent.")
        elif not self.enable_hardware_motion:
            self.get_logger().warn(
                "Hardware motion disabled: set enable_hardware_motion:=true together "
                "with dry_run:=false to allow serial j6 commands.")

    # ── Serial port management ────────────────────────────────────────────

    def _open_serial_port(self) -> None:
        try:
            self._serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.0,
                write_timeout=self.write_timeout_sec,
            )
            self._serial_read_buf = b""
        except SerialException as exc:
            self._serial = None
            if self.dry_run:
                self.get_logger().warn(
                    f"Serial port {self.serial_port} unavailable in dry-run mode: {exc}. "
                    "Continuing without hardware serial access.")
                return
            raise RuntimeError(
                f"Failed to open serial port {self.serial_port}: {exc}") from exc

    def _reopen_serial_port(self) -> bool:
        """Close and reopen serial; clears motion state to prevent jumps."""
        try:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
            self._serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.0,
                write_timeout=self.write_timeout_sec,
            )
            self._serial_read_buf = b""
            self._clear_motion_state()
            self.get_logger().info(f"Reopened serial port {self.serial_port}")
            return True
        except SerialException as exc:
            self.get_logger().warn(
                f"Failed to reopen {self.serial_port}: {exc}",
                throttle_duration_sec=5.0)
            return False

    # ── State machine helpers ─────────────────────────────────────────────

    def _clear_motion_state(self) -> None:
        """Zero all motion tracking. Must be followed by a sync before commands resume."""
        self._commanded_deg_by_joint.clear()
        self._command_velocity_dps_by_joint.clear()
        self._last_sent_deg_by_joint.clear()
        self._active_trajectory_points.clear()
        self._active_trajectory_start_sec = None
        self._active_trajectory_end_sec = None
        self._active_command_source = None
        self._trusted_open_loop_hold_active = False
        self._last_trusted_open_loop_hold_stream_sec = None

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def _sync_commanded_from_latest(self) -> None:
        """Initialise commanded positions from the latest known physical positions.

        Called when transitioning to ACTIVE so that the first sextet command
        tells motors to stay where they are (not jump to a stale commanded value).
        This is critical after a MoveIt plan moves the arm while servo was paused.
        """
        synced: list[str] = []
        for mapping in JOINT_MAPPINGS:
            pos = self._latest_deg_by_joint.get(mapping.moveit_name)
            if pos is not None:
                self._commanded_deg_by_joint[mapping.moveit_name]           = pos
                self._command_velocity_dps_by_joint[mapping.moveit_name]    = 0.0
                self._last_sent_deg_by_joint[mapping.moveit_name]           = pos
                synced.append(f"{mapping.moveit_name}={pos:.1f}°")
        if synced:
            self.get_logger().info(f"Position sync: {', '.join(synced)}")

    def _enter_fault(self, reason: str) -> None:
        if self._state == BridgeState.FAULT:
            return
        self._state = BridgeState.FAULT
        self._clear_motion_state()
        self._consecutive_serial_failures  = 0
        self._consecutive_partial_applies  = 0
        # Bump the command epoch: any state derived from the previous epoch
        # (bridge cache, ESP32 last-target cache, MoveIt planning snapshot) is
        # now considered invalid and must not be replayed.
        self._command_epoch += 1
        # Send jx6 to the ESP32 so its streamer stops replaying the last
        # target once motors re-energise.  Best-effort — we are already in FAULT.
        if self.send_jx6_on_fault:
            try:
                self._send_idle_to_all_joints()
            except Exception as exc:  # pragma: no cover — defensive
                self.get_logger().warn(
                    f"[FAULT] jx6 send failed: {exc}")
        self.get_logger().error(
            f"[STATE → FAULT epoch={self._command_epoch}]  {reason} — "
            f"commands suppressed, motion state cleared, jx6 sent to ESP32; "
            f"waiting for 'Failsafe cleared' from ESP32")

    def _enter_recovery(self, reason: str) -> None:
        if self._state not in (BridgeState.FAULT,):
            return
        self._state = BridgeState.RECOVERY
        self._fault_cleared_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().warn(
            f"[STATE → RECOVERY]  {reason} — "
            f"hold-off {self.fault_recovery_delay_sec:.1f}s before re-arm gate")

    def _enter_requires_rearm(self, reason: str) -> None:
        """Safe-idle state that blocks all motion until operator handshake.

        Reached after STARTUP completes, after RECOVERY hold-off, or after
        /disarm.  Exits via /rearm service or (opt-in) settle-based auto-rearm.
        """
        prev = self._state
        if prev == BridgeState.REQUIRES_REARM:
            return
        self._state = BridgeState.REQUIRES_REARM
        self._requires_rearm_entered_sec = self.get_clock().now().nanoseconds / 1e9
        self._rearm_requested            = False
        self._settle_start_sec           = None
        self._settle_last_positions_deg.clear()
        # Make sure no stale commanded positions linger; a fresh sync happens
        # on /rearm before the first sextet is emitted.
        self._clear_motion_state()
        self.get_logger().warn(
            f"[STATE → REQUIRES_REARM epoch={self._command_epoch}]  {reason} — "
            f"motion blocked; call {self.get_name()}/rearm to resume "
            f"(auto-rearm: "
            f"{'enabled' if self.auto_rearm_settle_threshold_deg > 0 else 'disabled'})")

    def _enter_active(self, reason: str) -> None:
        prev = self._state
        self._state = BridgeState.ACTIVE
        self._fault_cleared_time          = None
        self._consecutive_serial_failures  = 0
        self._consecutive_partial_applies  = 0
        self._rearm_requested              = False
        if prev != BridgeState.ACTIVE:
            # Sync positions so first sextet = hold-current-position, not jump.
            self._sync_commanded_from_latest()
            self.get_logger().info(
                f"[STATE → ACTIVE epoch={self._command_epoch}]  {reason}")

    # ── Re-arm / disarm service handlers ──────────────────────────────────

    def _handle_rearm_service(self, request, response):  # type: ignore[no-untyped-def]
        del request
        if self._state != BridgeState.REQUIRES_REARM:
            response.success = False
            response.message = (
                f"/rearm ignored: current state is {self._state.value}; "
                f"only REQUIRES_REARM accepts /rearm")
            self.get_logger().warn(response.message)
            return response

        if (
            self._real_hardware_motion_requested()
            and self.joint_state_source_mode == "hardware_feedback"
        ):
            snapshot, message = self.request_hardware_joint_snapshot(
                use_cached=False,
                reason="/rearm hardware state sync",
            )
            if snapshot is None:
                response.success = False
                response.message = (
                    f"/rearm rejected: hardware qjs6 snapshot failed: {message}")
                self.get_logger().error(response.message)
                return response
        elif (
            self._real_hardware_motion_requested()
            and self.joint_state_source_mode == "trusted_open_loop"
        ):
            age_sec = (self.get_clock().now() - self._last_joint_state_time).nanoseconds / 1e9
            have_fresh_positions = (
                bool(self._latest_deg_by_joint)
                and age_sec <= self.joint_state_timeout_sec
            )
            if not have_fresh_positions:
                snapshot = CurrentStateSnapshot(
                    deg_by_joint={
                        mapping.moveit_name: float(value)
                        for mapping, value in zip(
                            JOINT_MAPPINGS,
                            self.trusted_open_loop_initial_joint_positions_deg,
                        )
                    },
                    source="trusted_open_loop_initial_joint_positions_deg",
                )
                self._last_hardware_snapshot = snapshot
                self._last_hardware_snapshot_monotonic_sec = time.monotonic()
                self._publish_hardware_snapshot_joint_state(snapshot)
                self.get_logger().warn(
                    "/rearm trusted_open_loop: no qjs6 required; published "
                    "trusted_open_loop_initial_joint_positions_deg as the current "
                    "ROS joint state for MoveIt planning.")

        # Require at least one fresh joint_state so the ACTIVE entry sync is
        # meaningful — otherwise the first sextet would use whatever stale cache
        # we had before.
        age_sec = (self.get_clock().now() - self._last_joint_state_time).nanoseconds / 1e9
        if not self._latest_deg_by_joint or age_sec > self.joint_state_timeout_sec:
            response.success = False
            response.message = (
                f"/rearm rejected: no fresh joint_states "
                f"(age={age_sec:.2f}s, have_positions={bool(self._latest_deg_by_joint)})")
            self.get_logger().error(response.message)
            return response

        self._rearm_requested = True
        response.success = True
        response.message = (
            f"/rearm accepted at epoch={self._command_epoch}; "
            f"ACTIVE will begin on next timer tick with fresh position sync")
        self.get_logger().info(response.message)
        return response

    def _handle_request_snapshot_service(self, request, response):  # type: ignore[no-untyped-def]
        del request
        if self._action_execution_active:
            response.success = False
            response.message = (
                "hardware qjs6 snapshot rejected: FollowJointTrajectory execution is active")
            self.get_logger().warn(response.message)
            return response

        snapshot, message = self.request_hardware_joint_snapshot(
            use_cached=False,
            reason="manual request_hardware_snapshot service",
        )
        if snapshot is None:
            response.success = False
            response.message = f"hardware qjs6 snapshot failed: {message}"
            self.get_logger().error(response.message)
            return response

        summary = ", ".join(
            f"{mapping.moveit_name}={snapshot.deg_by_joint[mapping.moveit_name]:.3f}deg"
            for mapping in JOINT_MAPPINGS
        )
        response.success = True
        response.message = (
            f"hardware qjs6 snapshot accepted and cached; {summary}; "
            f"fixed flipper joints published as 0.0 rad")
        self.get_logger().info(response.message)
        return response

    def _handle_disarm_service(self, request, response):  # type: ignore[no-untyped-def]
        del request
        self._enter_fault("disarm service invoked")
        # Immediately fall through to RECOVERY→REQUIRES_REARM path so the
        # operator doesn't have to wait for an ESP32 'Failsafe cleared' that
        # may never arrive if this was a precautionary disarm.
        self._state = BridgeState.RECOVERY
        self._fault_cleared_time = self.get_clock().now().nanoseconds / 1e9
        response.success = True
        response.message = (
            f"/disarm executed at epoch={self._command_epoch}; "
            f"will transition to REQUIRES_REARM after "
            f"{self.fault_recovery_delay_sec:.1f}s hold-off")
        self.get_logger().warn(response.message)
        return response

    # ── Serial input drain ────────────────────────────────────────────────

    def _handle_serial_status_line(self, line: str) -> None:
        upper = line.upper()
        normalized = " ".join(upper.split())

        # Match only real ESP32 failsafe event lines.  Status/help text can
        # legitimately contain words like "failsafe=no" or "failsafe-aware"
        # and must not kick the bridge into FAULT.
        is_cleared = normalized.startswith("FAILSAFE CLEARED")
        is_failsafe = normalized.startswith("FAILSAFE ") and not is_cleared
        is_moveit_applied = "MOVEIT " in upper and (" APPLIED" in upper or " COMMITTED" in upper)
        is_moveit_rejected = "MOVEIT " in upper and " REJECTED" in upper
        is_motion_rejected = is_moveit_rejected or "CANNOT SEND" in upper

        if is_failsafe:
            self._enter_fault(f"ESP32: {line}")
        elif is_cleared:
            self._enter_recovery(f"ESP32: {line}")
        elif "PARTIALLY APPLIED" in upper:
            self._consecutive_partial_applies += 1
            if self._consecutive_partial_applies >= self.partial_apply_threshold:
                self._enter_fault(
                    f"partial-apply count reached {self.partial_apply_threshold}x: {line}")
            else:
                self.get_logger().warn(
                    f"ESP32 partial apply ({self._consecutive_partial_applies}"
                    f"/{self.partial_apply_threshold}): {line}",
                    throttle_duration_sec=1.0)
            self._consecutive_serial_failures = 0
        elif "BUS-OFF" in upper or "ERROR_FAILTX" in upper:
            self._enter_fault(f"ESP32 CAN fault: {line}")
        else:
            if "SEXTET" in upper and ("APPLIED" in upper or "COMMITTED" in upper):
                self._consecutive_partial_applies = 0

        if is_failsafe or "BUS-OFF" in upper or "ERROR_FAILTX" in upper:
            self.get_logger().error(f"ESP32: {line}", throttle_duration_sec=1.0)
        elif (
            is_cleared
            or "FAILED"   in upper
            or "WARNING"  in upper
            or "WARN"     in upper
            or "TIMEOUT"  in upper
            or is_motion_rejected
            or "PARTIALLY APPLIED" in upper
            or upper.startswith("JS6ERR ")
        ):
            self.get_logger().warn(f"ESP32: {line}", throttle_duration_sec=1.0)
        elif is_moveit_applied:
            self.get_logger().info(f"ESP32: {line}", throttle_duration_sec=0.25)
        else:
            self.get_logger().debug(f"ESP32: {line}")

    def _drain_serial_input(self) -> None:
        """Non-blocking drain of ESP32 serial output.

        Prevents OS receive buffer overflow and surfaces ESP32 status to the
        ROS logger.  Also drives the BridgeState machine based on FAILSAFE /
        Failsafe-cleared keywords.
        """
        with self._serial_io_lock:
            if self._serial is None or not self._serial.is_open:
                return

            try:
                waiting = self._serial.in_waiting
                if waiting > 0:
                    self._serial_read_buf += self._serial.read(waiting)
            except SerialException as exc:
                self.get_logger().warn(
                    f"Serial read error: {exc}", throttle_duration_sec=5.0)
                return

            while b"\n" in self._serial_read_buf:
                line_bytes, self._serial_read_buf = self._serial_read_buf.split(b"\n", 1)
                try:
                    line = line_bytes.decode("ascii", errors="replace").strip()
                except Exception:
                    continue
                if not line:
                    continue

                if self._state == BridgeState.STARTUP:
                    self._startup_serial_lines += 1

                self._handle_serial_status_line(line)

            if len(self._serial_read_buf) > 1024:
                self.get_logger().warn(
                    "Serial read buffer overflow (no newline in 1 KB); discarding",
                    throttle_duration_sec=10.0)
                self._serial_read_buf = b""

    # ── JointState subscription ───────────────────────────────────────────

    def _on_joint_state(self, msg: JointState) -> None:
        positions_by_name = dict(zip(msg.name, msg.position))

        for mapping in JOINT_MAPPINGS:
            radians_value = positions_by_name.get(mapping.moveit_name)
            if radians_value is None or not math.isfinite(radians_value):
                continue

            physical_deg = mapping.moveit_rad_to_physical_deg(radians_value)
            if mapping.moveit_name == "Joint4":
                physical_deg += self.joint4_offset_deg
            if physical_deg < mapping.min_deg or physical_deg > mapping.max_deg:
                self.get_logger().warn(
                    f"{mapping.moveit_name} target {physical_deg:.2f}° outside "
                    f"[{mapping.min_deg:.1f}, {mapping.max_deg:.1f}]; clamping",
                    throttle_duration_sec=2.0)
                physical_deg = min(max(physical_deg, mapping.min_deg), mapping.max_deg)

            prev_deg = self._latest_deg_by_joint.get(mapping.moveit_name)
            self._latest_deg_by_joint[mapping.moveit_name] = physical_deg

            # Diagnostic: log large jumps (> 2°) at full rate so we can tell
            # whether /joint_states is interpolating or snapping to the goal.
            if (self._state == BridgeState.ACTIVE
                    and prev_deg is not None
                    and abs(physical_deg - prev_deg) > 2.0):
                self.get_logger().warn(
                    f"[DIAG] {mapping.moveit_name} jumped "
                    f"{prev_deg:.2f}° → {physical_deg:.2f}° "
                    f"(Δ={physical_deg - prev_deg:+.2f}°)")

        self._last_joint_state_time = self.get_clock().now()
        self._timeout_warned = False
        self._joint_state_timeout_disarmed = False

    def _on_controller_state(self, msg: JointTrajectoryControllerState) -> None:
        if not msg.joint_names or len(msg.desired.positions) != len(msg.joint_names):
            return

        positions_by_name = dict(zip(msg.joint_names, msg.desired.positions))
        velocities_by_name = (
            dict(zip(msg.joint_names, msg.desired.velocities))
            if len(msg.desired.velocities) == len(msg.joint_names)
            else {}
        )
        accelerations_by_name = (
            dict(zip(msg.joint_names, msg.desired.accelerations))
            if len(msg.desired.accelerations) == len(msg.joint_names)
            else {}
        )
        desired_deg_by_joint: dict[str, float] = {}
        desired_vel_dps_by_joint: dict[str, float] = {}
        desired_accel_dps2_by_joint: dict[str, float] = {}

        for mapping in JOINT_MAPPINGS:
            radians_value = positions_by_name.get(mapping.moveit_name)
            if radians_value is None or not math.isfinite(radians_value):
                return

            physical_deg = mapping.moveit_rad_to_physical_deg(radians_value)
            if mapping.moveit_name == "Joint4":
                physical_deg += self.joint4_offset_deg
            if physical_deg < mapping.min_deg or physical_deg > mapping.max_deg:
                self.get_logger().warn(
                    f"{mapping.moveit_name} controller target {physical_deg:.2f}° outside "
                    f"[{mapping.min_deg:.1f}, {mapping.max_deg:.1f}]; clamping",
                    throttle_duration_sec=2.0)
                physical_deg = min(max(physical_deg, mapping.min_deg), mapping.max_deg)

            desired_deg_by_joint[mapping.moveit_name] = physical_deg

            vel_rad_s = velocities_by_name.get(mapping.moveit_name)
            if vel_rad_s is not None and math.isfinite(vel_rad_s):
                desired_vel_dps_by_joint[mapping.moveit_name] = (
                    mapping.sign * math.degrees(float(vel_rad_s))
                )

            accel_rad_s2 = accelerations_by_name.get(mapping.moveit_name)
            if accel_rad_s2 is not None and math.isfinite(accel_rad_s2):
                desired_accel_dps2_by_joint[mapping.moveit_name] = (
                    mapping.sign * math.degrees(float(accel_rad_s2))
                )

        self._latest_controller_desired_deg_by_joint = desired_deg_by_joint
        self._latest_controller_desired_vel_dps_by_joint = desired_vel_dps_by_joint
        self._latest_controller_desired_accel_dps2_by_joint = desired_accel_dps2_by_joint
        self._last_controller_state_time = self.get_clock().now()
        self._timeout_warned = False
        self._joint_state_timeout_disarmed = False

    # ── FollowJointTrajectory action server ──────────────────────────────

    def _hardware_motion_enabled(self) -> bool:
        """True only for intentional, armed serial motion commands."""
        trusted_ok, _ = self._trusted_joint_state_ok_for_hardware()
        return (
            not self.dry_run
            and self.enable_hardware_motion
            and self._state == BridgeState.ACTIVE
            and trusted_ok
        )

    def _idle_serial_enabled(self) -> bool:
        """Idle is allowed in real enabled mode even during fault/cancel cleanup."""
        return not self.dry_run and self.enable_hardware_motion

    def _real_hardware_motion_requested(self) -> bool:
        return not self.dry_run and self.enable_hardware_motion

    def _joint_state_publisher_full_names(self) -> list[str]:
        try:
            publishers = self.get_publishers_info_by_topic(self.joint_state_topic)
        except Exception as exc:  # pragma: no cover - defensive around RMW introspection
            self.get_logger().warn(
                f"Could not inspect publishers for {self.joint_state_topic}: {exc}",
                throttle_duration_sec=5.0,
            )
            return []

        full_names: list[str] = []
        for publisher in publishers:
            namespace = getattr(publisher, "node_namespace", "") or ""
            name = getattr(publisher, "node_name", "") or ""
            if not name:
                continue
            if not namespace.startswith("/"):
                namespace = "/" + namespace
            full_name = f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
            full_names.append(full_name)
        return sorted(set(full_names))

    def _trusted_joint_state_ok_for_hardware(self) -> tuple[bool, str]:
        if not self._real_hardware_motion_requested():
            return True, "real hardware motion is not requested"

        if not self.require_trusted_joint_state_for_hardware:
            return True, "trusted joint-state requirement disabled by parameter"

        if self.joint_state_source_mode == "trusted_open_loop":
            return (
                True,
                "joint_state_source_mode=trusted_open_loop; hardware feedback is optional "
                "and real motion relies on embedded controller execution",
            )

        if self.joint_state_source_mode != "hardware_feedback":
            return (
                False,
                "real hardware motion is enabled but joint_state_source_mode is "
                f"{self.joint_state_source_mode}. Real encoder/motor feedback is required; "
                "set joint_state_source_mode:=hardware_feedback only when /joint_states "
                "comes from trusted hardware feedback.",
            )

        publisher_names = self._joint_state_publisher_full_names()
        fake_publishers = {
            "/joint_state_publisher",
            "/joint_state_publisher_gui",
            "/robot_state_publisher",
            "/rviz",
        }
        detected_fake_publishers = sorted(fake_publishers.intersection(publisher_names))
        if detected_fake_publishers and not self.allow_fake_joint_states_for_hardware:
            return (
                False,
                f"real hardware motion is blocked because {self.joint_state_topic} "
                f"has fake/untrusted publisher(s): {detected_fake_publishers}. "
                "Stop fake joint-state publishers or keep dry_run:=true.",
            )

        if self.trusted_joint_state_publishers:
            trusted = set(self.trusted_joint_state_publishers)
            detected_trusted = sorted(trusted.intersection(publisher_names))
            if not detected_trusted:
                return (
                    False,
                    f"real hardware motion is blocked because no configured trusted "
                    f"joint-state publisher is active on {self.joint_state_topic}. "
                    f"trusted_joint_state_publishers={self.trusted_joint_state_publishers}, "
                    f"detected_publishers={publisher_names or ['none']}",
                )

        return True, (
            f"joint_state_source_mode=hardware_feedback; "
            f"detected_publishers={publisher_names or ['none']}"
        )

    @staticmethod
    def _duration_to_sec(duration) -> float:  # type: ignore[no-untyped-def]
        return float(duration.sec) + float(duration.nanosec) / 1e9

    def _mapping_by_name(self) -> dict[str, JointMapping]:
        return {mapping.moveit_name: mapping for mapping in JOINT_MAPPINGS}

    def _joint_index(self, mapping: JointMapping) -> int:
        for index, candidate in enumerate(JOINT_MAPPINGS):
            if candidate.moveit_name == mapping.moveit_name:
                return index
        raise KeyError(mapping.moveit_name)

    def _joint_max_velocity_dps(self, mapping: JointMapping) -> float:
        return min(
            self.max_command_velocity_dps,
            self.joint_max_velocity_dps[self._joint_index(mapping)],
        )

    def _joint_max_acceleration_dps2(self, mapping: JointMapping) -> float:
        return min(
            self.max_command_acceleration_dps2,
            self.joint_max_acceleration_dps2[self._joint_index(mapping)],
        )

    def _moveit_rad_to_physical_deg(self, joint_name: str, radians_value: float) -> float:
        mapping = self._mapping_by_name()[joint_name]
        physical_deg = mapping.moveit_rad_to_physical_deg(radians_value)
        if joint_name == "Joint4":
            physical_deg += self.joint4_offset_deg
        return physical_deg

    def _physical_deg_to_moveit_rad(self, joint_name: str, physical_deg: float) -> float:
        mapping = self._mapping_by_name()[joint_name]
        adjusted_deg = physical_deg
        if joint_name == "Joint4":
            adjusted_deg -= self.joint4_offset_deg
        return math.radians(adjusted_deg / mapping.sign)

    def _positions_to_degrees(
        self,
        joint_names: list[str],
        positions: list[float] | tuple[float, ...],
    ) -> tuple[Optional[dict[str, float]], str]:
        if len(positions) != len(joint_names):
            return None, "point positions length does not match joint_names length"

        deg_by_joint: dict[str, float] = {}
        mappings = self._mapping_by_name()
        for joint_name, radians_value in zip(joint_names, positions):
            if joint_name not in mappings:
                return None, f"unexpected joint {joint_name}"
            if not math.isfinite(float(radians_value)):
                return None, f"{joint_name} position is not finite"

            physical_deg = self._moveit_rad_to_physical_deg(joint_name, float(radians_value))
            mapping = mappings[joint_name]
            if physical_deg < mapping.min_deg or physical_deg > mapping.max_deg:
                return (
                    None,
                    f"{joint_name} target {physical_deg:.2f} deg outside "
                    f"[{mapping.min_deg:.1f}, {mapping.max_deg:.1f}]",
                )
            deg_by_joint[joint_name] = physical_deg

        return deg_by_joint, ""

    def _parse_js6_line(self, line: str) -> tuple[Optional[CurrentStateSnapshot], str]:
        parts = line.split()
        if len(parts) != 7 or parts[0] != "js6":
            return None, "not a js6 line"

        values: list[float] = []
        for index, text in enumerate(parts[1:], start=1):
            try:
                value = float(text)
            except ValueError:
                return None, f"joint {index} value is not a float"
            if not math.isfinite(value):
                return None, f"joint {index} value is not finite"
            values.append(value)

        deg_by_joint = {
            mapping.moveit_name: value
            for mapping, value in zip(JOINT_MAPPINGS, values)
        }
        for mapping in JOINT_MAPPINGS:
            value = deg_by_joint[mapping.moveit_name]
            if value < mapping.min_deg:
                if mapping.min_deg - value <= self.joint_limit_epsilon_deg:
                    self.get_logger().warn(
                        f"{mapping.moveit_name}={value:.6f}deg is within "
                        f"joint_limit_epsilon_deg={self.joint_limit_epsilon_deg:.3f}; "
                        f"clamping to {mapping.min_deg:.3f}deg",
                        throttle_duration_sec=2.0,
                    )
                    deg_by_joint[mapping.moveit_name] = mapping.min_deg
                    continue
                return (
                    None,
                    f"{mapping.moveit_name} snapshot {value:.2f}deg outside "
                    f"[{mapping.min_deg:.1f}, {mapping.max_deg:.1f}]",
                )
            if value > mapping.max_deg:
                if value - mapping.max_deg <= self.joint_limit_epsilon_deg:
                    self.get_logger().warn(
                        f"{mapping.moveit_name}={value:.6f}deg is within "
                        f"joint_limit_epsilon_deg={self.joint_limit_epsilon_deg:.3f}; "
                        f"clamping to {mapping.max_deg:.3f}deg",
                        throttle_duration_sec=2.0,
                    )
                    deg_by_joint[mapping.moveit_name] = mapping.max_deg
                    continue
                return (
                    None,
                    f"{mapping.moveit_name} snapshot {value:.2f}deg outside "
                    f"[{mapping.min_deg:.1f}, {mapping.max_deg:.1f}]",
                )

        return CurrentStateSnapshot(deg_by_joint=deg_by_joint, source="qjs6"), ""

    def _publish_joint_state_from_physical_degrees(
        self,
        deg_by_joint: dict[str, float],
        *,
        source: str,
        cache_for_republish: bool,
    ) -> None:
        for mapping in JOINT_MAPPINGS:
            self._latest_deg_by_joint[mapping.moveit_name] = deg_by_joint[mapping.moveit_name]
        self._last_joint_state_time = self.get_clock().now()
        self._timeout_warned = False
        self._joint_state_timeout_disarmed = False

        if cache_for_republish:
            self._last_hardware_snapshot = CurrentStateSnapshot(
                deg_by_joint={
                    mapping.moveit_name: deg_by_joint[mapping.moveit_name]
                    for mapping in JOINT_MAPPINGS
                },
                source=source,
            )
            self._last_hardware_snapshot_monotonic_sec = time.monotonic()

        if self.joint_state_source_mode not in ("hardware_feedback", "trusted_open_loop"):
            return

        msg = JointState()
        msg.header.stamp = self._last_joint_state_time.to_msg()
        msg.name = (
            [mapping.moveit_name for mapping in JOINT_MAPPINGS]
            + list(FIXED_JOINT_STATE_RAD)
        )
        msg.position = [
            self._physical_deg_to_moveit_rad(mapping.moveit_name, deg_by_joint[mapping.moveit_name])
            for mapping in JOINT_MAPPINGS
        ] + list(FIXED_JOINT_STATE_RAD.values())
        self._hardware_joint_state_pub.publish(msg)

    def _publish_hardware_snapshot_joint_state(self, snapshot: CurrentStateSnapshot) -> None:
        self._publish_joint_state_from_physical_degrees(
            snapshot.deg_by_joint,
            source=snapshot.source,
            cache_for_republish=False,
        )

    def _publish_trusted_open_loop_commanded_joint_state(
        self,
        deg_by_joint: dict[str, float],
    ) -> None:
        if (
            self.joint_state_source_mode != "trusted_open_loop"
            or not self.publish_trusted_open_loop_commanded_joint_states
        ):
            return

        self._publish_joint_state_from_physical_degrees(
            deg_by_joint,
            source="trusted_open_loop_commanded_state",
            cache_for_republish=True,
        )

    def _maybe_publish_hardware_snapshot_for_planning(self, now_sec: float) -> None:
        """Refresh low-rate trusted joint_states for MoveIt planning/rearm.

        By default this only republishes the last accepted qjs6/js6 snapshot
        with a fresh ROS timestamp. It does not query the ESP32 unless explicit
        periodic refresh or optional startup refresh is enabled.
        """
        if self.joint_state_source_mode not in ("hardware_feedback", "trusted_open_loop"):
            return
        if self._action_execution_active:
            return

        last_publish = self._last_cached_joint_state_republish_sec
        publish_due = (
            last_publish is None
            or now_sec - last_publish >= self.hardware_snapshot_publish_period_sec
        )
        if (
            self.republish_cached_hardware_joint_states
            and publish_due
            and self._last_hardware_snapshot is not None
        ):
            self._publish_hardware_snapshot_joint_state(self._last_hardware_snapshot)
            self._last_cached_joint_state_republish_sec = now_sec
            self.get_logger().debug(
                "Republished cached hardware snapshot as /joint_states "
                "without sending qjs6")

        should_request_startup_snapshot = (
            self.request_startup_hardware_snapshot
            and not self._startup_hardware_snapshot_requested
            and self._real_hardware_motion_requested()
        )
        should_request_periodic_snapshot = (
            self.publish_hardware_snapshots_for_planning
            and self.periodic_hardware_snapshot_refresh
            and self._real_hardware_motion_requested()
        )
        if should_request_startup_snapshot:
            self._startup_hardware_snapshot_requested = True
        elif not should_request_periodic_snapshot:
            return

        last_request = self._last_hardware_snapshot_publish_request_sec
        if (
            last_request is not None
            and now_sec - last_request < self.hardware_snapshot_publish_period_sec
        ):
            return

        self._last_hardware_snapshot_publish_request_sec = now_sec
        reason = (
            "startup hardware state refresh"
            if should_request_startup_snapshot
            else "explicit periodic planning/rearm joint_state refresh"
        )
        snapshot, message = self.request_hardware_joint_snapshot(
            use_cached=False,
            reason=reason,
        )
        if snapshot is None:
            self.get_logger().warn(
                "Hardware snapshot qjs6 refresh failed: "
                f"{message}. MoveIt may refuse execution until /joint_states "
                "contains a recent full hardware state.",
                throttle_duration_sec=2.0,
            )

    def _fresh_cached_hardware_snapshot(self) -> Optional[CurrentStateSnapshot]:
        if (
            self._last_hardware_snapshot is None
            or self._last_hardware_snapshot_monotonic_sec is None
        ):
            return None
        age_sec = time.monotonic() - self._last_hardware_snapshot_monotonic_sec
        if age_sec <= self.hardware_snapshot_timeout_sec:
            return self._last_hardware_snapshot
        return None

    def request_hardware_joint_snapshot(
        self,
        *,
        use_cached: bool = False,
        reason: str = "unspecified",
    ) -> tuple[Optional[CurrentStateSnapshot], str]:
        """Request one measured six-joint snapshot from the ESP32 via qjs6/js6."""
        if use_cached:
            cached = self._fresh_cached_hardware_snapshot()
            if cached is not None:
                return cached, "using cached qjs6 snapshot"

        with self._serial_io_lock:
            if self._serial is None or not self._serial.is_open:
                self.get_logger().warn(
                    "Serial port not open for qjs6 snapshot; attempting reconnect",
                    throttle_duration_sec=2.0,
                )
                if not self._reopen_serial_port():
                    return None, "serial port unavailable for qjs6 snapshot"

            try:
                self._serial_read_buf = b""
                self._serial.write(b"qjs6\n")  # type: ignore[union-attr]
                self._serial.flush()           # type: ignore[union-attr]
            except SerialException as exc:
                self._reopen_serial_port()
                return None, f"failed to send qjs6 snapshot request: {exc}"

            deadline_sec = time.monotonic() + self.hardware_snapshot_timeout_sec
            last_error = "timed out waiting for js6"
            self.get_logger().info(
                f"Requested hardware joint snapshot via qjs6 ({reason}); "
                f"timeout={self.hardware_snapshot_timeout_sec:.2f}s"
            )

            while time.monotonic() <= deadline_sec:
                try:
                    waiting = self._serial.in_waiting  # type: ignore[union-attr]
                    if waiting > 0:
                        self._serial_read_buf += self._serial.read(waiting)  # type: ignore[union-attr]
                except SerialException as exc:
                    self._reopen_serial_port()
                    return None, f"serial read failed waiting for js6: {exc}"

                while b"\n" in self._serial_read_buf:
                    line_bytes, self._serial_read_buf = self._serial_read_buf.split(b"\n", 1)
                    line = line_bytes.decode("ascii", errors="replace").strip()
                    if not line:
                        continue
                    if line.startswith("js6err "):
                        last_error = line
                        self._handle_serial_status_line(line)
                        return None, last_error
                    if line.startswith("js6 "):
                        snapshot, parse_error = self._parse_js6_line(line)
                        if snapshot is None:
                            return None, f"invalid js6 response: {parse_error}"
                        self._last_hardware_snapshot = snapshot
                        self._last_hardware_snapshot_monotonic_sec = time.monotonic()
                        self._publish_hardware_snapshot_joint_state(snapshot)
                        summary = ", ".join(
                            f"{name}={deg:.3f}deg"
                            for name, deg in snapshot.deg_by_joint.items()
                        )
                        self.get_logger().info(
                            f"Hardware joint snapshot accepted from qjs6: {summary}"
                        )
                        return snapshot, "qjs6 snapshot received"

                    self._handle_serial_status_line(line)

                time.sleep(0.005)

            return None, last_error

    def _current_state_snapshot_for_action(self) -> tuple[Optional[CurrentStateSnapshot], str]:
        now = self.get_clock().now()
        joint_age_sec = (now - self._last_joint_state_time).nanoseconds / 1e9
        have_all_joint_states = all(
            mapping.moveit_name in self._latest_deg_by_joint for mapping in JOINT_MAPPINGS
        )
        if have_all_joint_states and joint_age_sec <= self.joint_state_timeout_sec:
            return (
                CurrentStateSnapshot(
                    deg_by_joint={
                        mapping.moveit_name: self._latest_deg_by_joint[mapping.moveit_name]
                        for mapping in JOINT_MAPPINGS
                    },
                    source="/joint_states",
                ),
                f"/joint_states age={joint_age_sec:.3f}s",
            )

        if self.dry_run and self.allow_dry_run_initial_joint_state:
            return (
                CurrentStateSnapshot(
                    deg_by_joint={
                        mapping.moveit_name: float(value)
                        for mapping, value in zip(
                            JOINT_MAPPINGS, self.dry_run_initial_joint_positions_deg
                        )
                    },
                    source="dry_run_initial_joint_positions_deg",
                ),
                (
                    "dry_run_initial_joint_positions_deg "
                    f"(no fresh /joint_states; age={joint_age_sec:.3f}s)"
                ),
            )

        return None, (
            f"unavailable (fresh /joint_states required; age={joint_age_sec:.3f}s, "
            f"have_all_joints={have_all_joint_states})"
        )

    def _validate_follow_joint_trajectory_goal(self, goal) -> TrajectoryValidation:  # type: ignore[no-untyped-def]
        if self.dry_run:
            if self._state != BridgeState.ACTIVE and not self.allow_dry_run_without_active_hardware:
                return TrajectoryValidation(
                    False,
                    "dry-run goal rejected because bridge is not ACTIVE and "
                    "allow_dry_run_without_active_hardware is false",
                )
        elif self._state != BridgeState.ACTIVE:
            return TrajectoryValidation(
                False,
                f"real hardware goal rejected because bridge state is {self._state.value}, not ACTIVE",
            )

        if not self.dry_run and not self.enable_hardware_motion:
            return TrajectoryValidation(
                False,
                "real hardware goal rejected because enable_hardware_motion is false",
            )

        trusted_ok, trusted_reason = self._trusted_joint_state_ok_for_hardware()
        if not trusted_ok:
            return TrajectoryValidation(
                False,
                f"real hardware goal rejected: {trusted_reason}",
            )

        if self._action_execution_active:
            return TrajectoryValidation(False, "another FollowJointTrajectory goal is already active")

        trajectory = goal.trajectory
        joint_names = list(trajectory.joint_names)
        expected_names = {mapping.moveit_name for mapping in JOINT_MAPPINGS}
        if len(joint_names) != len(expected_names):
            return TrajectoryValidation(
                False,
                f"goal must contain exactly {len(expected_names)} joints; got {len(joint_names)}",
            )
        if len(set(joint_names)) != len(joint_names):
            return TrajectoryValidation(False, "goal joint_names contains duplicate entries")
        if set(joint_names) != expected_names:
            missing = sorted(expected_names - set(joint_names))
            extra = sorted(set(joint_names) - expected_names)
            return TrajectoryValidation(
                False,
                f"goal joint_names mismatch; missing={missing or []}, extra={extra or []}",
            )
        if not trajectory.points:
            return TrajectoryValidation(False, "goal trajectory contains no points")
        if len(trajectory.points) == 1 and not self.allow_single_point_goal:
            return TrajectoryValidation(
                False,
                "one-point trajectory goals are disabled; provide at least two timed points",
            )

        trusted_open_loop = (
            self._real_hardware_motion_requested()
            and self.joint_state_source_mode == "trusted_open_loop"
        )
        if (
            self._real_hardware_motion_requested()
            and self.require_pre_motion_hardware_snapshot
            and not trusted_open_loop
        ):
            current_state, snapshot_message = self.request_hardware_joint_snapshot(
                use_cached=True,
                reason="pre-motion validation",
            )
            source_desc = f"hardware snapshot: {snapshot_message}"
            if current_state is None:
                return TrajectoryValidation(
                    False,
                    f"pre-motion hardware snapshot unavailable: {snapshot_message}",
                )
        else:
            current_state, source_desc = self._current_state_snapshot_for_action()
        self.get_logger().info(
            f"FollowJointTrajectory current-state source: {source_desc}",
            throttle_duration_sec=0.5,
        )
        if current_state is None and not trusted_open_loop:
            return TrajectoryValidation(False, f"current joint state unavailable: {source_desc}")
        if (
            not self.dry_run
            and not trusted_open_loop
            and current_state.source not in ("/joint_states", "qjs6")
        ):
            return TrajectoryValidation(
                False,
                "real hardware motion requires current joint state from /joint_states "
                "or qjs6 hardware snapshot",
            )

        timed_points: list[TimedJointTarget] = []
        previous_time_sec: Optional[float] = None
        previous_deg_by_joint: Optional[dict[str, float]] = None

        for index, point in enumerate(trajectory.points):
            deg_by_joint, error = self._positions_to_degrees(joint_names, list(point.positions))
            if deg_by_joint is None:
                return TrajectoryValidation(False, f"point {index}: {error}")

            point_time_sec = self._duration_to_sec(point.time_from_start)
            if not math.isfinite(point_time_sec) or point_time_sec < 0.0:
                return TrajectoryValidation(
                    False,
                    f"point {index}: time_from_start must be finite and non-negative",
                )

            if previous_time_sec is not None and previous_deg_by_joint is not None:
                if point_time_sec + self.zero_duration_epsilon_sec < previous_time_sec:
                    return TrajectoryValidation(
                        False,
                        f"point {index}: time_from_start decreases",
                    )

                duration_sec = point_time_sec - previous_time_sec
                max_delta_deg = max(
                    abs(deg_by_joint[mapping.moveit_name]
                        - previous_deg_by_joint[mapping.moveit_name])
                    for mapping in JOINT_MAPPINGS
                )
                if duration_sec <= self.zero_duration_epsilon_sec:
                    if max_delta_deg > self.position_epsilon_deg:
                        return TrajectoryValidation(
                            False,
                            f"point {index}: moving segment has zero/near-zero duration "
                            f"({duration_sec:.6f}s, delta={max_delta_deg:.3f}deg)",
                        )
                else:
                    segment_velocity_dps = max_delta_deg / duration_sec
                    if segment_velocity_dps > self.max_command_velocity_dps + 1e-6:
                        return TrajectoryValidation(
                            False,
                            f"point {index}: segment velocity {segment_velocity_dps:.2f}deg/s "
                            f"exceeds max_command_velocity_dps={self.max_command_velocity_dps:.2f}",
                        )

            timed_points.append(TimedJointTarget(point_time_sec, deg_by_joint))
            previous_time_sec = point_time_sec
            previous_deg_by_joint = deg_by_joint

        first_point = timed_points[0].deg_by_joint
        if current_state is None and trusted_open_loop:
            current_state = CurrentStateSnapshot(
                deg_by_joint=dict(first_point),
                source="trusted_open_loop_first_trajectory_point",
            )
            self.get_logger().warn(
                "trusted_open_loop: no pre-motion qjs6 required; assuming the "
                "first trajectory point is the current command start state.")
        start_deltas = {
            mapping.moveit_name: abs(
                first_point[mapping.moveit_name]
                - current_state.deg_by_joint[mapping.moveit_name]
            )
            for mapping in JOINT_MAPPINGS
        }
        max_start_delta = max(start_deltas.values())
        if max_start_delta > self.first_point_max_delta_deg:
            worst_joint = max(start_deltas, key=start_deltas.get)
            return TrajectoryValidation(
                False,
                f"first trajectory point is too far from current state: "
                f"{worst_joint} delta={max_start_delta:.2f}deg > "
                f"first_point_max_delta_deg={self.first_point_max_delta_deg:.2f}",
            )
        if max_start_delta > self.goal_start_tolerance_deg:
            worst_joint = max(start_deltas, key=start_deltas.get)
            return TrajectoryValidation(
                False,
                f"first trajectory point fails start tolerance: "
                f"{worst_joint} delta={max_start_delta:.2f}deg > "
                f"goal_start_tolerance_deg={self.goal_start_tolerance_deg:.2f}",
            )

        return TrajectoryValidation(
            True,
            f"accepted {len(timed_points)} points over {timed_points[-1].time_from_start_sec:.3f}s",
            tuple(timed_points),
            current_state,
        )

    def _handle_follow_joint_trajectory_goal(self, goal_request):  # type: ignore[no-untyped-def]
        validation = self._validate_follow_joint_trajectory_goal(goal_request)
        mode = "dry-run" if self.dry_run else "real-hardware"
        if not validation.ok:
            self.get_logger().error(
                f"Rejecting FollowJointTrajectory goal in {mode} mode: {validation.message}"
            )
            return GoalResponse.REJECT

        source = validation.current_state.source if validation.current_state else "unavailable"
        self.get_logger().info(
            f"Accepting FollowJointTrajectory goal in {mode} mode: "
            f"{validation.message}; current_state_source={source}"
        )
        return GoalResponse.ACCEPT

    def _handle_follow_joint_trajectory_cancel(self, goal_handle):  # type: ignore[no-untyped-def]
        del goal_handle
        self.get_logger().warn("FollowJointTrajectory cancel requested; accepting")
        return CancelResponse.ACCEPT

    def _sample_action_trajectory(
        self,
        points: tuple[TimedJointTarget, ...],
        elapsed_sec: float,
    ) -> dict[str, float]:
        if elapsed_sec <= points[0].time_from_start_sec:
            return dict(points[0].deg_by_joint)

        for index in range(1, len(points)):
            prev_point = points[index - 1]
            next_point = points[index]
            if elapsed_sec > next_point.time_from_start_sec:
                continue

            duration_sec = next_point.time_from_start_sec - prev_point.time_from_start_sec
            if duration_sec <= self.zero_duration_epsilon_sec:
                return dict(next_point.deg_by_joint)

            alpha = (elapsed_sec - prev_point.time_from_start_sec) / duration_sec
            return {
                mapping.moveit_name: (
                    prev_point.deg_by_joint[mapping.moveit_name]
                    + alpha * (
                        next_point.deg_by_joint[mapping.moveit_name]
                        - prev_point.deg_by_joint[mapping.moveit_name]
                    )
                )
                for mapping in JOINT_MAPPINGS
            }

        return dict(points[-1].deg_by_joint)

    def _build_follow_feedback(
        self,
        joint_names: list[str],
        desired_deg_by_joint: dict[str, float],
        actual_deg_by_joint: dict[str, float],
    ):
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names
        for joint_name in joint_names:
            desired_rad = self._physical_deg_to_moveit_rad(
                joint_name, desired_deg_by_joint[joint_name])
            actual_rad = self._physical_deg_to_moveit_rad(
                joint_name, actual_deg_by_joint[joint_name])
            feedback.desired.positions.append(desired_rad)
            feedback.actual.positions.append(actual_rad)
            feedback.error.positions.append(desired_rad - actual_rad)
        return feedback

    def _dispatch_joint_sextet(self, command_deg_sextet: list[float], source: str) -> bool:
        line = "j6 " + " ".join(f"{d:.3f}" for d in command_deg_sextet)
        if not self._hardware_motion_enabled():
            _, block_reason = self._trusted_joint_state_ok_for_hardware()
            if self.dry_run:
                self.get_logger().info(
                    f"[DRY-RUN {source}] serial motion suppressed: {line}",
                    throttle_duration_sec=0.2,
                )
            else:
                self.get_logger().warn(
                    f"[MOTION DISABLED {source}] serial motion suppressed "
                    f"(state={self._state.value}, enable_hardware_motion="
                    f"{self.enable_hardware_motion}, joint_state_source_mode="
                    f"{self.joint_state_source_mode}, reason={block_reason}): {line}",
                    throttle_duration_sec=1.0,
                )
            return True
        return self._send_joint_sextet(command_deg_sextet)

    def _execute_follow_joint_trajectory(self, goal_handle):  # type: ignore[no-untyped-def]
        validation = self._validate_follow_joint_trajectory_goal(goal_handle.request)
        result = FollowJointTrajectory.Result()
        if not validation.ok:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = validation.message
            self.get_logger().error(
                f"Aborting FollowJointTrajectory before execution: {validation.message}"
            )
            goal_handle.abort()
            return result

        assert validation.current_state is not None
        points = validation.points
        joint_names = list(goal_handle.request.trajectory.joint_names)
        mode = "DRY-RUN" if self.dry_run else "REAL HARDWARE"
        if self.dry_run:
            self.get_logger().warn(
                "FollowJointTrajectory executing in DRY-RUN mode: timing loop will run, "
                "sampled j6 commands will be logged, and serial motion is suppressed."
            )
        else:
            self.get_logger().warn(
                "FollowJointTrajectory executing in REAL HARDWARE mode: serial j6 commands "
                "are enabled only while bridge remains ACTIVE."
            )
        self.get_logger().info(
            f"FollowJointTrajectory {mode}: {len(points)} points, "
            f"duration={points[-1].time_from_start_sec:.3f}s, "
            f"current_state_source={validation.current_state.source}"
        )
        trusted_open_loop_real_motion = (
            self._real_hardware_motion_requested()
            and self.joint_state_source_mode == "trusted_open_loop"
        )
        tracking_error_abort_enabled = not trusted_open_loop_real_motion
        if trusted_open_loop_real_motion:
            self.get_logger().warn(
                "trusted_open_loop: trajectory tracking lag will be logged but will "
                "not abort execution; the bridge will continue ramp-limited commands "
                "until the final target is reached within goal_tolerance_deg.")
            if self.publish_trusted_open_loop_commanded_joint_states:
                self.get_logger().warn(
                    "trusted_open_loop: publishing commanded /joint_states during "
                    "execution for MoveIt/RViz synchronization. These states are not "
                    "measured hardware feedback.")

        self._action_execution_active = True
        self._active_action_goal_handle = goal_handle
        self._trusted_open_loop_hold_active = False
        self._last_trusted_open_loop_hold_stream_sec = None
        self._commanded_deg_by_joint = dict(validation.current_state.deg_by_joint)
        self._command_velocity_dps_by_joint = {
            mapping.moveit_name: 0.0 for mapping in JOINT_MAPPINGS
        }
        self._last_sent_deg_by_joint = dict(validation.current_state.deg_by_joint)

        period_sec = 1.0 / self.command_rate_hz
        trajectory_duration_sec = points[-1].time_from_start_sec
        tracking_error_started_sec: Optional[float] = None
        start_wall_sec = time.monotonic()
        next_tick_sec = start_wall_sec
        command_count = 0
        final_commanded_deg_by_joint = dict(validation.current_state.deg_by_joint)

        try:
            while True:
                now_wall_sec = time.monotonic()
                elapsed_sec = max(0.0, now_wall_sec - start_wall_sec)
                # Execute the full trajectory over time: sample/interpolate by timestamp,
                # then rate-limit toward that sample. Never consume only the final point.
                desired_deg_by_joint = self._sample_action_trajectory(points, elapsed_sec)

                if goal_handle.is_cancel_requested:
                    if self.idle_on_action_cancel:
                        self.get_logger().warn(
                            "FollowJointTrajectory canceled; idle_on_action_cancel=true, "
                            "sending jx6 idle to hardware."
                        )
                        self._send_idle_to_all_joints()
                    else:
                        if (
                            trusted_open_loop_real_motion
                            and self.trusted_open_loop_hold_after_action
                            and command_count > 0
                        ):
                            self._trusted_open_loop_hold_active = True
                            self._last_trusted_open_loop_hold_stream_sec = None
                        hold_summary = ", ".join(
                            f"{mapping.moveit_name}="
                            f"{self._last_sent_deg_by_joint.get(mapping.moveit_name, 0.0):.3f}deg"
                            for mapping in JOINT_MAPPINGS
                        )
                        self.get_logger().error(
                            "FollowJointTrajectory canceled by client/MoveIt timeout; "
                            "NOT sending jx6 because idle_on_action_cancel=false. "
                            "Holding last streamed target via bridge/firmware hold streaming: "
                            f"{hold_summary}. Use /disarm for an intentional motor idle."
                        )
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "Canceled by client"
                    goal_handle.canceled()
                    return result

                commanded_deg_sextet: list[float] = []
                next_commanded_deg_by_joint: dict[str, float] = {}
                next_velocity_dps_by_joint: dict[str, float] = {}
                dt_sec = max(period_sec, 1e-3)
                for mapping in JOINT_MAPPINGS:
                    current_deg = self._commanded_deg_by_joint.get(
                        mapping.moveit_name,
                        validation.current_state.deg_by_joint[mapping.moveit_name],
                    )
                    current_vel = self._command_velocity_dps_by_joint.get(
                        mapping.moveit_name, 0.0)
                    next_deg, next_vel = self._step_motion_toward_target(
                        current_deg,
                        current_vel,
                        desired_deg_by_joint[mapping.moveit_name],
                        dt_sec,
                        max_vel_dps=self._joint_max_velocity_dps(mapping),
                        max_accel_dps2=self._joint_max_acceleration_dps2(mapping),
                    )
                    commanded_deg_sextet.append(next_deg)
                    next_commanded_deg_by_joint[mapping.moveit_name] = next_deg
                    next_velocity_dps_by_joint[mapping.moveit_name] = next_vel

                if not self._dispatch_joint_sextet(commanded_deg_sextet, "FollowJointTrajectory"):
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = "serial write failed during trajectory execution"
                    self.get_logger().error(result.error_string)
                    goal_handle.abort()
                    return result

                command_count += 1
                final_commanded_deg_by_joint = dict(next_commanded_deg_by_joint)
                self._commanded_deg_by_joint.update(next_commanded_deg_by_joint)
                self._command_velocity_dps_by_joint.update(next_velocity_dps_by_joint)
                for mapping, deg in zip(JOINT_MAPPINGS, commanded_deg_sextet):
                    self._last_sent_deg_by_joint[mapping.moveit_name] = deg
                self._publish_trusted_open_loop_commanded_joint_state(
                    next_commanded_deg_by_joint)

                max_tracking_error_deg = max(
                    abs(desired_deg_by_joint[mapping.moveit_name]
                        - next_commanded_deg_by_joint[mapping.moveit_name])
                    for mapping in JOINT_MAPPINGS
                )
                if max_tracking_error_deg > self.trajectory_tracking_error_abort_deg:
                    if tracking_error_abort_enabled:
                        if tracking_error_started_sec is None:
                            tracking_error_started_sec = now_wall_sec
                        elif (
                            now_wall_sec - tracking_error_started_sec
                            > self.trajectory_tracking_error_abort_sec
                        ):
                            result.error_code = (
                                FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                            )
                            result.error_string = (
                                f"trajectory tracking error {max_tracking_error_deg:.2f}deg "
                                f"exceeded {self.trajectory_tracking_error_abort_deg:.2f}deg "
                                f"for more than "
                                f"{self.trajectory_tracking_error_abort_sec:.2f}s"
                            )
                            self.get_logger().error(result.error_string)
                            goal_handle.abort()
                            return result
                    else:
                        tracking_error_started_sec = None
                        self.get_logger().warn(
                            "trusted_open_loop tracking lag "
                            f"{max_tracking_error_deg:.2f}deg exceeds "
                            f"{self.trajectory_tracking_error_abort_deg:.2f}deg; "
                            "continuing safe ramp-limited execution instead of "
                            "jumping or aborting.",
                            throttle_duration_sec=1.0,
                        )
                else:
                    tracking_error_started_sec = None

                feedback = self._build_follow_feedback(
                    joint_names,
                    desired_deg_by_joint,
                    next_commanded_deg_by_joint,
                )
                goal_handle.publish_feedback(feedback)

                final_error_deg = max(
                    abs(points[-1].deg_by_joint[mapping.moveit_name]
                        - next_commanded_deg_by_joint[mapping.moveit_name])
                    for mapping in JOINT_MAPPINGS
                )
                if (
                    elapsed_sec >= trajectory_duration_sec
                    and final_error_deg <= self.goal_tolerance_deg
                ):
                    break

                next_tick_sec += period_sec
                sleep_sec = max(0.0, next_tick_sec - time.monotonic())
                time.sleep(sleep_sec)

            total_elapsed_sec = max(time.monotonic() - start_wall_sec, 1e-9)
            final_error_deg = max(
                abs(points[-1].deg_by_joint[mapping.moveit_name]
                    - final_commanded_deg_by_joint[mapping.moveit_name])
                for mapping in JOINT_MAPPINGS
            )
            final_summary = ", ".join(
                f"{mapping.moveit_name}={final_commanded_deg_by_joint[mapping.moveit_name]:.3f}deg"
                for mapping in JOINT_MAPPINGS
            )
            self.get_logger().info(
                f"FollowJointTrajectory command loop summary: "
                f"commands={command_count}, elapsed={total_elapsed_sec:.3f}s, "
                f"effective_rate={command_count / total_elapsed_sec:.1f}Hz, "
                f"configured_rate={self.command_rate_hz:.1f}Hz, "
                f"final_error={final_error_deg:.3f}deg <= "
                f"goal_tolerance={self.goal_tolerance_deg:.3f}deg; "
                f"final_command=({final_summary})"
            )
            if (
                self._real_hardware_motion_requested()
                and self.require_post_motion_hardware_snapshot
                and self.joint_state_source_mode != "trusted_open_loop"
            ):
                post_snapshot, post_message = self.request_hardware_joint_snapshot(
                    use_cached=False,
                    reason="post-motion verification",
                )
                if post_snapshot is None:
                    result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    result.error_string = (
                        f"post-motion hardware snapshot unavailable: {post_message}"
                    )
                    self.get_logger().error(result.error_string)
                    goal_handle.abort()
                    return result

                physical_error_by_joint = {
                    mapping.moveit_name: (
                        post_snapshot.deg_by_joint[mapping.moveit_name]
                        - points[-1].deg_by_joint[mapping.moveit_name]
                    )
                    for mapping in JOINT_MAPPINGS
                }
                max_physical_error_deg = max(
                    abs(error) for error in physical_error_by_joint.values()
                )
                error_summary = ", ".join(
                    f"{joint}={error:+.3f}deg"
                    for joint, error in physical_error_by_joint.items()
                )
                self.get_logger().warn(
                    f"Post-motion hardware snapshot final physical error: "
                    f"max={max_physical_error_deg:.3f}deg; {error_summary}"
                )

            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = (
                "Trajectory accepted, sampled over time, and commands produced on schedule. "
                "Final physical pose is not verified unless /joint_states is real "
                "motor/encoder feedback."
            )
            if trusted_open_loop_real_motion and self.trusted_open_loop_hold_after_action:
                self._trusted_open_loop_hold_active = True
                self._last_trusted_open_loop_hold_stream_sec = None
                self.get_logger().warn(
                    "trusted_open_loop: post-action hold streaming is active; "
                    "the bridge will keep sending the final j6 target at "
                    f"{self.trusted_open_loop_hold_rate_hz:.1f}Hz until a new "
                    "trajectory, /disarm, or fault.")
            self.get_logger().warn(
                "FollowJointTrajectory succeeded by command-generation criteria only; "
                "final physical pose is not verified unless /joint_states is real "
                "motor/encoder feedback."
            )
            goal_handle.succeed()
            return result
        finally:
            self._active_action_goal_handle = None
            self._action_execution_active = False

    def _on_joint_trajectory(self, msg: JointTrajectory, topic: str) -> None:
        if not msg.joint_names or not msg.points:
            return

        trajectory_points: list[tuple[float, dict[str, float]]] = []
        last_time_sec = 0.0

        for point in msg.points:
            if len(point.positions) != len(msg.joint_names):
                self.get_logger().warn(
                    f"Ignoring trajectory from {topic}: point positions do not match joint_names",
                    throttle_duration_sec=2.0)
                return

            positions_by_name = dict(zip(msg.joint_names, point.positions))
            deg_by_joint: dict[str, float] = {}
            for mapping in JOINT_MAPPINGS:
                radians_value = positions_by_name.get(mapping.moveit_name)
                if radians_value is None or not math.isfinite(radians_value):
                    self.get_logger().warn(
                        f"Ignoring trajectory from {topic}: missing/invalid {mapping.moveit_name}",
                        throttle_duration_sec=2.0)
                    return

                physical_deg = mapping.moveit_rad_to_physical_deg(radians_value)
                if mapping.moveit_name == "Joint4":
                    physical_deg += self.joint4_offset_deg
                if physical_deg < mapping.min_deg or physical_deg > mapping.max_deg:
                    self.get_logger().warn(
                        f"{mapping.moveit_name} trajectory target {physical_deg:.2f}° outside "
                        f"[{mapping.min_deg:.1f}, {mapping.max_deg:.1f}]; clamping",
                        throttle_duration_sec=2.0)
                    physical_deg = min(max(physical_deg, mapping.min_deg), mapping.max_deg)

                deg_by_joint[mapping.moveit_name] = physical_deg

            point_time_sec = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1e9
            if point_time_sec < last_time_sec:
                point_time_sec = last_time_sec
            last_time_sec = point_time_sec
            trajectory_points.append((point_time_sec, deg_by_joint))

        now_sec = self.get_clock().now().nanoseconds / 1e9
        header_start_sec = self._stamp_to_sec(msg.header.stamp)
        start_sec = now_sec
        header_skew_sec = header_start_sec - now_sec if header_start_sec > 0.0 else 0.0
        if header_start_sec > 0.0 and abs(header_skew_sec) > 0.05:
            self.get_logger().info(
                f"[ACTIVE] trajectory header skew={header_skew_sec:+.3f}s from {topic}; "
                "using receipt time to avoid skipping ahead",
                throttle_duration_sec=1.0,
            )

        self._active_trajectory_points = trajectory_points
        self._active_trajectory_start_sec = start_sec
        self._active_trajectory_end_sec = start_sec + trajectory_points[-1][0] + self.trajectory_hold_sec
        self._timeout_warned = False
        self._joint_state_timeout_disarmed = False

        self.get_logger().info(
            f"[ACTIVE] trajectory received from {topic}: {len(trajectory_points)} points, "
            f"duration={trajectory_points[-1][0]:.3f}s "
            f"(start=receipt_time, header_skew={header_skew_sec:+.3f}s)",
            throttle_duration_sec=1.0)

    def _sample_active_trajectory(self, now_sec: float) -> Optional[dict[str, float]]:
        if (not self._active_trajectory_points or
                self._active_trajectory_start_sec is None or
                self._active_trajectory_end_sec is None):
            return None

        if now_sec > self._active_trajectory_end_sec:
            self._active_trajectory_points.clear()
            self._active_trajectory_start_sec = None
            self._active_trajectory_end_sec = None
            return None

        elapsed_sec = max(0.0, now_sec - self._active_trajectory_start_sec)

        first_time_sec, first_deg_by_joint = self._active_trajectory_points[0]
        if elapsed_sec <= first_time_sec:
            return dict(first_deg_by_joint)

        for index in range(1, len(self._active_trajectory_points)):
            prev_time_sec, prev_deg_by_joint = self._active_trajectory_points[index - 1]
            next_time_sec, next_deg_by_joint = self._active_trajectory_points[index]
            if elapsed_sec > next_time_sec:
                continue

            if next_time_sec <= prev_time_sec:
                return dict(next_deg_by_joint)

            alpha = (elapsed_sec - prev_time_sec) / (next_time_sec - prev_time_sec)
            return {
                mapping.moveit_name: prev_deg_by_joint[mapping.moveit_name] +
                alpha * (next_deg_by_joint[mapping.moveit_name] - prev_deg_by_joint[mapping.moveit_name])
                for mapping in JOINT_MAPPINGS
            }

        return dict(self._active_trajectory_points[-1][1])

    # ── REQUIRES_REARM tick handler ───────────────────────────────────────

    def _handle_requires_rearm_tick(self, now_sec: float) -> None:
        """Executed on every timer tick while in REQUIRES_REARM.

        Responsibilities:
          1. Accept an /rearm request if fresh joint_states are available.
          2. Track physical-arm-settled time for optional auto-rearm.
          3. Log periodic status so the operator knows what's blocking.
        """
        joint_age_sec = (
            self.get_clock().now() - self._last_joint_state_time).nanoseconds / 1e9
        have_fresh_positions = (
            bool(self._latest_deg_by_joint)
            and joint_age_sec <= self.joint_state_timeout_sec)

        # ── Option A: operator issued /rearm ──
        if self._rearm_requested:
            if not have_fresh_positions:
                self.get_logger().warn(
                    "[REQUIRES_REARM] /rearm requested but joint_states stale; "
                    "waiting for fresh data before activating",
                    throttle_duration_sec=2.0)
                return
            self._enter_active(
                f"explicit /rearm after {now_sec - (self._requires_rearm_entered_sec or now_sec):.1f}s "
                f"in REQUIRES_REARM")
            return

        # ── Option B: settle-based auto-rearm (opt-in) ──
        if self.auto_rearm_settle_threshold_deg > 0.0 and have_fresh_positions:
            max_delta_deg = 0.0
            for name, pos in self._latest_deg_by_joint.items():
                prev = self._settle_last_positions_deg.get(name)
                if prev is not None:
                    max_delta_deg = max(max_delta_deg, abs(pos - prev))
            self._settle_last_positions_deg = dict(self._latest_deg_by_joint)

            if max_delta_deg <= self.auto_rearm_settle_threshold_deg:
                if self._settle_start_sec is None:
                    self._settle_start_sec = now_sec
                dwell = now_sec - self._settle_start_sec
                if dwell >= self.auto_rearm_settle_dwell_sec:
                    self._enter_active(
                        f"auto-rearm: arm settled ≤{self.auto_rearm_settle_threshold_deg:.2f}° "
                        f"for {dwell:.1f}s")
                    return
                self.get_logger().info(
                    f"[REQUIRES_REARM] arm settling: "
                    f"Δmax={max_delta_deg:.2f}°, dwell={dwell:.1f}/"
                    f"{self.auto_rearm_settle_dwell_sec:.1f}s",
                    throttle_duration_sec=1.0)
            else:
                # Motion detected → restart the dwell timer.
                self._settle_start_sec = None
                self.get_logger().info(
                    f"[REQUIRES_REARM] motion detected (Δmax={max_delta_deg:.2f}°); "
                    f"auto-rearm dwell reset",
                    throttle_duration_sec=1.0)

        # ── Heartbeat for the operator ──
        if not have_fresh_positions:
            self.get_logger().warn(
                f"[REQUIRES_REARM epoch={self._command_epoch}] "
                f"waiting for fresh joint_states (age={joint_age_sec:.1f}s); "
                f"call {self.get_name()}/rearm once ready",
                throttle_duration_sec=5.0)
        else:
            self.get_logger().info(
                f"[REQUIRES_REARM epoch={self._command_epoch}] "
                f"joint_states OK — call {self.get_name()}/rearm to resume motion",
                throttle_duration_sec=5.0)

    # ── Main timer callback ───────────────────────────────────────────────

    def _publish_latest_targets(self) -> None:
        self._drain_serial_input()

        now_sec = self.get_clock().now().nanoseconds / 1e9
        dt_sec = max(now_sec - self._last_publish_time_sec, 1e-3)
        self._last_publish_time_sec = now_sec

        # ── STARTUP ───────────────────────────────────────────────────────
        if self._state == BridgeState.STARTUP:
            time_ready   = now_sec >= self._startup_ready_time
            serial_ready = (self.startup_min_serial_lines == 0 or
                            self._startup_serial_lines >= self.startup_min_serial_lines)
            if time_ready and serial_ready:
                # Route through REQUIRES_REARM (default) so boot never auto-commands
                # motion — even if MoveIt happens to publish a stale joint_state
                # during our startup window.
                if self.require_explicit_rearm:
                    self._enter_requires_rearm(
                        f"startup complete: delay elapsed, "
                        f"{self._startup_serial_lines} serial lines received")
                else:
                    self._enter_active(
                        f"startup complete (require_explicit_rearm=False): "
                        f"{self._startup_serial_lines} serial lines received")
            else:
                remaining = max(0.0, self._startup_ready_time - now_sec)
                if not serial_ready:
                    self.get_logger().info(
                        f"[STARTUP] waiting — serial lines: "
                        f"{self._startup_serial_lines}/{self.startup_min_serial_lines}"
                        f", time: {remaining:.1f}s remaining"
                        f" (ESP32 silent — is main bus power on?)",
                        throttle_duration_sec=3.0)
                else:
                    self.get_logger().info(
                        f"[STARTUP] waiting for hardware ready... {remaining:.1f}s",
                        throttle_duration_sec=2.0)
            return

        # ── FAULT ─────────────────────────────────────────────────────────
        if self._state == BridgeState.FAULT:
            self.get_logger().warn(
                f"[FAULT epoch={self._command_epoch}] Commands suppressed — "
                f"waiting for ESP32 'Failsafe cleared'",
                throttle_duration_sec=5.0)
            return

        # ── RECOVERY ──────────────────────────────────────────────────────
        if self._state == BridgeState.RECOVERY:
            elapsed = now_sec - (self._fault_cleared_time or now_sec)
            if elapsed >= self.fault_recovery_delay_sec:
                # After a fault, ALWAYS route through REQUIRES_REARM (even if
                # require_explicit_rearm=False, the command epoch was invalidated
                # so we need a fresh position sync and explicit operator consent
                # to avoid replaying the pre-fault target).
                if self.require_explicit_rearm:
                    self._enter_requires_rearm(
                        "fault recovery hold-off elapsed; "
                        "operator must /rearm to resume motion")
                else:
                    # Non-strict mode: still gate through REQUIRES_REARM briefly
                    # to force a fresh sync, then auto-accept.
                    self._enter_requires_rearm(
                        "fault recovery hold-off elapsed (auto-rearm mode)")
                    self._rearm_requested = True
            else:
                remaining = self.fault_recovery_delay_sec - elapsed
                self.get_logger().info(
                    f"[RECOVERY] advancing to REQUIRES_REARM in {remaining:.1f}s",
                    throttle_duration_sec=1.0)
            return

        # ── REQUIRES_REARM ────────────────────────────────────────────────
        if self._state == BridgeState.REQUIRES_REARM:
            self._maybe_publish_hardware_snapshot_for_planning(now_sec)
            self._handle_requires_rearm_tick(now_sec)
            return

        # ── ACTIVE ────────────────────────────────────────────────────────
        self._maybe_publish_hardware_snapshot_for_planning(now_sec)
        if self._action_execution_active:
            # FollowJointTrajectory owns hardware command streaming while it is
            # active. Do not let the legacy topic/joint_states bridge loop send
            # hold-current sextets that would overwrite the action trajectory.
            return
        joint_age_sec = (self.get_clock().now() - self._last_joint_state_time).nanoseconds / 1e9
        controller_age_sec = (
            self.get_clock().now() - self._last_controller_state_time).nanoseconds / 1e9
        trajectory_deg_by_joint = (
            self._sample_active_trajectory(now_sec)
            if self.prefer_trajectory_topic
            else None
        )
        have_active_trajectory = trajectory_deg_by_joint is not None
        have_fresh_controller_targets = (
            self.prefer_controller_state
            and bool(self._latest_controller_desired_deg_by_joint)
            and controller_age_sec <= self.controller_state_timeout_sec
        )
        have_fresh_joint_targets = (
            bool(self._latest_deg_by_joint)
            and joint_age_sec <= self.joint_state_timeout_sec
        )
        joint_states_may_command_hardware = not self._real_hardware_motion_requested()

        if have_active_trajectory:
            target_deg_by_joint = trajectory_deg_by_joint
            target_source = "joint_trajectory"
            use_bridge_profile = True
        elif have_fresh_controller_targets:
            target_deg_by_joint = self._latest_controller_desired_deg_by_joint
            target_source = "controller_state"
            use_bridge_profile = True
        elif have_fresh_joint_targets and joint_states_may_command_hardware:
            target_deg_by_joint = self._latest_deg_by_joint
            target_source = "joint_states"
            use_bridge_profile = True
        elif (
            self._real_hardware_motion_requested()
            and self.joint_state_source_mode == "trusted_open_loop"
            and self.trusted_open_loop_hold_after_action
            and self._trusted_open_loop_hold_active
            and all(
                mapping.moveit_name in self._commanded_deg_by_joint
                for mapping in JOINT_MAPPINGS
            )
        ):
            target_deg_by_joint = {
                mapping.moveit_name: self._commanded_deg_by_joint[mapping.moveit_name]
                for mapping in JOINT_MAPPINGS
            }
            target_source = "trusted_open_loop_hold"
            use_bridge_profile = False
        else:
            if not self._timeout_warned:
                if self.prefer_trajectory_topic and self._active_trajectory_points:
                    source_desc = (
                        "joint_trajectory expired and "
                        f"controller_state stale for {controller_age_sec:.2f}s "
                        f"and joint_states stale for {joint_age_sec:.2f}s"
                    )
                elif self.prefer_controller_state and self._latest_controller_desired_deg_by_joint:
                    source_desc = (
                        f"controller_state stale for {controller_age_sec:.2f}s "
                        f"and joint_states stale for {joint_age_sec:.2f}s"
                    )
                else:
                    if have_fresh_joint_targets and not joint_states_may_command_hardware:
                        source_desc = (
                            "joint_states are feedback-only in real hardware mode; "
                            "holding last hardware command"
                        )
                    else:
                        source_desc = f"no fresh joint_states for {joint_age_sec:.2f}s"
                action = "sending idle" if self.idle_on_stale else "holding last targets"
                self.get_logger().warn(
                    f"[ACTIVE] {source_desc}; {action}")
                self._timeout_warned = True
            if self.idle_on_stale and not self._joint_state_timeout_disarmed:
                self._send_idle_to_all_joints()
                self._joint_state_timeout_disarmed = True
            return

        if self._active_command_source != target_source:
            if target_source == "joint_trajectory":
                source_age_desc = (
                    f"t={max(0.0, now_sec - (self._active_trajectory_start_sec or now_sec)):.3f}s"
                )
            else:
                source_age = controller_age_sec if target_source == "controller_state" else joint_age_sec
                source_age_desc = f"age={source_age:.3f}s"
            self.get_logger().info(
                f"[ACTIVE] command source -> {target_source} ({source_age_desc})",
                throttle_duration_sec=0.5)
            self._active_command_source = target_source

        # The previous smooth workspace behavior came from the bridge's own
        # velocity/acceleration limiter.  Keep using that limiter for every
        # source, including sampled JointTrajectory targets, so a sparse
        # one-point IK goal does not become an instant pose snap.
        commanded_deg_sextet: list[float] = []
        next_commanded_deg_by_joint: dict[str, float] = {}
        next_velocity_dps_by_joint: dict[str, float] = {}
        any_change = False
        force_hold_stream = False
        if target_source == "trusted_open_loop_hold":
            hold_period_sec = 1.0 / self.trusted_open_loop_hold_rate_hz
            force_hold_stream = (
                self._last_trusted_open_loop_hold_stream_sec is None
                or now_sec - self._last_trusted_open_loop_hold_stream_sec >= hold_period_sec
            )

        for mapping in JOINT_MAPPINGS:
            target_deg = target_deg_by_joint.get(mapping.moveit_name)
            if target_deg is None:
                return

            profile_streaming_required = False
            if use_bridge_profile:
                profile_max_vel_dps: Optional[float] = self._joint_max_velocity_dps(mapping)
                profile_max_accel_dps2: Optional[float] = self._joint_max_acceleration_dps2(mapping)
                if target_source == "controller_state":
                    source_vel_dps = abs(
                        self._latest_controller_desired_vel_dps_by_joint.get(
                            mapping.moveit_name, 0.0)
                    )
                    source_accel_dps2 = abs(
                        self._latest_controller_desired_accel_dps2_by_joint.get(
                            mapping.moveit_name, 0.0)
                    )
                    if source_vel_dps > 1e-3:
                        profile_max_vel_dps = min(
                            profile_max_vel_dps, source_vel_dps)
                    if source_accel_dps2 > 1e-3:
                        profile_max_accel_dps2 = min(
                            profile_max_accel_dps2, source_accel_dps2)

                current_deg = self._commanded_deg_by_joint.get(
                    mapping.moveit_name,
                    self._latest_deg_by_joint.get(mapping.moveit_name, target_deg),
                )
                current_vel = self._command_velocity_dps_by_joint.get(mapping.moveit_name, 0.0)
                next_deg, next_vel = self._step_motion_toward_target(
                    current_deg,
                    current_vel,
                    target_deg,
                    dt_sec,
                    max_vel_dps=profile_max_vel_dps,
                    max_accel_dps2=profile_max_accel_dps2,
                )
                profile_streaming_required = (
                    abs(target_deg - current_deg) > self.min_delta_deg * 0.25
                    or abs(current_vel) > 1e-3
                    or abs(next_vel) > 1e-3
                )
            else:
                next_deg = target_deg
                next_vel = 0.0

            commanded_deg_sextet.append(next_deg)
            next_commanded_deg_by_joint[mapping.moveit_name] = next_deg
            next_velocity_dps_by_joint[mapping.moveit_name] = next_vel

            last_sent = self._last_sent_deg_by_joint.get(mapping.moveit_name)
            if (
                last_sent is None
                or force_hold_stream
                or (use_bridge_profile and profile_streaming_required)
                or abs(next_deg - last_sent) >= self.min_delta_deg
            ):
                any_change = True

        if not any_change:
            self._commanded_deg_by_joint.update(next_commanded_deg_by_joint)
            self._command_velocity_dps_by_joint.update(next_velocity_dps_by_joint)
            return

        if not self._dispatch_joint_sextet(commanded_deg_sextet, target_source):
            self._consecutive_serial_failures += 1
            if self._consecutive_serial_failures >= self.serial_fault_threshold:
                self._enter_fault(
                    f"serial write failed {self.serial_fault_threshold}x consecutively")
            return

        self._consecutive_serial_failures = 0
        if target_source == "trusted_open_loop_hold":
            self._last_trusted_open_loop_hold_stream_sec = now_sec
            self._publish_trusted_open_loop_commanded_joint_state(
                next_commanded_deg_by_joint)
        for mapping, deg in zip(JOINT_MAPPINGS, commanded_deg_sextet):
            self._commanded_deg_by_joint[mapping.moveit_name] = deg
            self._command_velocity_dps_by_joint[mapping.moveit_name] = (
                next_velocity_dps_by_joint[mapping.moveit_name]
            )
            self._last_sent_deg_by_joint[mapping.moveit_name] = deg

    # ── Motion planning ───────────────────────────────────────────────────

    def _step_motion_toward_target(
        self,
        current_deg: float,
        current_vel_dps: float,
        target_deg: float,
        dt_sec: float,
        *,
        max_vel_dps: Optional[float] = None,
        max_accel_dps2: Optional[float] = None,
    ) -> tuple[float, float]:
        error_deg = target_deg - current_deg
        if abs(error_deg) <= 1e-4 and abs(current_vel_dps) <= 1e-4:
            return target_deg, 0.0

        accel     = (
            max_accel_dps2
            if max_accel_dps2 is not None and max_accel_dps2 > 1e-6
            else self.max_command_acceleration_dps2
        )
        max_vel   = (
            max_vel_dps
            if max_vel_dps is not None and max_vel_dps > 1e-6
            else self.max_command_velocity_dps
        )
        direction = 1.0 if error_deg >= 0.0 else -1.0

        stopping_speed   = math.sqrt(max(0.0, 2.0 * accel * abs(error_deg)))
        desired_vel_dps  = direction * min(max_vel, stopping_speed)
        max_vel_change   = accel * dt_sec

        if desired_vel_dps > current_vel_dps:
            next_vel_dps = min(desired_vel_dps, current_vel_dps + max_vel_change)
        else:
            next_vel_dps = max(desired_vel_dps, current_vel_dps - max_vel_change)

        next_deg  = current_deg + next_vel_dps * dt_sec
        remaining = target_deg - next_deg

        if error_deg == 0.0 or error_deg * remaining <= 0.0:
            return target_deg, 0.0
        if abs(remaining) < self.min_delta_deg * 0.5 and abs(next_vel_dps) < accel * dt_sec:
            return target_deg, 0.0

        return next_deg, next_vel_dps

    # ── Serial writes ─────────────────────────────────────────────────────

    def _send_joint_sextet(self, command_deg_sextet: list[float]) -> bool:
        with self._serial_io_lock:
            if self._serial is None or not self._serial.is_open:
                self.get_logger().warn(
                    "Serial port not open; attempting reconnect",
                    throttle_duration_sec=5.0)
                if not self._reopen_serial_port():
                    return False

            try:
                line = "j6 " + " ".join(f"{d:.3f}" for d in command_deg_sextet) + "\n"
                self._serial.write(line.encode("ascii"))  # type: ignore[union-attr]
                self._serial.flush()                       # type: ignore[union-attr]
            except SerialException as exc:
                self.get_logger().error(f"Serial write failed: {exc}")
                self._reopen_serial_port()
                return False

        summary = ", ".join(
            f"{m.moveit_name}={d:.2f}"
            for m, d in zip(JOINT_MAPPINGS, command_deg_sextet))
        self.get_logger().info(
            f"[{self._state.value}] sextet: {summary}",
            throttle_duration_sec=0.5)
        return True

    def _send_idle_to_all_joints(self) -> None:
        if not self._idle_serial_enabled():
            self.get_logger().warn(
                f"[{'DRY-RUN' if self.dry_run else 'MOTION DISABLED'}] "
                "serial idle suppressed: jx6",
                throttle_duration_sec=1.0,
            )
            self._clear_motion_state()
            return
        with self._serial_io_lock:
            if self._serial is None or not self._serial.is_open:
                return
            try:
                self._serial.write(b"jx6\n")
                self._serial.flush()
            except SerialException as exc:
                self.get_logger().error(f"Failed to send jx6 idle: {exc}")
                self._reopen_serial_port()
                return
        self._clear_motion_state()

    # ── Cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self) -> bool:
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        return super().destroy_node()


def main(argv: list[str] | None = None) -> None:
    if rclpy is None:
        raise RuntimeError(f"ROS 2 imports unavailable: {_ROS_IMPORT_ERROR}")
    rclpy.init(args=argv)
    node = MoveItArmBridge6DofNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
