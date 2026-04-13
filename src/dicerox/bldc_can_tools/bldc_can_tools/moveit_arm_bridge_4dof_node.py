from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Optional


if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import HistoryPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from sensor_msgs.msg import JointState
except ImportError as exc:  # pragma: no cover - optional in direct usage
    rclpy = None
    Node = object  # type: ignore[assignment]
    QoSProfile = None  # type: ignore[assignment]
    ReliabilityPolicy = None  # type: ignore[assignment]
    HistoryPolicy = None  # type: ignore[assignment]
    DurabilityPolicy = None  # type: ignore[assignment]
    JointState = None  # type: ignore[assignment]
    _ROS_IMPORT_ERROR = exc
else:
    _ROS_IMPORT_ERROR = None


try:
    import serial
    from serial import Serial
    from serial import SerialException
except ImportError as exc:  # pragma: no cover - optional in direct usage
    serial = None  # type: ignore[assignment]
    Serial = object  # type: ignore[assignment]
    SerialException = Exception  # type: ignore[assignment]
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None


@dataclass(frozen=True)
class JointMapping:
    moveit_name: str
    min_deg: float
    max_deg: float

    def moveit_rad_to_physical_deg(self, radians_value: float) -> float:
        return -math.degrees(radians_value)


JOINT_MAPPINGS: tuple[JointMapping, ...] = (
    JointMapping("Joint1", -90.0, 90.0),
    JointMapping("Joint2", 0.0, 180.0),
    JointMapping("Joint3", -200.0, 0.0),
    JointMapping("Joint4", -90.0, 90.0),
)


class MoveItArmBridge4DofNode(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError(f"ROS 2 imports are unavailable: {_ROS_IMPORT_ERROR}")
        if serial is None:
            raise RuntimeError(f"pyserial is unavailable: {_SERIAL_IMPORT_ERROR}")

        super().__init__("moveit_arm_bridge_4dof")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("command_rate_hz", 15.0)
        self.declare_parameter("min_delta_deg", 0.25)
        self.declare_parameter("joint_state_timeout_sec", 3.0)
        self.declare_parameter("idle_on_stale", False)
        self.declare_parameter("max_command_velocity_dps", 90.0)
        self.declare_parameter("joint4_offset_deg", 0.0)
        self.declare_parameter("write_timeout_sec", 0.25)
        self.declare_parameter("startup_delay_sec", 3.0)
        self.declare_parameter("joint_state_topic", "/joint_states")

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.min_delta_deg = float(self.get_parameter("min_delta_deg").value)
        self.joint_state_timeout_sec = float(self.get_parameter("joint_state_timeout_sec").value)
        self.idle_on_stale = bool(self.get_parameter("idle_on_stale").value)
        self.max_command_velocity_dps = float(self.get_parameter("max_command_velocity_dps").value)
        self.joint4_offset_deg = float(self.get_parameter("joint4_offset_deg").value)
        self.write_timeout_sec = float(self.get_parameter("write_timeout_sec").value)
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)

        if self.command_rate_hz <= 0.0:
            raise ValueError("command_rate_hz must be > 0")
        if self.max_command_velocity_dps <= 0.0:
            raise ValueError("max_command_velocity_dps must be > 0")

        self._serial: Optional[Serial] = None
        self._latest_deg_by_joint: dict[str, float] = {}
        self._commanded_deg_by_joint: dict[str, float] = {}
        self._last_sent_deg_by_joint: dict[str, float] = {}
        self._last_joint_state_time = self.get_clock().now()
        self._startup_ready_time = self.get_clock().now().nanoseconds / 1e9 + self.startup_delay_sec
        self._last_publish_time_sec = self.get_clock().now().nanoseconds / 1e9
        self._timeout_warned = False
        self._joint_state_timeout_disarmed = False

        self._open_serial_port()

        joint_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            joint_state_qos,
        )
        self.create_timer(1.0 / self.command_rate_hz, self._publish_latest_targets)

        summary = ", ".join(
            f"{mapping.moveit_name}[{mapping.min_deg:.0f},{mapping.max_deg:.0f}]"
            for mapping in JOINT_MAPPINGS
        )
        self.get_logger().info(
            f"Bridging {self.joint_state_topic} to {self.serial_port} at {self.baud_rate} baud; "
            f"mappings: {summary}; startup_delay={self.startup_delay_sec:.1f}s; "
            f"idle_on_stale={self.idle_on_stale}; max_command_velocity_dps={self.max_command_velocity_dps:.1f}; "
            f"joint4_offset_deg={self.joint4_offset_deg:.2f}"
        )

    def _open_serial_port(self) -> None:
        try:
            self._serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.0,
                write_timeout=self.write_timeout_sec,
            )
        except SerialException as exc:
            raise RuntimeError(f"Failed to open serial port {self.serial_port}: {exc}") from exc

    def _on_joint_state(self, msg: JointState) -> None:
        positions_by_name = {
            name: position
            for name, position in zip(msg.name, msg.position)
        }

        for mapping in JOINT_MAPPINGS:
            radians_value = positions_by_name.get(mapping.moveit_name)
            if radians_value is None:
                continue

            physical_deg = mapping.moveit_rad_to_physical_deg(radians_value)
            if mapping.moveit_name == "Joint4":
                physical_deg += self.joint4_offset_deg
            if physical_deg < mapping.min_deg or physical_deg > mapping.max_deg:
                self.get_logger().warn(
                    f"{mapping.moveit_name} target {physical_deg:.2f} deg is outside physical limits "
                    f"[{mapping.min_deg:.1f}, {mapping.max_deg:.1f}]; clamping",
                    throttle_duration_sec=2.0,
                )
                physical_deg = min(max(physical_deg, mapping.min_deg), mapping.max_deg)

            self._latest_deg_by_joint[mapping.moveit_name] = physical_deg

        self._last_joint_state_time = self.get_clock().now()
        self._timeout_warned = False
        self._joint_state_timeout_disarmed = False

    def _publish_latest_targets(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        dt_sec = max(now_sec - self._last_publish_time_sec, 1e-3)
        self._last_publish_time_sec = now_sec
        if now_sec < self._startup_ready_time:
            return

        age_sec = (self.get_clock().now() - self._last_joint_state_time).nanoseconds / 1e9
        if age_sec > self.joint_state_timeout_sec:
            if not self._timeout_warned:
                action = "sending idle4" if self.idle_on_stale else "holding last targets"
                self.get_logger().warn(f"No fresh joint_states for {age_sec:.2f}s; {action}")
                self._timeout_warned = True
            if self.idle_on_stale and not self._joint_state_timeout_disarmed:
                self._send_idle_to_all_joints()
                self._joint_state_timeout_disarmed = True
            return

        desired_quartet: list[float] = []
        commanded_quartet: list[float] = []
        any_change = False
        for mapping in JOINT_MAPPINGS:
            target_deg = self._latest_deg_by_joint.get(mapping.moveit_name)
            if target_deg is None:
                continue

            desired_quartet.append(target_deg)
            current_deg = self._commanded_deg_by_joint.get(mapping.moveit_name, target_deg)
            max_step_deg = self.max_command_velocity_dps * dt_sec
            delta_deg = target_deg - current_deg
            if abs(delta_deg) > max_step_deg:
                current_deg += max_step_deg if delta_deg > 0.0 else -max_step_deg
            else:
                current_deg = target_deg

            commanded_quartet.append(current_deg)
            last_sent = self._last_sent_deg_by_joint.get(mapping.moveit_name)
            if last_sent is None or abs(current_deg - last_sent) >= self.min_delta_deg:
                any_change = True

        if len(desired_quartet) != len(JOINT_MAPPINGS):
            return
        if not any_change:
            return
        if not self._send_joint_quartet(commanded_quartet):
            return

        for mapping, target_deg in zip(JOINT_MAPPINGS, commanded_quartet):
            self._commanded_deg_by_joint[mapping.moveit_name] = target_deg
            self._last_sent_deg_by_joint[mapping.moveit_name] = target_deg

    def _send_joint_quartet(self, command_deg_quartet: list[float]) -> bool:
        if self._serial is None:
            raise RuntimeError("Serial port is not open")

        try:
            command_line = "j4 " + " ".join(f"{target_deg:.3f}" for target_deg in command_deg_quartet) + "\n"
            self._serial.write(command_line.encode("ascii"))
            self._serial.flush()
        except SerialException as exc:
            self.get_logger().error(f"Failed to write MoveIt quartet: {exc}")
            return False

        summary = ", ".join(
            f"{mapping.moveit_name}={target_deg:.2f}"
            for mapping, target_deg in zip(JOINT_MAPPINGS, command_deg_quartet)
        )
        self.get_logger().info(f"MoveIt quartet sent: {summary}", throttle_duration_sec=0.5)
        return True

    def _send_idle_to_all_joints(self) -> None:
        if self._serial is None:
            raise RuntimeError("Serial port is not open")

        try:
            self._serial.write(b"jx\n")
            self._serial.flush()
        except SerialException as exc:
            self.get_logger().error(f"Failed to send jx idle command: {exc}")
            return

        self._commanded_deg_by_joint.clear()
        self._last_sent_deg_by_joint.clear()

    def destroy_node(self) -> bool:
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        return super().destroy_node()


def main(argv: list[str] | None = None) -> None:
    if rclpy is None:
        raise RuntimeError(f"ROS 2 imports are unavailable: {_ROS_IMPORT_ERROR}")

    rclpy.init(args=argv)
    node = MoveItArmBridge4DofNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
