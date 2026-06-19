from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _include_launch(package_share: Path, launch_file: str, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / "launch" / launch_file)),
        **kwargs,
    )


def generate_launch_description():
    """MoveIt hardware-mode launch using the bridge action server.

    This launch intentionally does not start ros2_control_node or any
    joint_trajectory_controller spawner. The only owner of
    /dicerox_arm_controller/follow_joint_trajectory must be
    /moveit_arm_bridge_6dof.
    """

    moveit_share = Path(get_package_share_directory("dicerox_moveit"))

    dry_run = LaunchConfiguration("dry_run")
    enable_hardware_motion = LaunchConfiguration("enable_hardware_motion")
    rviz = LaunchConfiguration("rviz")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    command_rate_hz = LaunchConfiguration("command_rate_hz")
    action_name = LaunchConfiguration("follow_joint_trajectory_action_name")
    joint_state_source_mode = LaunchConfiguration("joint_state_source_mode")
    require_trusted_joint_state_for_hardware = LaunchConfiguration(
        "require_trusted_joint_state_for_hardware"
    )
    allow_fake_joint_states_for_hardware = LaunchConfiguration(
        "allow_fake_joint_states_for_hardware"
    )
    hardware_snapshot_timeout_sec = LaunchConfiguration("hardware_snapshot_timeout_sec")
    require_pre_motion_hardware_snapshot = LaunchConfiguration(
        "require_pre_motion_hardware_snapshot"
    )
    require_post_motion_hardware_snapshot = LaunchConfiguration(
        "require_post_motion_hardware_snapshot"
    )
    publish_hardware_snapshots_for_planning = LaunchConfiguration(
        "publish_hardware_snapshots_for_planning"
    )
    periodic_hardware_snapshot_refresh = LaunchConfiguration(
        "periodic_hardware_snapshot_refresh"
    )
    republish_cached_hardware_joint_states = LaunchConfiguration(
        "republish_cached_hardware_joint_states"
    )
    hardware_snapshot_publish_period_sec = LaunchConfiguration(
        "hardware_snapshot_publish_period_sec"
    )
    request_startup_hardware_snapshot = LaunchConfiguration(
        "request_startup_hardware_snapshot"
    )
    joint_limit_epsilon_deg = LaunchConfiguration("joint_limit_epsilon_deg")
    allow_dry_run_without_active_hardware = LaunchConfiguration(
        "allow_dry_run_without_active_hardware"
    )
    idle_on_action_cancel = LaunchConfiguration("idle_on_action_cancel")
    trusted_open_loop_hold_after_action = LaunchConfiguration(
        "trusted_open_loop_hold_after_action"
    )
    trusted_open_loop_hold_rate_hz = LaunchConfiguration(
        "trusted_open_loop_hold_rate_hz"
    )
    allowed_execution_duration_scaling = LaunchConfiguration(
        "allowed_execution_duration_scaling"
    )
    allowed_goal_duration_margin = LaunchConfiguration("allowed_goal_duration_margin")

    moveit_config = (
        MoveItConfigsBuilder("Dicerox_robot_arm_URDF", package_name="dicerox_moveit")
        .to_moveit_configs()
    )
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": moveit_config.move_group_capabilities["capabilities"],
        "disable_capabilities": moveit_config.move_group_capabilities[
            "disable_capabilities"
        ],
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "trajectory_execution": {
            "allowed_execution_duration_scaling": ParameterValue(
                allowed_execution_duration_scaling, value_type=float
            ),
            "allowed_goal_duration_margin": ParameterValue(
                allowed_goal_duration_margin, value_type=float
            ),
        },
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description=(
                    "Keep the bridge in command-generation-only mode. "
                    "No serial j6 motion commands are sent while true."
                ),
            ),
            DeclareLaunchArgument(
                "enable_hardware_motion",
                default_value="false",
                description=(
                    "Dangerous opt-in. Serial motion is allowed only when this is true, "
                    "dry_run is false, and the bridge is ACTIVE."
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start MoveIt RViz.",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyUSB0",
                description="Serial port used by moveit_arm_bridge_6dof.",
            ),
            DeclareLaunchArgument(
                "baud_rate",
                default_value="921600",
                description="Serial baud rate used by moveit_arm_bridge_6dof.",
            ),
            DeclareLaunchArgument(
                "command_rate_hz",
                default_value="50.0",
                description="Bridge command-generation rate.",
            ),
            DeclareLaunchArgument(
                "follow_joint_trajectory_action_name",
                default_value="/dicerox_arm_controller/follow_joint_trajectory",
                description="FollowJointTrajectory action endpoint served by the bridge.",
            ),
            DeclareLaunchArgument(
                "joint_state_source_mode",
                default_value="dry_run_fake",
                description=(
                    "Joint-state trust declaration: dry_run_fake, hardware_feedback, "
                    "or trusted_open_loop. trusted_open_loop skips qjs6 snapshots "
                    "during normal execution."
                ),
            ),
            DeclareLaunchArgument(
                "require_trusted_joint_state_for_hardware",
                default_value="true",
                description="Require trusted real joint feedback before real hardware motion.",
            ),
            DeclareLaunchArgument(
                "allow_fake_joint_states_for_hardware",
                default_value="false",
                description="Dangerous override. Keep false for real hardware.",
            ),
            DeclareLaunchArgument(
                "hardware_snapshot_timeout_sec",
                default_value="1.0",
                description="Timeout for qjs6/js6 hardware state snapshots.",
            ),
            DeclareLaunchArgument(
                "require_pre_motion_hardware_snapshot",
                default_value="true",
                description="Require qjs6 hardware snapshot before real trajectory execution.",
            ),
            DeclareLaunchArgument(
                "require_post_motion_hardware_snapshot",
                default_value="true",
                description="Require qjs6 hardware snapshot after real trajectory execution.",
            ),
            DeclareLaunchArgument(
                "publish_hardware_snapshots_for_planning",
                default_value="false",
                description=(
                    "Compatibility gate for periodic planning snapshots. Keep false "
                    "unless intentionally allowing periodic qjs6 refresh."
                ),
            ),
            DeclareLaunchArgument(
                "periodic_hardware_snapshot_refresh",
                default_value="false",
                description=(
                    "If true together with publish_hardware_snapshots_for_planning, "
                    "allow periodic qjs6 refresh. Default false avoids CAN/serial polling."
                ),
            ),
            DeclareLaunchArgument(
                "republish_cached_hardware_joint_states",
                default_value="true",
                description=(
                    "Republish the last valid js6 snapshot as /joint_states with fresh "
                    "ROS timestamps without querying the ESP32."
                ),
            ),
            DeclareLaunchArgument(
                "hardware_snapshot_publish_period_sec",
                default_value="0.1",
                description=(
                    "Minimum period for cached /joint_states republish, and for "
                    "explicitly enabled periodic qjs6 refresh."
                ),
            ),
            DeclareLaunchArgument(
                "request_startup_hardware_snapshot",
                default_value="false",
                description="Optionally request one qjs6 snapshot after startup/rearm gate.",
            ),
            DeclareLaunchArgument(
                "joint_limit_epsilon_deg",
                default_value="0.05",
                description="Small epsilon for clamping snapshot values at joint limits.",
            ),
            DeclareLaunchArgument(
                "allow_dry_run_without_active_hardware",
                default_value="true",
                description="Allow dry-run action goals before the bridge reaches ACTIVE.",
            ),
            DeclareLaunchArgument(
                "idle_on_action_cancel",
                default_value="false",
                description=(
                    "If false, a MoveIt action cancel/timeout holds the last streamed "
                    "target instead of sending jx6 idle. Use /disarm for intentional idle."
                ),
            ),
            DeclareLaunchArgument(
                "trusted_open_loop_hold_after_action",
                default_value="true",
                description=(
                    "In trusted_open_loop real motion, keep sending the final j6 "
                    "target after an action completes or is canceled without idle."
                ),
            ),
            DeclareLaunchArgument(
                "trusted_open_loop_hold_rate_hz",
                default_value="10.0",
                description="Rate for post-action trusted_open_loop hold j6 streaming.",
            ),
            DeclareLaunchArgument(
                "allowed_execution_duration_scaling",
                default_value="10.0",
                description=(
                    "Hardware-mode MoveIt timeout scaling. Higher values allow the "
                    "slow firmware/bridge ramp to finish without MoveIt canceling."
                ),
            ),
            DeclareLaunchArgument(
                "allowed_goal_duration_margin",
                default_value="10.0",
                description="Extra hardware-mode MoveIt execution timeout margin in seconds.",
            ),
            LogInfo(
                msg=(
                    "Dicerox hardware MoveIt launch: do not run demo.launch.py at "
                    "the same time, because demo.launch.py starts fake/mock "
                    "controllers that may also own "
                    "/dicerox_arm_controller/follow_joint_trajectory."
                )
            ),
            LogInfo(
                msg=[
                    "Bridge safety: dry_run=",
                    dry_run,
                    ", enable_hardware_motion=",
                    enable_hardware_motion,
                    ", joint_state_source_mode=",
                    joint_state_source_mode,
                    ", action=",
                    action_name,
                ]
            ),
            _include_launch(moveit_share, "static_virtual_joint_tfs.launch.py"),
            _include_launch(moveit_share, "rsp.launch.py"),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict(), move_group_configuration],
            ),
            _include_launch(
                moveit_share,
                "moveit_rviz.launch.py",
                condition=IfCondition(rviz),
            ),
            Node(
                package="bldc_can_tools",
                executable="moveit_arm_bridge_6dof",
                name="moveit_arm_bridge_6dof",
                output="screen",
                parameters=[
                    {
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                        "enable_hardware_motion": ParameterValue(
                            enable_hardware_motion, value_type=bool
                        ),
                        "follow_joint_trajectory_action_name": action_name,
                        "joint_state_source_mode": joint_state_source_mode,
                        "require_trusted_joint_state_for_hardware": ParameterValue(
                            require_trusted_joint_state_for_hardware, value_type=bool
                        ),
                        "allow_fake_joint_states_for_hardware": ParameterValue(
                            allow_fake_joint_states_for_hardware, value_type=bool
                        ),
                        "hardware_snapshot_timeout_sec": ParameterValue(
                            hardware_snapshot_timeout_sec, value_type=float
                        ),
                        "require_pre_motion_hardware_snapshot": ParameterValue(
                            require_pre_motion_hardware_snapshot, value_type=bool
                        ),
                        "require_post_motion_hardware_snapshot": ParameterValue(
                            require_post_motion_hardware_snapshot, value_type=bool
                        ),
                        "publish_hardware_snapshots_for_planning": ParameterValue(
                            publish_hardware_snapshots_for_planning, value_type=bool
                        ),
                        "periodic_hardware_snapshot_refresh": ParameterValue(
                            periodic_hardware_snapshot_refresh, value_type=bool
                        ),
                        "republish_cached_hardware_joint_states": ParameterValue(
                            republish_cached_hardware_joint_states, value_type=bool
                        ),
                        "hardware_snapshot_publish_period_sec": ParameterValue(
                            hardware_snapshot_publish_period_sec, value_type=float
                        ),
                        "request_startup_hardware_snapshot": ParameterValue(
                            request_startup_hardware_snapshot, value_type=bool
                        ),
                        "joint_limit_epsilon_deg": ParameterValue(
                            joint_limit_epsilon_deg, value_type=float
                        ),
                        "trusted_open_loop_initial_joint_positions_deg": [
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                        ],
                        "serial_port": serial_port,
                        "baud_rate": ParameterValue(baud_rate, value_type=int),
                        "command_rate_hz": ParameterValue(command_rate_hz, value_type=float),
                        "allow_dry_run_without_active_hardware": (
                            ParameterValue(
                                allow_dry_run_without_active_hardware, value_type=bool
                            )
                        ),
                        "idle_on_action_cancel": ParameterValue(
                            idle_on_action_cancel, value_type=bool
                        ),
                        "trusted_open_loop_hold_after_action": ParameterValue(
                            trusted_open_loop_hold_after_action, value_type=bool
                        ),
                        "trusted_open_loop_hold_rate_hz": ParameterValue(
                            trusted_open_loop_hold_rate_hz, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
