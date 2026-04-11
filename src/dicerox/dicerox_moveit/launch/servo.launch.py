import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("Dicerox_robot_arm_URDF", package_name="dicerox_moveit")
        .to_moveit_configs()
    )

    servo_params_path = os.path.join(
        get_package_share_directory("dicerox_moveit"),
        "config",
        "servo_params.yaml",
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            {"use_sim_time": False},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            servo_params_path,
        ],
        output="screen",
    )

    start_servo = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/servo_node/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([servo_node, start_servo])
