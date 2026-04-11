import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = os.path.dirname(__file__)

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, "demo.launch.py"))
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, "servo.launch.py"))
    )

    spacemouse_node = Node(
        package="jaguar_teleop",
        executable="spacemouse_servo",
        name="spacemouse_servo",
        output="screen",
        parameters=[
            {
                "robot_name":              "Jaguar Arm",
                "frame_id":               "base_link",
                "twist_topic":            "/servo_node/delta_twist_cmds",
                "publish_rate_hz":        50.0,
                "linear_scale":           0.20,
                "angular_scale":          0.35,
                "deadband":               0.05,
                "axis_relative_threshold": 0.40,
                "dominance_threshold":    1.5,
            }
        ],
    )

    return LaunchDescription([demo_launch, servo_launch, spacemouse_node])
