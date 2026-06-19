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
                "robot_name":  "Jaguar Arm",
                "frame_id":    "base_link",
                "twist_topic": "/servo_node/delta_twist_cmds",

                "publish_rate_hz": 50.0,
                "linear_scale":    1.0,
                "angular_scale":   1.0,

                "linear_x_scale":      1.0,
                "linear_y_scale":      1.0,
                "linear_z_scale":      0.8,
                "angular_roll_scale":  1.0,
                "angular_pitch_scale": 1.0,
                "angular_yaw_scale":   0.8,

                "swap_xy_translation": True,
                "swap_xy_rotation":    False,
                "invert_x":     False,
                "invert_y":     True,
                "invert_z":     False,
                "invert_roll":  False,
                "invert_pitch": False,
                "invert_yaw":   False,

                "deadband":                0.07,
                "axis_relative_threshold": 0.40,
                "dominance_threshold":     1.5,
                "command_timeout":         0.20,

                "smoothing_alpha": 0.65,

                # Expo curve for fine control near zero.
                # 0.0 = linear, 0.5 = recommended blend, 1.0 = pure cubic.
                # At 10% stick with 0.5: output ≈ 5% (vs 10% linear).
                "expo_factor": 0.5,

                # Start paused so the user can move the arm out of any
                # singular pose via RViz before enabling SpaceMouse.
                "start_paused":                  True,
                "auto_resume_after_singularity": True,
                "singularity_resume_delay_sec":  1.0,

                # Set True to log raw/mapped/twist values for axis diagnosis.
                "debug_input": False,
            }
        ],
    )

    return LaunchDescription([demo_launch, servo_launch, spacemouse_node])
