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
                "robot_name":  "Dicerox Arm",
                "frame_id":    "base_link",
                "twist_topic": "/servo_node/delta_twist_cmds",

                # ── Output range ──────────────────────────────────────────
                # With servo_params command_in_type "unitless", keep these at 1.0
                # so values passed to servo are in [-1, 1].  Tune velocity in
                # servo_params.yaml  scale.linear / scale.rotational.
                "publish_rate_hz": 50.0,
                "linear_scale":    1.0,
                "angular_scale":   1.0,

                # ── Per-axis fine-tuning (on top of global scale) ─────────
                # Set < 1.0 to attenuate, > 1.0 to amplify a specific axis.
                "linear_x_scale":      1.0,
                "linear_y_scale":      1.0,
                "linear_z_scale":      0.8,   # Z range slightly reduced
                "angular_roll_scale":  1.0,
                "angular_pitch_scale": 1.0,
                "angular_yaw_scale":   0.8,

                # ── Axis remapping ────────────────────────────────────────
                # Space Explorer HID bytes 1-2 = lateral (left/right);
                # bytes 3-4 = fore/aft (forward/backward).
                # Robot X = forward → swap needed.
                "swap_xy_translation": True,
                "swap_xy_rotation":    False,
                # Sign inversion (after swap): flip any axis that moves backwards.
                "invert_x":     False,
                "invert_y":     True,
                "invert_z":     False,
                "invert_roll":  False,
                "invert_pitch": False,
                "invert_yaw":   False,

                # ── Noise / filtering ─────────────────────────────────────
                "deadband":                0.07,   # 7% floor — slightly wider to cut resting jitter
                "axis_relative_threshold": 0.40,
                "dominance_threshold":     1.5,
                "command_timeout":         0.20,

                # ── Smoothing ─────────────────────────────────────────────
                # EMA alpha: 1.0 = no smoothing, 0.65 = light filter.
                # Increase toward 1.0 if the arm feels sluggish.
                "smoothing_alpha": 0.65,

                # ── Expo curve ────────────────────────────────────────────
                # 0.0 = linear, 0.5 = blend (recommended), 1.0 = pure cubic.
                # Reduces sensitivity for small inputs without limiting top speed.
                "expo_factor": 0.5,

                # ── Singularity mode handling ─────────────────────────────
                # Start paused so the user can move the arm out of any
                # singular pose via RViz before enabling SpaceMouse.
                "start_paused":                  True,
                "auto_resume_after_singularity": True,
                "singularity_resume_delay_sec":  1.0,

                # ── Debug ─────────────────────────────────────────────────
                # Set True to log raw/mapped/twist values for axis diagnosis.
                "debug_input": False,
            }
        ],
    )

    return LaunchDescription([demo_launch, servo_launch, spacemouse_node])
