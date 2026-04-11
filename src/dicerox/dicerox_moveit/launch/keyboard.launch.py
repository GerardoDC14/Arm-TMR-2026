from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

    return LaunchDescription([
        Node(
            package='jaguar_teleop',
            executable='keyboard_servo',
            name='keyboard_servo',
            output='screen',
            parameters=[{
                'robot_name': 'Dicerox Arm',
                'joint_names': joint_names,
                'planning_frame': 'base_link',
            }],
        ),
    ])
