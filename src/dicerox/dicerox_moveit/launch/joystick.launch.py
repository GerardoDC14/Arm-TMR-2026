from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='jaguar_teleop',
            executable='joystick_servo',
            name='joystick_servo',
            output='screen',
            parameters=[{
                'robot_name': 'Dicerox Arm',
                'joint_names': joint_names,
                'planning_frame': 'base_link',
            }],
        ),
    ])
