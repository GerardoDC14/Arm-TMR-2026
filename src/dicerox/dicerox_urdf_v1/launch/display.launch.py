import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('dicerox_urdf_v1')
    default_model = os.path.join(package_dir, 'urdf', 'Dicerox_URDFv1.xacro')
    default_rviz = os.path.join(package_dir, 'rviz', 'default.rviz')

    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rvizconfig')
    use_gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    xacro_args = LaunchConfiguration('xacro_args')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            model,
            ' ',
            xacro_args,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Absolute path to the robot xacro/URDF file.',
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=default_rviz,
            description='Absolute path to the RViz config file.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='True',
            description='Start joint_state_publisher_gui when true.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Start RViz when true.',
        ),
        DeclareLaunchArgument(
            'xacro_args',
            default_value='',
            description='Extra arguments passed to xacro.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            condition=IfCondition(use_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            condition=UnlessCondition(use_gui),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
