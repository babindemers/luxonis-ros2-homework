from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_bringup_path = get_package_share_path('robot_bringup')
    default_rviz_config_path = robot_bringup_path / 'rviz/simulation.rviz'

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robot_gazebo"), '/launch/world.launch.py'])
    )

    control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robot_control"), '/launch/control.launch.py'])
    )

    upload_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robot_description"), '/launch/upload.launch.py'])
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        rviz_arg,
        control_node,
        gazebo_node,
        upload_node,
        rviz_node
    ])
