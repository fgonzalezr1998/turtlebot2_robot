import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_dir = get_package_share_directory('tb2_bringup')

    ld = LaunchDescription()

    kobuki_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kobuki_node'),
                'launch',
                'kobuki_node-launch.py'
            ])
        ])
    )

    ld.add_action(kobuki_launch)

    kobuki_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tb2_bringup'),
                'launch',
                'robot_description.launch.py'
            ])
        ])
    )

    ld.add_action(kobuki_description)

    tf_base2lidar_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        remappings=[
            ('/tf', 'tf'), ('/tf_static', 'tf_static')
        ],
        arguments=[
            '0.0', '0.0', '0.4',
            '3.14', '0.0','3.14',
            'base_footprint', 'laser'
        ],
    )

    ld.add_action(tf_base2lidar_cmd)

    '''
    laser_filter_cmd = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        namespace=''namespace'',
        parameters=[filter_file],
        condition=IfCondition(PythonExpression([lidar]))
    )
    ld.add_action(laser_filter_cmd)

    tf_footprint2base_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace,
        output='screen',
        remappings=[
            ('/tf', 'tf'), ('/tf_static', 'tf_static')
        ],
        arguments=[
            '0.0', '0.0', '0.001',
            '0.0', '0.0', '0.0',
            '1.0', 'base_link', 'base_footprint'
        ],
    )

    tf_base2camera_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace,
        output='screen',
        remappings=[
            ('/tf', 'tf'), ('/tf_static', 'tf_static')
        ],
        arguments=[
            '0.1', '0.0', '0.2',
            '0.0', '0.0', '0.0',
            '1.0', 'base_footprint', 'camera_link'
        ],
    )

    ld.add_action(tf_footprint2base_cmd)
    ld.add_action(tf_base2camera_cmd)
    '''
    return ld