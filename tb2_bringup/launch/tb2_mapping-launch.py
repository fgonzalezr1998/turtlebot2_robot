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

    tb2_bringup_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          FindPackageShare('tb2_bringup'),
          'launch',
          'tb2_bringup-launch.py'
        ])
      ])
    )

    ld.add_action(tb2_bringup_launch)

    rplidar_a3_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([
        PathJoinSubstitution([
          FindPackageShare('rplidar_ros'),
          'launch',
          'rplidar_a3_launch.py'
        ])
      ]),
      launch_arguments={
        'serial_port': '/dev/ttyUSB0',
        'serial_baudrate': '256000',
        'frame_id': 'laser',
      }.items()
    )

    ld.add_action(rplidar_a3_launch)

    slam_toolbox_node = Node(
      parameters=[
        package_dir + '/config/mapping_config.yaml'
      ],
      package='slam_toolbox',
      executable='sync_slam_toolbox_node',
      name='slam_toolbox',
      output='screen'
    )

    ld.add_action(slam_toolbox_node)

    rviz2_node = Node(
      package='rviz2',
      executable='rviz2',
      namespace='',
      name='rviz2',
      arguments=['-d' + os.path.join(package_dir, 'config', 'mapping_rviz.rviz')]
    )

    ld.add_action(rviz2_node)

    return ld