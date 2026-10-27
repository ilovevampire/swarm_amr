import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_params = os.path.join(
        get_package_share_directory('amr_control'), 'config', 'ekf.yaml'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo clock (set false on the real robot)',
    )

    # The node name must match the top-level key in ekf.yaml (ekf_filter_node),
    # otherwise the parameters are silently ignored.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        # publishes /odometry/filtered and the odom -> base_footprint transform
    )

    return LaunchDescription([declare_use_sim_time, ekf])