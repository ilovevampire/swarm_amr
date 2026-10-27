import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# >>> MAP USED BY NAV2 (map_server + AMCL). Change this line to use another map. <<<
# The .yaml points to its .pgm image in the same folder, so both files must stay together.
MAP_YAML = '/home/vshal/swarm_ws/src/amr_navigation/maps/warehouse_v2.yaml'


def generate_launch_description():
    pkg = get_package_share_directory('amr_navigation')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    # Can still be overridden at launch time:  map:=/full/path/to/other.yaml
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=MAP_YAML,
        description='Full path to the map yaml (map_server + AMCL use it)',
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
        description='Nav2 parameter file',
    )

    # map_server + amcl + planner + controller + smoother + behaviors + bt_navigator ...
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',
            'slam': 'False',
        }.items(),
    )

    # Nav2 publishes Twist on /cmd_vel; diff_drive_controller needs TwistStamped
    stamper = Node(
        package='amr_navigation',
        executable='cmd_vel_stamper.py',
        name='cmd_vel_stamper',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([map_arg, params_arg, nav2, stamper])