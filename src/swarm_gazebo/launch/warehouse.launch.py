import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    package_share = get_package_share_directory('swarm_gazebo')
    default_world = os.path.join(package_share, 'worlds', 'warehouse_world.sdf')


    amr_share_parent = os.path.dirname(get_package_share_directory('amr_description'))
    resource_path = os.pathsep.join(
        [p for p in [os.environ.get('GZ_SIM_RESOURCE_PATH', ''), amr_share_parent] if p]
    )
    set_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo SDF world file',
    )

    gazebo_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    )

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'gz_args': [
                TextSubstitution(text='-r -v 4 '),
                LaunchConfiguration('world'),
            ]
        }.items(),
    )

    return LaunchDescription([set_resource_path, world_arg, start_gazebo])
