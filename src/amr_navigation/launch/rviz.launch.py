from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Config files live in amr_navigation/rviz/ :
    #   ros2 launch amr_navigation rviz.launch.py                         (slam.rviz)
    #   ros2 launch amr_navigation rviz.launch.py rviz_config:=nav.rviz   (later, for Nav2)
    config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='slam.rviz',
        description='RViz config file name inside amr_navigation/rviz/',
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('amr_navigation'), 'rviz', LaunchConfiguration('rviz_config')
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([config_arg, rviz])