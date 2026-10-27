from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    xacro_file = PathJoinSubstitution([
        FindPackageShare('amr_description'),
        'urdf',
        'amr.urdf.xacro'
    ])

    # The xacro output is a plain string; without ParameterValue(..., value_type=str)
    # launch tries to parse it as YAML and fails on Jazzy.
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Gazebo -> ROS bridge: simulation clock + lidar, camera and IMU
    # Syntax: <topic>@<ROS type>[<gz type>   ('[' = gz to ROS only)
    sim_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sim_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/ultrasonic/center/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultrasonic/left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultrasonic/right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ]
    )

    spawn_amr = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_amr',
        output='screen',
        arguments=[
            '-world', 'world_demo',
            '-topic', 'robot_description',
            '-name', 'amr',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.02'
        ]
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Start the broadcaster only after the robot has been spawned,
    # and diff_drive only after the broadcaster is up.
    start_broadcaster_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_amr, on_exit=[joint_state_broadcaster])
    )
    start_diff_drive_after_broadcaster = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[diff_drive_controller])
    )

    return LaunchDescription([
        robot_state_publisher,
        sim_bridge,
        TimerAction(period=3.0, actions=[spawn_amr]),
        start_broadcaster_after_spawn,
        start_diff_drive_after_broadcaster,
    ])