import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

def generate_launch_description():
    pkg_share = get_package_share_directory('amr_leader_follower')

    xacro_file = os.path.join(pkg_share, 'urdf', '2_wheel.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Whether to run Gazebo headless (server only)'
    )
    headless = LaunchConfiguration('headless')

    gz_args_val = PythonExpression([
        "'-s -r ' + '", world_file, "' if '", headless, "' == 'true' else '-r ' + '", world_file, "'"
    ])

    # 1. Include the Gazebo simulation launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args_val}.items()
    )

    # 2. Leader Robot Setup
    robot_description_leader = Command(['xacro ', xacro_file, ' config_file:=leader_controllers.yaml', ' robot_namespace:=leader'])
    robot_state_publisher_leader = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='leader',
        parameters=[{
            'robot_description': robot_description_leader,
            'use_sim_time': True,
            'frame_prefix': 'leader/'
        }],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ],
        output='screen'
    )

    spawn_leader = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/leader/robot_description',
            '-name', 'leader',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    joint_state_broadcaster_leader = Node(
        package='controller_manager',
        executable='spawner',
        namespace='leader',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_spawner_leader = Node(
        package='controller_manager',
        executable='spawner',
        namespace='leader',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    # 3. Static Transform Publisher to connect odom frame to map frame
    static_tf_leader = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': True}],
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'map', '--child-frame-id', 'leader/odom'],
        output='screen'
    )

    # 4. Gazebo Parameter Bridge for Clock, scan, and camera topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/leader/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/leader/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen'
    )

    return LaunchDescription([
        headless_arg,
        gz_sim,
        robot_state_publisher_leader,
        spawn_leader,
        joint_state_broadcaster_leader,
        diff_drive_spawner_leader,
        static_tf_leader,
        bridge
    ])
