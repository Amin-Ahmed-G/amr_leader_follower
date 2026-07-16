import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('amr_leader_follower')

    localization_source_arg = DeclareLaunchArgument(
        'localization_source',
        default_value='ground_truth',
        description='Localization source: "ground_truth", "slam", or "saved_map"'
    )
    localization_source = LaunchConfiguration('localization_source')

    run_follower_arg = DeclareLaunchArgument(
        'run_follower',
        default_value='true',
        description='Whether to spawn the follower robot and run its controller'
    )
    run_follower = LaunchConfiguration('run_follower')

    run_nav2_follower_arg = DeclareLaunchArgument(
        'run_nav2_follower',
        default_value='false',
        description='Whether to run the follower Nav2 navigation stack (separate from AMCL localization)'
    )
    run_nav2_follower = LaunchConfiguration('run_nav2_follower')

    xacro_file = os.path.join(pkg_share, 'urdf', '2_wheel.xacro')
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='industrial_world.sdf',
        description='World file to launch'
    )
    world_name = LaunchConfiguration('world')
    world_file = PythonExpression(["'", os.path.join(pkg_share, 'worlds'), "/' + '", world_name, "'"])

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Whether to run Gazebo headless (server only)'
    )
    headless = LaunchConfiguration('headless')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port for rosbridge_websocket'
    )
    port = LaunchConfiguration('port')

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

    # 3. Follower Robot Setup
    robot_description_follower = Command(['xacro ', xacro_file, ' config_file:=follower_controllers.yaml', ' robot_namespace:=follower'])
    robot_state_publisher_follower = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='follower',
        condition=IfCondition(run_follower),
        parameters=[{
            'robot_description': robot_description_follower,
            'use_sim_time': True,
            'frame_prefix': 'follower/'
        }],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ],
        output='screen'
    )
    spawn_follower = Node(
        package='ros_gz_sim',
        executable='create',
        condition=IfCondition(run_follower),
        arguments=[
            '-topic', '/follower/robot_description',
            '-name', 'follower',
            '-x', '-1.2',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    joint_state_broadcaster_follower = Node(
        package='controller_manager',
        executable='spawner',
        namespace='follower',
        condition=IfCondition(run_follower),
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    diff_drive_spawner_follower = Node(
        package='controller_manager',
        executable='spawner',
        namespace='follower',
        condition=IfCondition(run_follower),
        arguments=['diff_drive_controller'],
        output='screen'
    )
    ekf_follower = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    namespace='follower',
    condition=IfCondition(run_follower),
    parameters=[
        os.path.join(pkg_share, 'config', 'ekf_follower.yaml'),
        {'use_sim_time': True}
    ],
    remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')],
    output='screen'
)

    # 4. AMCL Localization for follower robot against the leader's SLAM map

    amcl_params_file = os.path.join(pkg_share, 'config', 'amcl_params_follower.yaml')

    amcl_follower = Node(
        package='nav2_amcl',
        executable='amcl',
        namespace='follower',
        name='amcl',
        condition=IfCondition(PythonExpression(["'", localization_source, "' == 'slam' or ('", localization_source, "' == 'saved_map' and '", run_follower, "' == 'true')"])),
        parameters=[
            amcl_params_file,
            {'use_sim_time': True}  # Guaranteed — YAML namespace may not match node name
        ],
        remappings=[
            ('/follower/map', '/map'),   # read from global SLAM map
            ('/map', '/map'),
            ('map', '/map'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('tf', '/tf'),               # publish/read map↔odom on shared /tf
            ('tf_static', '/tf_static'),
        ],
        output='screen'
    )

    amcl_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl_follower',
        namespace='follower',
        condition=IfCondition(PythonExpression(["'", localization_source, "' == 'slam' or ('", localization_source, "' == 'saved_map' and '", run_follower, "' == 'true')"])),
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['amcl']
        }],
        output='screen'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='leader',
        condition=IfCondition(PythonExpression(["'", localization_source, "' == 'saved_map'"])),
        parameters=[{
            'yaml_filename': os.path.join(pkg_share, 'maps', 'industrial_world.yaml'),
            'frame_id': 'map',
            'use_sim_time': True
        }],
        remappings=[
            ('/map', '/map'),
            ('map', '/map'),
        ],
        output='screen'
    )

    leader_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        namespace='leader',
        name='amcl',
        condition=IfCondition(PythonExpression(["'", localization_source, "' == 'saved_map'"])),
        parameters=[
            os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            {'use_sim_time': True}
        ],
        remappings=[
            ('/map', '/map'),
            ('map', '/map'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
        output='screen'
    )

    leader_amcl_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl_leader',
        namespace='leader',
        condition=IfCondition(PythonExpression(["'", localization_source, "' == 'saved_map'"])),
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['amcl']
        }],
        output='screen'
    )

    # 5. Gazebo Parameter Bridge
    # 10-second delay: Gazebo transport discovery takes time to register
    # all sensor topics. 5s was not enough — bridge connected before lidar
    # sensor was ready, causing silent scan failures.
    #
    # Shared: clock + lidar scans.
    # Ground-truth mode only: also bridge Gazebo PosePublisher Pose_V topics
    # (/model/{leader,follower}/pose → TFMessage). Those ROS topics are then
    # remapped onto /tf so ground_truth_bridge.py's TF listener can look up
    # industrial_world|empty_world → leader|follower and publish map→*/odom.
    # (YAML keeps unique ros_topic_names; merging onto /tf via remaps avoids
    # the "two entries, same ros_topic_name" bridge breakage.)
    # SLAM mode: poses excluded — SLAM/AMCL own map→odom; extra world TF
    # would add CPU load and confuse the tree.
    bridge_common_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/leader/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/follower/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/follower/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
    ]
    bridge_pose_args = [
        '/model/leader/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/model/follower/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    ]

    bridge = TimerAction(
        period=10.0,
        condition=IfCondition(PythonExpression(["'", localization_source, "' == 'slam'"])),
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=bridge_common_args,
                output='screen'
            )
        ]
    )

    bridge_ground_truth = TimerAction(
        period=10.0,
        condition=IfCondition(PythonExpression(["'", localization_source, "' == 'ground_truth'"])),
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=bridge_common_args + bridge_pose_args,
                remappings=[
                    ('/model/leader/pose', '/tf'),
                    ('/model/follower/pose', '/tf'),
                ],
                output='screen'
            )
        ]
    )

    ground_truth_bridge = Node(
        package='amr_leader_follower',
        executable='ground_truth_bridge.py',
        condition=IfCondition(PythonExpression(["'", localization_source, "' == 'ground_truth'"])),
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/leader/camera/image_raw', '/follower/camera/image_raw'],
        output='screen'
    )

    # 6. Custom Control Nodes
    follower_controller = Node(
        package='amr_leader_follower',
        executable='follower_controller',
        condition=IfCondition(run_follower),
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'),
                         'launch', 'rosbridge_websocket_launch.xml')
        ),
        launch_arguments={
            'port': port,
            'respawn': 'true'
        }.items()
    )

    # 7. Include the SLAM Toolbox online async launch
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params_file
        }.items()
    )
    
    slam_toolbox_with_remappings = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_source, "' == 'slam'"])),
        actions=[
            SetRemap(src='scan', dst='/leader/scan'),
            SetRemap(src='odom', dst='/leader/diff_drive_controller/odom'),
            SetRemap(src='/odom', dst='/leader/diff_drive_controller/odom'),
            SetRemap(src='map', dst='/map'),
            slam_toolbox
        ]
    )

    # 8. Include the Nav2 navigation launch with our custom params file
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    nav2_params_follower_file = os.path.join(pkg_share, 'config', 'nav2_params_follower.yaml')
    nav2_navigation_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_follower_file,
            # 'namespace': 'follower' is intentionally omitted here.
            # When passed, nav2_bringup applies the namespace INSIDE the launch
            # file before the GroupAction's SetRemap can intercept it, causing
            # nav2 nodes to subscribe to /follower/tf instead of /tf.
            # PushRosNamespace('follower') in the GroupAction handles namespacing
            # AFTER SetRemap is applied, which is the correct order.
        }.items()
    )

    # nav2_bringup remaps ('/tf','tf') so namespaced nodes would otherwise
    # subscribe to /leader/tf (empty). Absolute identity remaps must come
    # first — ROS2 uses first-match — to keep Nav2 on the shared /tf tree.
    # Leave the internal cmd_vel name unremapped so Nav2's velocity_smoother
    # and collision_monitor chain can relay normally; only the collision monitor
    # output should reach the real actuator topic.
    nav2_with_remappings = GroupAction(
        actions=[
            PushRosNamespace('leader'),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            SetRemap(src='tf', dst='/tf'),
            SetRemap(src='tf_static', dst='/tf_static'),
            SetRemap(src='/map', dst='/map'),
            SetRemap(src='map', dst='/map'),
            SetRemap(src='odom', dst='/leader/diff_drive_controller/odom'),
            SetRemap(src='/odom', dst='/leader/diff_drive_controller/odom'),
            SetRemap(src='map', dst='/map'),
            SetRemap(src='goal_pose', dst='/leader/goal_pose'),
            SetRemap(src='/goal_pose', dst='/leader/goal_pose'),
            nav2_navigation
        ]
    )

    nav2_follower_with_remappings = GroupAction(
        condition=IfCondition(PythonExpression(["'", run_nav2_follower, "' == 'true' and '", run_follower, "' == 'true'"])),
        actions=[
            PushRosNamespace('follower'),
            SetRemap(src='/tf', dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            SetRemap(src='tf', dst='/tf'),
            SetRemap(src='tf_static', dst='/tf_static'),
            SetRemap(src='/map', dst='/map'),
            SetRemap(src='map', dst='/map'),
            SetRemap(src='odom', dst='/follower/diff_drive_controller/odom'),
            SetRemap(src='/odom', dst='/follower/diff_drive_controller/odom'),
            SetRemap(src='map', dst='/map'),
            SetRemap(src='goal_pose', dst='/follower/goal_pose'),
            SetRemap(src='/goal_pose', dst='/follower/goal_pose'),
            nav2_navigation_follower
        ]
    )

    # Delay Nav2 until Gazebo controllers publish odom TF (~15s). Without this,
    # lifecycle activate races the spawners and local_costmap times out waiting
    # for leader/odom even when the later TF tree is healthy.
    nav2_delayed = TimerAction(
        period=20.0,
        actions=[nav2_with_remappings]
    )
    nav2_follower_delayed = TimerAction(
        period=20.0,
        condition=IfCondition(PythonExpression(["'", run_nav2_follower, "' == 'true' and '", run_follower, "' == 'true'"])),
        actions=[nav2_follower_with_remappings]
    )

    return LaunchDescription([
        # Enforce isolated localhost-only communication and bind to loopback (127.0.0.1)
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),

        world_arg,
        port_arg,
        run_follower_arg,
        run_nav2_follower_arg,
        localization_source_arg,
        headless_arg,
        gz_sim,
        
        # Leader spawning & publishers
        robot_state_publisher_leader,
        spawn_leader,
        joint_state_broadcaster_leader,
        diff_drive_spawner_leader,
        ekf_follower,
        # Follower spawning & publishers
        robot_state_publisher_follower,
        spawn_follower,
        joint_state_broadcaster_follower,
        diff_drive_spawner_follower,
        
        # Shared TF & Bridges
        amcl_follower,
        amcl_lifecycle_manager,
        map_server,
        leader_amcl,
        leader_amcl_lifecycle_manager,
        bridge,
        bridge_ground_truth,
        ground_truth_bridge,
        image_bridge,
        
        # Proportional Steering Controllers
        follower_controller,
        
        # Web socket bridge
        rosbridge,

        # SLAM and Nav2 Navigation (Nav2 delayed until odom TF exists)
        slam_toolbox_with_remappings,
        nav2_delayed,
        nav2_follower_delayed
    ])
