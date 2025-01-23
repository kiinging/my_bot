from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    def robot_state_publisher(context):
        performed_description_format = LaunchConfiguration('description_format').perform(context)
        # Get URDF or SDF via xacro
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('my_bot'),
                    'description',
                    f'robot.{performed_description_format}.xacro'
                ]),
            ]
        )
        params = {'robot_description': robot_description_content,'use_sim_time': use_sim_time}
        
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
            ]
        )
        return [node_robot_state_publisher]

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('my_bot'),
            'config',
            'diff_drive_controller.yaml',
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'diff_drive', '-allow_renaming', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    # Node to bridge messages like /cmd_vel and /odom
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            PathJoinSubstitution([
                FindPackageShare('my_bot'),
                'config',
                'gz_bridge.yaml'
            ])
        ],
        output='screen',
    )


    # Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('my_bot'),
                'config',
                'my_bot_sensors.rviz',
            ])
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Include joystick launch file
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_bot'),
                'launch',
                'joystick.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

      # Define the gz_image_bridge_node
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image"],
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time,
             'camera.image.compressed.jpeg_quality': 60},
        ],
    )

    # Define the relay_camera_info_node
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Define the ekf_node
    ekf_config_file = PathJoinSubstitution([
        FindPackageShare('my_bot'),
        'config',
        'ekf.yaml'
    ])
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': use_sim_time},
        ],
    )


    ld = LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),
        bridge_node,
        gz_spawn_entity,
        rviz_node,
        joystick_launch,  # Add joystick node here
        gz_image_bridge_node,  # Include the image bridge node
        relay_camera_info_node,  # Include the relay node
        ekf_node,  # Include the EKF node
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),
    ])
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld
