# This launch file will include rsp_gz.launch.py that we created earlier
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_dir = get_package_share_directory('my_bot')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_dir)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path


    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use sim time if true'
    )
    
    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control', default_value='false',
        description='Use ros2_control if false'
    )


    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='world.sdf',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='robot.urdf.xacro',
        description='Name of the URDF description to load'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    world = LaunchConfiguration('world')

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_dir,'launch','rsp.launch.py')), 
            launch_arguments={
                'use_sim_time': use_sim_time, 
                'use_ros2_control': use_ros2_control,}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_dir,'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )


    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_dir, 'launch', 'gz.launch.py'),),
            launch_arguments={
                'world': world, }.items()
    )


    # Spawn the URDF model using the `/world/<world_name>/create` service
    # It loads the robot into Gazebo before any other control or communication steps occur.
    spawn_urdf_node = Node(package="ros_gz_sim", executable="create",
                            arguments=["-name", "my_robot",
                                        "-topic", "robot_description",
                                        "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0"],
                            output="screen", parameters=[{'use_sim_time':use_sim_time},])

  

     # Node to bridge messages like /cmd_vel and /odom
    bridge_params = os.path.join(pkg_dir,'config','gz_bridge.yaml')
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    #         "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    #         "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
    #         "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
    #         "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
    #         #"/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
    #         #"/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
    #         "/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
    #         "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
    #         "imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
    #         "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
    #         "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
    #         "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
    #     ],
    #     output="screen",
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #     ]
    # )


    # Node to bridge camera image with image_transport and compressed_image_transport
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time,
             'camera.image.compressed.jpeg_quality': 60},
        ],
    )

    # Relay node to republish /camera/camera_info to /camera/image/camera_info
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )

    # trajectory_node = Node(
    #     package='mogi_trajectory_server',
    #     executable='mogi_trajectory_server',
    #     name='mogi_trajectory_server'
    # )


    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )


    # trajectory_odom_topic_node = Node(
    #     package='mogi_trajectory_server',
    #     executable='mogi_trajectory_server_topic_based',
    #     name='mogi_trajectory_server_odom_topic',
    #     parameters=[{'trajectory_topic': 'trajectory_raw'},
    #                 {'odometry_topic': 'odom'}]
    # )
   
       # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'my_bot_sensors.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )


    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(use_sim_time_arg )
    launchDescriptionObject.add_action(use_ros2_control_arg)
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(joystick)
    launchDescriptionObject.add_action(gz_launch)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(rsp_launch)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(gz_image_bridge_node)
    launchDescriptionObject.add_action(relay_camera_info_node)
    # launchDescriptionObject.add_action(trajectory_node)
    launchDescriptionObject.add_action(ekf_node)
    # launchDescriptionObject.add_action(trajectory_odom_topic_node)
    launchDescriptionObject.add_action(rviz_node)


    return launchDescriptionObject