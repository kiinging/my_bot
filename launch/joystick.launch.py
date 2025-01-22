from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_dir    = get_package_share_directory('my_bot')
    
# (joy_node)joy -> teleop_node(cmd_vel_joy) -> twist_mux(cmd_vel)
    joy_node = Node(
            package='joy',
            executable='joy_node',
            name="joy_node",
            parameters=[ 
                os.path.join(pkg_dir,'config','joystick.yaml'),
                {'use_sim_time': use_sim_time},
            ],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[
                os.path.join(pkg_dir,'config','joy_teleop.yaml'),
                {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel_joy')
            ]
         )
    

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            os.path.join(pkg_dir, "config", "twist_mux_topics.yaml"),
            os.path.join(pkg_dir, "config", "twist_mux_locks.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/cmd_vel_out", "/cmd_vel_unstamped")
        ],
    )
   
    twist_stamper = Node(
            package='twist_stamper',
            executable='twist_stamper',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_in','/cmd_vel_unstamped'),
                        ('/cmd_vel_out','/diff_drive_base_controller/cmd_vel')]
        )


    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(joy_node)
    launchDescriptionObject.add_action(teleop_node)
    launchDescriptionObject.add_action(twist_mux_node)
    launchDescriptionObject.add_action(twist_stamper)



  
    return launchDescriptionObject