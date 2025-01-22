import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_bot')

    model_arg = DeclareLaunchArgument(
        'model', default_value='robot.urdf.xacro',
        description='Name of the URDF description to load'
    )

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    model = LaunchConfiguration('model')

   
    # Process the URDF file
    urdf_file_path = PathJoinSubstitution([
        pkg_dir,  # Replace with your package name
        "description",
        model
    ]) 

    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', urdf_file_path, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    launchDescriptionObject =  LaunchDescription()


    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(robot_state_publisher_node)

    return launchDescriptionObject