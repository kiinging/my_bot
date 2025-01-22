# Use IncludLaunchDescription function to launch another launch file  
# in this case gz_sim.launch.py from ros_gz_sim package
# DeclareLaunchArgument function define 'world' argument for the 
# LaunchConfiguration function to use.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_bot') #<--- CHANGE ME

    # Check if we're told to use sim time
    world = LaunchConfiguration('world')


    # Add your own gazebo library path here
    gazebo_models_path = "/home/pizza/gazebo_models"
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': [PathJoinSubstitution([ 
                pkg_dir, 'worlds',  world]),
                #TextSubstitution(text=' -r -v -v1 --render-engine ogre')],
                TextSubstitution(text=' -r -v -v1')],
                'on_exit_shutdown': 'true'}.items()
                )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(gazebo)

    return launchDescriptionObject