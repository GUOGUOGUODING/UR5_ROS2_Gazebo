from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os
from launch_ros.actions import Node

def generate_launch_description():
    
    ur_gazebo_dir = get_package_share_directory('ur_simulation_gazebo')
    ur_launch = os.path.join(ur_gazebo_dir, 'launch', 'ur_sim_moveit.launch.py')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_launch)
    )
    
    return LaunchDescription([
        gazebo_launch
    ])
    