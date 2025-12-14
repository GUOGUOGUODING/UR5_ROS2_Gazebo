from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    ur_driver_dir = get_package_share_directory('ur_robot_driver')
    ur_driver_launch = os.path.join(ur_driver_dir, 'launch', 'ur_control.launch.py')

    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_driver_launch),
        launch_arguments={
            'ur_type': 'ur5',               
            'robot_ip': '172.22.22.2',       
            'launch_rviz': 'true'            
        }.items()
    )

    control_node = Node(
        package='ar',
        executable='ur5_control',
        output='screen'
    )

    return LaunchDescription([
        ur_launch,
        control_node
    ])
