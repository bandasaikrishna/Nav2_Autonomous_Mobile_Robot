import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Create an instance of LaunchDescription
    ld = LaunchDescription()
    
    
    # Include another launch file
    diff_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('agv_proto'), 'launch', 'diffbot_system.launch.py'))
    )
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar.launch.py'))
    )
    
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('agv_proto'), 'launch', 'navigation.launch.py'))
    )
    
    
    
    actions = [
        #rplidar,
        diff_controller,  # Include the other launch file
        navigation,
    ]
    
    return LaunchDescription(actions)
