import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

	map_file = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))

	return LaunchDescription([
	 	Node(
		    package='nav2_map_server',
		    executable='map_server',
		    name='map_server',
		    output='screen',
		    parameters=[{'yaml_filename':map_file},
		    		{'use_sim_time':False}]
		    ),
		Node(
		    package='nav2_lifecycle_manager',
		    executable='lifecycle_manager',
		    name='lifecycle_manager_localization',
		    output='screen',
		    parameters=[{'autostart': True},
		    		{'use_sim_time': False},
		                {'node_names': ['map_server']}]
		    ),
		])
