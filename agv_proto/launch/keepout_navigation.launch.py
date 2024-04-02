import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('agv_proto')
    
    
    nav2_yaml = os.path.join(get_package_share_directory('agv_proto'), 'config', 'keepout_nav2_params.yaml')
    #map_file = os.path.join(get_package_share_directory('agv_proto'), 'maps', 'remap.yaml')
    map_file = os.path.join(get_package_share_directory('agv_proto'), 'maps', 'my_home_map.yaml')
    #default_bt_xml_filename= os.path.join(get_package_share_directory('agv_proto'), 'config', 'navigate_w_replanning_and_recovery.xml')
    
    lifecycle_nodes = ['map_server', 
                       'amcl',
                       'planner_server',
                       'controller_server',
                       'recoveries_server',
                       'bt_navigator'
                       ]
    #remappings = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped'),('/odom','/odom_filtered')] #odom_filtered for sensor fusion only
    remappings = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'yaml_filename':map_file}]),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]),
                     
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml]),
            
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[nav2_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml]),
            
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]),
        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    arguments=['-d', rviz_config_dir],
        #    parameters=[{'use_sim_time': True}],
        #    output='screen')
    ])
