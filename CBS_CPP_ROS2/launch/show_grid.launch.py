import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import lifecycle_msgs.msg

def generate_launch_description():

    params_file = os.path.join(get_package_share_directory('mapf'), 'config', 'warehouse_params.yaml')
    mapf_params_file = os.path.join(get_package_share_directory('mapf'), 'config', 'mapf_params.yaml')
    map_file = os.path.join(get_package_share_directory('mapf'), 'maps', 'warehouse.yaml')
    rviz_config = os.path.join(get_package_share_directory('mapf'), 'config', 'warehouse.rviz')

    return LaunchDescription([
        Node(
            package = 'nav2_map_server',
            executable = 'map_server',
            name = 'map_server',
            output = 'screen',
            parameters=[params_file,
                        {'yaml_filename':map_file}]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        Node(
            package='mapf',
            executable='mapf',
            name='mapf_cbs',
            output='screen',
            # prefix=['xterm -e gdb -ex run --args'],
            parameters=[mapf_params_file]
        )

      
    ])