import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    # /mxck2_ws/install/line_follower → /mxck2_ws/src/line_follower
    pkg_dir = get_package_prefix('wall_runner').replace('install', 'src') 


    return LaunchDescription([
        # Launch the Parking Node from the 'smart_parking' package with a YAML parameter file
        Node(
            package='wall_runner',
            executable='wall_follower',
            name='wall_follower',
            output='screen',
            parameters=[pkg_dir + "/config/uss_mapping.yaml"]
        )
    ])