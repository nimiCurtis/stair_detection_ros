import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # get parameters from yaml    
    config = os.path.join(
        get_package_share_directory('stair_detection_ros'),
        'params',
        'stair_detection_params.yaml'
        )
    
    # set launch os stair detector
    launch_stair_detection =   Node(
            package='stair_detection_ros',
            executable='stair_detection_node',
            output='screen',
            parameters=[config]
            )
    
    # return launch file
    return LaunchDescription([
        launch_stair_detection
    ])
    