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
    config_rviz2 = os.path.join(
        get_package_share_directory('stair_detection_ros'),
        'rviz2',
        'stair_detection.rviz')
    
    launch_rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=[["-d"], [config_rviz2] ]
        )
    
        # return launch file
    return LaunchDescription([
        launch_rviz
    ])