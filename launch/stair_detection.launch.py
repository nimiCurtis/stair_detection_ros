import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # get parameters from yaml    
    config = os.path.join(
        get_package_share_directory('stair_detection_ros'),
        'params',
        'stair_detection_params.yaml'
        )

    # set launch os stair detector
    stair_detection_node =   Node(
            package='stair_detection_ros',
            executable='stair_detection_node',
            output='screen',
            parameters=[config]
            )

    # return launch file
    return LaunchDescription([
        stair_detection_node
    ])
    