import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    # define ns as robot name.
    robot = 'zion'
    
    # get parameters from yaml    
    config = os.path.join(
        get_package_share_directory('stair_detection_ros'),
        'config',
        'stair_detection.yaml'
        )

    # set launch os stair detector
    stair_detection_node =   Node(
            package='stair_detection_ros',
            executable='stair_detection',
            output='screen',
            namespace=robot,
            parameters=[config]
            )

    # return launch file
    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        stair_detection_node
    ])
