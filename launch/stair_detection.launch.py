import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    packge_name = 'stair_detection_ros'

    # define ns as robot name.
    robot = LaunchConfiguration('robot',default="zion")
    node_name = LaunchConfiguration('node_name',default="stair_detection")

    # get parameters from yaml    
    config = os.path.join(
        get_package_share_directory(packge_name),
        'config',
        'stair_detection.yaml'
        )

    # set launch of stair detector
    stair_detection_node =   Node(
            package=packge_name,
            executable='stair_detection',
            name=node_name,
            namespace=robot,
            output='screen',
            parameters=[config]
            )

    # return launch file
    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT', value='{time} [{name}] [{severity}] {message}'),
        stair_detection_node
    ])
