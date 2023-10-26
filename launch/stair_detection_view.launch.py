import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # get parameters from rviz config    
    config_rviz2 = os.path.join(
        get_package_share_directory('stair_detection_ros'),
        'rviz2',
        'stair_detection.rviz')
    
    # set launch of rviz node
    rviz_node = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=[["-d"], [config_rviz2] ]
        )
    
        # return launch file
    return LaunchDescription([
        rviz_node
    ])