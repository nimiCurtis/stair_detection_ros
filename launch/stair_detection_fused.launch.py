import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

        
        # ZED Wrapper node
        zion_zed_launch = IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource([
                        PathJoinSubstitution( 
                                [FindPackageShare("zion_zed_ros2_interface"), "launch", "zedm.launch.py"]
                        )
                ]),
        )
        
        stair_detection_launch = IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource([
                        PathJoinSubstitution( 
                                [FindPackageShare("stair_detection_ros"), "launch", "stair_detection.launch.py"]
                        )
                ]),
        )

        stair_modeling_launch = IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource([
                        PathJoinSubstitution( 
                                [FindPackageShare("stair_modeling_ros"), "launch", "stair_modeling.launch.py"]
                        )
                ]),
        )
        

        on_bringup_time = TimerAction(
                period=5.0,
                actions=[stair_detection_launch,
                         stair_modeling_launch]
                
        )
        
        fused = TimerAction(
                period=10.0,
                actions=[
                        Node(
                                package='stair_detection_ros',
                                executable='stair_detection_fused_node',
                                output='screen',
                        )
                ]
        )
        

        # # group detection and modeling
        # detection_and_modeling_group = GroupAction(
        #         actions=[stair_detection_launch,stair_modeling_launch]
        # )

        # # get parameters from yaml    
        # config = PathJoinSubstitution( 
        #         [FindPackageShare("stair_detection_ros"), "params", "stair_detection_params.yaml"]
        # )
        
        # # set launch os stair detector
        # launch_stair_detection =   Node(
        #         package='stair_detection_ros',
        #         executable='stair_detection_node',
        #         output='screen',
        #         parameters=[config]
        #         )

        # return launch file
        return LaunchDescription([
                # zion_zed_launch,
                on_bringup_time,
                fused
    ])
    