
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
        
        packge_name = 'stair_detection_ros'
        
        # define ns as robot name, and nodes names
        robot = 'zion'
        stair_det_fused_node_name = 'stair_detection_fused'
        stair_det_node_name = 'stair_detection'
        stair_modeling_node_name = 'stair_modeling'
        
        # get parameters from yaml    
        config = os.path.join(
                get_package_share_directory(packge_name),
                'config',
                'stair_detection_fused.yaml'
                )

        # launch ZED Wrapper node
        zion_zed_launch = IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource([
                        PathJoinSubstitution( 
                                [FindPackageShare("zion_zed_ros2_interface"),
                                        "launch",
                                        "zedm.launch.py"]
                        )
                ]),
        )


        # launch stair detection node
        stair_detection_launch = IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource([
                        PathJoinSubstitution( 
                                [FindPackageShare(packge_name),
                                        "launch",
                                        "stair_detection.launch.py"]
                        )
                ]),
                launch_arguments={
                        'robot': robot,
                        'node_name': stair_det_node_name,
                }.items()
        )

        # stair modeling node
        # stair_modeling_launch = IncludeLaunchDescription(
        #         launch_description_source=PythonLaunchDescriptionSource([
        #                 PathJoinSubstitution( 
        #                         [FindPackageShare("stair_modeling_ros"),
        #                                 "launch",
        #                                 "stair_modeling.launch.py"]
        #                 )
        #         ]),
        # )
        

        # start the nodes after 5 secs
        on_bringup_time = TimerAction(
                period=5.0,
                actions=[stair_detection_launch]#,
                        #stair_modeling_launch]
        )
        
        # start the fused node after 10 secs
        fused = TimerAction(
                period=10.0,
                actions=[
                        Node(
                                package=packge_name,
                                executable='stair_detection_fused',
                                name=stair_det_fused_node_name,
                                namespace=robot,
                                output='screen',
                                parameters=[config,
                                        { 
                                        'detection_topic_in': '/'+robot+'/'+stair_det_node_name+'/detection',
                                        'stair_topic_in': '/'+robot+'/'+stair_modeling_node_name+'/detection'
                                        }]
                        )
                ]
        )


        return LaunchDescription([
                zion_zed_launch,
                on_bringup_time,
                fused
        ])
