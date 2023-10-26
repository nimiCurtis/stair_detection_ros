
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

        # ZED Wrapper node
        zion_zed_launch = IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource([
                        PathJoinSubstitution( 
                                [FindPackageShare("zion_zed_ros2_interface"),
                                        "launch",
                                        "zedm.launch.py"]
                        )
                ]),
        )
        
        # stair detection node
        stair_detection_launch = IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource([
                        PathJoinSubstitution( 
                                [FindPackageShare("stair_detection_ros"),
                                        "launch",
                                        "stair_detection.launch.py"]
                        )
                ]),
        )

        # stair modeling node
        stair_modeling_launch = IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource([
                        PathJoinSubstitution( 
                                [FindPackageShare("stair_modeling_ros"),
                                        "launch",
                                        "stair_modeling.launch.py"]
                        )
                ]),
        )
        

        # start the nodes after 5 secs
        on_bringup_time = TimerAction(
                period=5.0,
                actions=[stair_detection_launch,
                        stair_modeling_launch]
        )
        
        # start the fused node after 10 secs
        fused = TimerAction(
                period=10.0,
                actions=[
                        Node(
                                package='stair_detection_ros',
                                executable='stair_detection_fused',
                                output='screen',
                        )
                ]
        )
        

        return LaunchDescription([
                zion_zed_launch,
                on_bringup_time,
                fused
        ])
