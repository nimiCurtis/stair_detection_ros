#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import Header
from vision_msgs.msg import Detection2D
from geometry_msgs.msg import PoseStamped
#document https://github.com/ros2/message_filters/blob/rolling/index.rst
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
import message_filters

#Custom imports
from zion_msgs.msg import StairStamped, StairDetStamped

class StairDetectionFused(Node):
    
    def __init__(self):
        super().__init__("stair_detection_fused") #Nodes name must be equal to the node
        self.get_logger().info(f"{self.get_name()} is running")

        # params = self.init_params()
        
        self.detector_sub = message_filters.Subscriber(self, 
                                                         Detection2D, 
                                                         '/zion/stair_detection/detection')#params['detector_topic'],)
        
        self.stair_sub = message_filters.Subscriber(self, 
                                                    StairStamped, 
                                                    '/stair_modeling_ros/stair') #params['modeling_topic'])
        
        self.fused_det_pub = self.create_publisher(StairStamped,
                                               '/stair_detection_fused/stair',
                                               10
        )

        self.fused_pose_pub = self.create_publisher(PoseStamped,
                                                    '/stair_detection_fused/pose',
                                                    10)
        
        self.ats = message_filters.ApproximateTimeSynchronizer(
            fs=[self.detector_sub, self.stair_sub],
            queue_size = 10,
            slop=0.1)

        self.ats.registerCallback(self.topics_sync_callback) 

        self.prev_time = time.time()

    def init_params():
        pass
    
    def topics_sync_callback(self, detection_msg:Detection2D, stair_msg:StairStamped):
        self.get_logger().info("sync")
        cur_time = time.time()
        self.get_logger().info(f"dt = {cur_time-self.prev_time}")
        self.prev_time = cur_time
                
        pose_msg = PoseStamped()
        stair_fused_msg = StairDetStamped
        header = Header()
        header.frame_id = detection_msg.header.frame_id
        header.stamp = self.get_clock().now().to_msg()
        
        stair_fused_msg.header = header
        pose_msg.header = header

        if len(detection_msg.results)==1:
            result = detection_msg.results[0]
            bbox = detection_msg.bbox
            if result.id == stair_msg.stair.id:
                stair_fused_msg.stair = stair_msg.stair
                stair_fused_msg.bbox = bbox
                pose_msg.pose = stair_fused_msg.stair.pose
                self.fused_det_pub.publish(stair_fused_msg)
                self.fused_pose_pub.publish(pose_msg)

def main(args=None):

    # init node
    rclpy.init(args=args)
    detect_node = StairDetectionFused()
    
    # Create a SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    # Add the node to the executor
    executor.add_node(detect_node)

    try:
        # Spin the executor
        executor.spin()
    except KeyboardInterrupt as error:
        print(error)
        detect_node.get_logger().warning('KeyboardInterrupt error, shutting down.\n')
    
    finally:
        detect_node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
