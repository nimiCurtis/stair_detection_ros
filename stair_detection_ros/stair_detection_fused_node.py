#!/usr/bin/env python3

import time

import rclpy
from rclpy.exceptions import ROSInterruptException
from rclpy.node import Node

from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D

#document https://github.com/ros2/message_filters/blob/rolling/index.rst
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
import message_filters

#Custom imports
from zion_msgs.msg import EnvGeoStamped

class StairDetectionFused(Node):
    
    def __init__(self):
        super().__init__("stair_detection_fused") #Nodes name must be equal to the node
        self.get_logger().info(f"{self.get_name()} is running")

        # params = self.init_params()
        
        self.detector_sub = message_filters.Subscriber(self, 
                                                         Detection2D, 
                                                         '/zion/stair_detection/detection')#params['detector_topic'],)
                                                         
        
        self.stair_sub = message_filters.Subscriber(self, 
                                                    EnvGeoStamped, 
                                                    '/stair_modeling_ros/env_geo_params') #params['modeling_topic'])
        
        self.fused_pub = self.create_publisher(Bool,
                                               '/fused_topic',
                                               10
        )
        
        self.ats = message_filters.ApproximateTimeSynchronizer(
            fs=[self.detector_sub, self.stair_sub],
            queue_size= 5,
            slop=0.1)

        self.ats.registerCallback(self.topics_sync_callback) 

        self.prev_time = time.time()

    def init_params():
        pass
    
    def topics_sync_callback(self, detection_msg:Detection2D, stair_msg:EnvGeoStamped):
        self.get_logger().info("sync")
        cur_time = time.time()
        self.get_logger().info(f"dt = {cur_time-self.prev_time}")
        
        self.prev_time = cur_time
        
        # just an example
        msg = Bool()
        msg.data = True
        self.fused_pub.publish(msg)
        
        
        
        ## TODO: on the refactoring
        # - from detection 2D project to detection 3D using EnvGeoStamped and camera info topic
        # - use the current hypothesis

def main():

    try:
        rclpy.init()
        running_node = StairDetectionFused()
        rclpy.spin(running_node)

    except KeyboardInterrupt:
        running_node.get_logger().info('Keyboard interrupt, shutting down.\n')
        running_node.destroy_node()
        rclpy.shutdown()

    except ROSInterruptException as error:
        print(error)
        running_node.get_logger().error('Caught Exception error, shutting down.\n')
        running_node.destroy_node()
        rclpy.shutdown()
        pass

if __name__ == '__main__':
    main()
