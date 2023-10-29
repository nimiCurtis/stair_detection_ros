#!/usr/bin/env python3

# Standard library imports
import time

# ROS application/library specific imports
import rclpy
from rclpy.node import Node, QoSProfile
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D
from geometry_msgs.msg import PoseStamped
import message_filters

# Custom imports
from zion_msgs.msg import StairStamped, StairDetStamped

class StairDetectionFused(Node):
    """
    StairDetectionFused is a ROS2 node responsible for fusing data from multiple sources 
    related to stair detection.
    
    Attributes:
        detector_sub: Subscriber for detected objects.
        stair_sub: Subscriber for stair detection.
        fused_det_pub: Publisher for fused stair detection.
        fused_pose_pub: Publisher for fused pose of stair.
        
    Methods:
        - __init__: Initializes the node and its attributes.
        - topics_sync_callback: Callback function that processes and sync the incoming messages.
    """
    
    def __init__(self):
        super().__init__("stair_detection_fused")  # Node's name must be equal to the node's class name
        self.get_logger().info("****************************************")
        self.get_logger().info(f"      {self.get_name()} is running     ")
        self.get_logger().info("****************************************")

        # Loading parameters
        params = self.init_params()
        
        # Initializing subscribers for detected objects and stairs
        detSubQos = QoSProfile(depth=10)
        self.detector_sub = message_filters.Subscriber(self, Detection2D,
                                                    params.get('detection_topic_in'),
                                                    qos_profile=detSubQos)
        StairSubQos = QoSProfile(depth=10)
        self.stair_sub = message_filters.Subscriber(self, StairStamped,
                                                    params.get('stair_topic_in'),
                                                    qos_profile=StairSubQos)
        
        # Initializing publishers for fused detection and pose
        detPubQos = QoSProfile(depth=10)
        self.fused_det_pub = self.create_publisher(StairStamped,
                                                params.get('stair_topic_out'),
                                                detPubQos)
        
        PosePubQos = QoSProfile(depth=10)
        self.fused_pose_pub = self.create_publisher(PoseStamped,
                                                    params.get('pose_topic_out'),
                                                    PosePubQos)

        # Synchronize the incoming messages based on their timestamp
        self.ats = message_filters.ApproximateTimeSynchronizer(
            fs=[self.detector_sub, self.stair_sub],
            queue_size=10,
            slop=0.1)
        self.ats.registerCallback(self.topics_sync_callback) 

        self.prev_time = time.time()

    def init_params(self):
        """
        Initialize the parameters for the node from ROS parameters.

        :return: Dictionary of parameters
        :rtype: dict
        """

        self.get_logger().info('************   Parameters   ************')
        
        detection_topic_in = self.declare_parameter('general.detection_topic_in',
                                            '/zion/stair_detection/detection')
        detection_topic_in = self.get_parameter('general.detection_topic_in').get_parameter_value().string_value
        self.get_logger().info('detection_topic_in: %s' % detection_topic_in)

        stair_topic_in = self.declare_parameter('general.stair_topic_in',
                                            '/zion/stair_modeling/stair')
        stair_topic_in = self.get_parameter('general.stair_topic_in').get_parameter_value().string_value
        self.get_logger().info('stair_topic_in: %s' % stair_topic_in)

        pose_topic_out = self.declare_parameter('general.pose_topic_out',
                                            '/zion/stair_detection_fused/pose')
        pose_topic_out = self.get_parameter('general.pose_topic_out').get_parameter_value().string_value
        self.get_logger().info('pose_topic_out: %s' % pose_topic_out)

        stair_topic_out = self.declare_parameter('general.stair_topic_out',
                                            '/zion/stair_detection_fused/stair')
        stair_topic_out = self.get_parameter('general.stair_topic_out').get_parameter_value().string_value
        self.get_logger().info('stair_topic_out: %s' % stair_topic_out)
        
        params_dic = {
            'detection_topic_in': detection_topic_in,
            'stair_topic_in': stair_topic_in,
            'stair_topic_out': stair_topic_out,
            'pose_topic_out': pose_topic_out
        }
        
        self.get_logger().info("****************************************")

        return params_dic

    def topics_sync_callback(self, detection_msg: Detection2D, stair_msg: StairStamped):
        """
        Callback function to handle synchronized messages from the detection and stair topics.
        
        Args:
            detection_msg (Detection2D): The message from the detection topic.
            stair_msg (StairStamped): The message from the stair topic.
        """
        # self.get_logger().info("sync")
        # cur_time = time.time()
        # self.get_logger().info(f"dt = {cur_time-self.prev_time}")
        # self.prev_time = cur_time
                
        pose_msg = PoseStamped()
        stair_fused_msg = StairDetStamped()
        header = Header()
        header.frame_id = detection_msg.header.frame_id
        header.stamp = self.get_clock().now().to_msg()
        
        stair_fused_msg.header = header
        pose_msg.header = header

        # Check if a single detection result matches with the stair model ID and publish the fused data
        if len(detection_msg.results) == 1:
            result = detection_msg.results[0]
            bbox = detection_msg.bbox
            if result.id == stair_msg.stair.id:
                stair_fused_msg.stair = stair_msg.stair
                stair_fused_msg.bbox = bbox
                pose_msg.pose = stair_fused_msg.stair.pose
                self.fused_det_pub.publish(stair_fused_msg)
                self.fused_pose_pub.publish(pose_msg)

def main(args=None):

    # Initializing Node
    rclpy.init(args=args)
    detect_node = StairDetectionFused()
    
    # Create a SingleThreadedExecutor and add the node to it
    executor = SingleThreadedExecutor()
    executor.add_node(detect_node)

    try:
        # Spin the executor to process callbacks
        executor.spin()
    except KeyboardInterrupt as error:
        print(error)
        detect_node.get_logger().warning('KeyboardInterrupt error, shutting down.')
    finally:
        detect_node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
