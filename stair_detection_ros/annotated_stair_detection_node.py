#!/usr/bin/env python3

import os
import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
import numpy as np
import torch
from omegaconf import OmegaConf
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesis 
from ultralytics import YOLO

class StairDetectorNode(Node):
    """StairDetectorNode is a ROS2 node responsible for detecting stair ascending and descending 
    using the YOLO object detection model.
    
    Attributes:
        cv_bridge: A helper object to convert between ROS Image messages and OpenCV images.
        image_sub: Subscriber for image topics.
        detection_data_pub: Publisher for detected object data.
        detection_img_pub: Publisher for images with annotated detections.
        model: Loaded YOLO object detection model.
    
    Methods:
        - __init__: Initializes the node and its attributes.
        - init_params: Loads parameters required by the node.
        - publish: Publishes the annotated image.
        - load_model: Loads the YOLO object detection model.
        - update_detection: Annotates input frame with bounding boxes of detected objects.
        - predict: Predicts objects in the given frame using the loaded YOLO model.
        - image_callback: Callback function that processes incoming image data.
    """

    def __init__(self):

        super().__init__("stair_detection_node") #Nodes name must be equal to the node
        self.get_logger().info("NODE is running")

        # load parameters
        params = self.init_params()
        
        self.cv_bridge = CvBridge()

        self.image_sub  = self.create_subscription(Image, 
                                                   params['camera_topic'], 
                                                   self.image_callback, 10)
        
        self.detection_data_pub = self.create_publisher(Detection2D,
                                                 params['detection_topic_data'],
                                                 10)
        
        self.detection_img_pub = self.create_publisher(Image,
                                                 params['detection_topic_img'],
                                                 10)
        
        device, model_path = params["device"], params["model_path"]
        self.model = self.load_model(model_path,device)

    def init_params(self):
        """
        Initialize the parameters for the node from ROS parameters.

        :return: Dictionary of parameters
        :rtype: dict
        """

        self.get_logger().info('Parameters: ')

        camera_topic = self.declare_parameter('camera_topic', '/zedm/zed_node/rgb/image_rect_color') ## <<< continue
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.get_logger().info('camera_topic: %s' % camera_topic)
        
        detection_topic_data = self.declare_parameter('detection_topic_data', '/zion/stair_detection/detection') ## <<< continue
        detection_topic_data = self.get_parameter('detection_topic_data').get_parameter_value().string_value
        self.get_logger().info('detection_topic_data: %s' % detection_topic_data)
        
        detection_topic_img = self.declare_parameter('detection_topic_img', '/zion/stair_detection/detection_image') ## <<< continue
        detection_topic_img = self.get_parameter('detection_topic_img').get_parameter_value().string_value
        self.get_logger().info('detection_topic_img: %s' % detection_topic_img)
        
        device = self.declare_parameter('model.device', 'cuda')
        device = self.get_parameter('model.device').get_parameter_value().string_value
        self.get_logger().info('device: %s' % device)
        
        model_path = self.declare_parameter('model.model_path', '/home/nimrod/ros2_ws/src/stair_detection_pkg/stair_detection_ros/models/yolov8n.pt')
        model_path = self.get_parameter('model.model_path').get_parameter_value().string_value
        self.get_logger().info('model_path: %s' % model_path)
        
        params_dic = {
            'camera_topic': camera_topic,
            'detection_topic_data': detection_topic_data,
            'detection_topic_img': detection_topic_img,
            'device': device,
            'model_path': model_path
        }
        
        return params_dic

    def publish(self, img_msg:Image):
        """
        Publishes the image message containing detected objects with bounding boxes.
        
        :param img_msg: Image message containing detections
        :type img_msg: Image
        """
        
        # publish image with bbox
        self.detection_img_pub.publish(img_msg)

    def load_model(self, model_path:String, device:String):
        """
        Loads the YOLO model from the specified path and transfers it to the desired device.
        
        :param model_path: Path to the YOLO model file
        :type model_path: String
        :param device: Device to load the model on ('cuda' for GPU, 'cpu' for CPU)
        :type device: String
        :return: Loaded YOLO model
        :rtype: YOLO
        """
        
        # Load the specified model architecture
        model = YOLO(model_path).to(device)
        return model

    def update_detection(self, results, frame):
        """
        Updates the input frame with bounding boxes around detected objects.
        
        :param results: Detection results from the YOLO model
        :type results: list
        :param frame: Input image/frame to be annotated
        :type frame: ndarray
        :return: Annotated frame with bounding boxes
        :rtype: ndarray
        """
        # Iterate through the detection results
        for r in results:
            boxes = r.boxes
            # Process each detected bounding box
            for box in boxes:
                # Extract bounding box coordinates, confidence score, and class label
                xyxy = box.xyxy.cpu().detach().numpy().copy()[0]
                conf = box.conf.cpu().detach().numpy().copy()[0]
                cls = box.cls.cpu().detach().numpy().copy()[0]
                cls_name = r.names[cls]

                # Annotate the frame with the bounding box
                frame = self.plot_bbox(frame, xyxy, cls_name, conf)
                    
        return frame

    def predict(self, frame):
        """
        Predicts objects in the given frame using the loaded YOLO model.
        
        :param frame: Image/frame for object detection
        :type frame: ndarray
        :return: Detection results from the YOLO model
        :rtype: list
        """
        # Predict objects in the frame using the model
        results = self.model.predict(source=frame,
                                    verbose=True,
                                    show=False)
        return results

    def image_callback(self, data:Image):
        """
        Callback function for image data. Detects objects in the image, annotates it, and publishes the annotated image.
        
        :param data: Image message received from the topic
        :type data: Image
        """
        
        # Convert ROS image message to OpenCV format
        cv_img = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        # Predict objects in the image
        result = self.predict(cv_img)
        
        # Annotate the image with bounding boxes
        cv_img_with_bbox = self.update_detection(results=result, frame=cv_img)
        
        # Convert the annotated OpenCV image to ROS image message
        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img_with_bbox)
        
        # Publish the annotated image
        self.publish(img_msg)


def main():

    try:
        rclpy.init()
        running_node = StairDetectorNode()
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



