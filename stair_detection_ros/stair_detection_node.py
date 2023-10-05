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
    def __init__(self):
        super().__init__("stair_detection_node") #Nodes name must be equal to the node
        self.get_logger().info("NODE is running") #nodes info

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


    def publish(self, img_msg):
        """

        """

        # publish image with bbox
        self.detection_img_pub.publish(img_msg)


    def load_model(self,model_path,device):
        """

        """
        # Load the specified model architecture
        model = YOLO(model_path).to(device)
        return model
    
    def update_detection(self, results, frame, frame_id):

        for r in results:
            boxes = r.boxes
            for box in boxes:
                
                xyxy = box.xyxy.cpu().detach().numpy().copy()[0]
                conf = box.conf.cpu().detach().numpy().copy()[0]
                cls = box.cls.cpu().detach().numpy().copy()[0]
                cls_name = r.names[cls]

                frame = self.plot_bbox(frame,xyxy,cls_name,conf)
                
        return frame

    def plot_bbox(self,frame,xyxy,cls_name,conf):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_thickness = 2

        ## colors BGR
        color = (0, 0, 255)  # red 
        text_color = (255, 255, 255)  # white color for the text
        background_color = (0, 0, 255)  # red color for the background of the text

        label = "{} | {:.2f}%".format(cls_name, conf * 100)

        cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color, 2)

        # Calculate width and height of the text box
        (w, h), _ = cv2.getTextSize(label, font, font_scale, font_thickness)
        
        # Draw a filled rectangle to put text inside
        cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1]-30)), (int(xyxy[0]+w), int(xyxy[1])), background_color, -1)
        
        # Put the text inside the rectangle
        cv2.putText(frame, label, (int(xyxy[0]), int(xyxy[1]-5)), font, font_scale, text_color, font_thickness, cv2.LINE_AA)

        return frame
    
    def predict(self,frame):
        results = self.model.predict(source=frame,
                                     verbose = True,
                                     show=False)
        
        return results
    
    def image_callback(self, data:Image):
        """
        """
        cv_img = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        result = self.predict(cv_img)
        cv_img_with_bbox = self.update_detection(results=result, frame=cv_img)

        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img_with_bbox)
        
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



