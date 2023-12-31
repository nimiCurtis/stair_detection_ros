#!/usr/bin/env python3

# Standard library imports
import os
import re

# Related third party imports
import cv2
from ultralytics import YOLO
from ultralytics.engine.results import Results

# ROS application/library specific imports
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node, QoSProfile
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose 

class StairDetectorNode(Node):
    """StairDetectorNode is a ROS2 node responsible for detecting stair ascending and descending 
    using the YOLO object detection model.
    
    Attributes:
        cv_bridge: A helper object to convert between ROS Image messages and OpenCV images.
        image_sub: Subscriber for image topics.
        detection_data_pub: Publisher for detected object data.
        detection_img_pub: Publisher for images with annotated detections.
        model: Loaded YOLO object detection model.
        conf: model confidence bootom threshold.
    
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
        # super().__init__()  
        super().__init__("stair_detection")  
        self.get_logger().info(f"****************************************")
        self.get_logger().info(f"      {self.get_namespace()}/{self.get_name()} is running     ")
        self.get_logger().info(f"****************************************")

        # Loading parameters
        params = self.init_params()
        
        # Initializing cd bridge
        self.cv_bridge = CvBridge()

        # Initializing publishers and subscribers
        imgSubQos = QoSProfile(depth=10)
        self.image_sub  = self.create_subscription(Image, 
                                                params.get('camera_topic'), 
                                                self.image_callback,
                                                imgSubQos)
        
        detQos = QoSProfile(depth=10)
        self.detection_data_pub = self.create_publisher(Detection2D,
                                                params.get('detection_topic_data'),
                                                detQos)

        imgPubQos = QoSProfile(depth=10)
        self.detection_img_pub = self.create_publisher(Image,
                                                params.get('detection_topic_img'),
                                                imgPubQos)

        # Loading detection model
        device, model_path, use_trt = params.get('device'), params.get('model_path'), params.get('use_trt')
        self.model = self.load_model(model_path,device,trt=use_trt)
        self.conf = params.get('conf')
        # Extract image size from model name if it exists , set 320 to default
        match = re.search(r'imgsz(\d+)', model_path)
        self.imgsz = int(match.group(1)) if match else 320
        
        self.is_detected = False

    def init_params(self):
        """
        Initialize the parameters for the node from ROS parameters.

        :return: Dictionary of parameters
        :rtype: dict
        """

        self.get_logger().info('************   Parameters   ************')
        
        camera_topic = self.declare_parameter('general.camera_topic',
                                            '/zedm/zed_node/rgb/image_rect_color')
        camera_topic = self.get_parameter('general.camera_topic').get_parameter_value().string_value
        self.get_logger().info('camera_topic: %s' % camera_topic)
        
        detection_topic_data = self.declare_parameter('general.detection_topic_data',
                                                    '/zion/stair_detection/detection')
        detection_topic_data = self.get_parameter('general.detection_topic_data').get_parameter_value().string_value
        self.get_logger().info('detection_topic_data: %s' % detection_topic_data)
        
        detection_topic_img = self.declare_parameter('general.detection_topic_img',
                                                    '/zion/stair_detection/detection_image')
        detection_topic_img = self.get_parameter('general.detection_topic_img').get_parameter_value().string_value
        self.get_logger().info('detection_topic_img: %s' % detection_topic_img)
        
        device = self.declare_parameter('model.device', 'cuda')
        device = self.get_parameter('model.device').get_parameter_value().string_value
        self.get_logger().info('device: %s' % device)
        
        model_name = self.declare_parameter('model.model_name', 'best')
        model_name = self.get_parameter('model.model_name').get_parameter_value().string_value
        
        use_trt = self.declare_parameter('model.trt', False)
        use_trt = self.get_parameter('model.trt').get_parameter_value().bool_value
        self.get_logger().info('trt: %s' % use_trt)


        model_name = model_name +'.engine' if use_trt else model_name +'.pt' 
        model_path = os.path.join(
            get_package_share_directory('stair_detection_ros'),
            'models',
            model_name
        )
        self.get_logger().info('model_path: %s' % model_path)

        conf = self.declare_parameter('model.conf', 0.75)
        conf = self.get_parameter('model.conf').get_parameter_value().double_value
        self.get_logger().info('confidence threshold: %s' % conf)
        
        params_dic = {
            'camera_topic': camera_topic,
            'detection_topic_data': detection_topic_data,
            'detection_topic_img': detection_topic_img,
            'device': device,
            'model_path': model_path,
            'conf': conf,
            'use_trt': use_trt
        }

        self.get_logger().info(f"****************************************")

        return params_dic

    def load_model(self, model_path:String, device:String, trt:bool=True)->YOLO:
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
        model = YOLO(model_path, task='detect') 
        
        # if not tensorrt -> move model to device
        if not trt:
            model.to(device)
            
        return model

    def image_callback(self, data:Image):
        """
        Callback function for image data. Detects objects in the image, annotates it, and publishes the annotated image.
        
        :param data: Image message received from the topic
        :type data: Image
        """
        
        # Convert ROS image message to OpenCV format
        cv_img = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        # Predict objects in the image
        result = self.predict(cv_img, imgsz=self.imgsz, conf=self.conf)

        # Get the detection data
        conf, cls_id, cls_name, xyxy = self.get_detection(results=result)
        
        self.is_detected = True if cls_id is not None else False
        
        # Annotate the image with bounding boxes
        if self.is_detected:
            cv_img_with_bbox = self.plot_bbox(cv_img, xyxy, cls_name, conf)
        else:
            cv_img_with_bbox = cv_img
        
        # Convert the annotated OpenCV image to ROS image message and to Detection2D message
        header = Header()
        header.frame_id = data.header.frame_id
        header.stamp = self.get_clock().now().to_msg()
        
        img_msg = self.cv_bridge.cv2_to_imgmsg(cv_img_with_bbox)
        detection_msg = self.set_detection2d_msg(data,conf, cls_name, xyxy, header=header)
        
        # Publish the annotated image adn the detection data
        self.publish(img_msg,detection_msg)

    def predict(self, frame, imgsz, conf=0.75)->Results:
        """
        Predicts objects in the given frame using the loaded YOLO model.
        
        :param frame: image/frame for object detection
        :type frame: ndarray
        :param imgsz: image size 
        :type imgsz: int
        :param conf: prediction confidence threshold
        :type conf: float
        :return: Detection result from the YOLO model
        :rtype: Results object
        """
        # Predict objects in the frame using the model
        results = self.model.predict(source=frame,
                                    imgsz=[imgsz,imgsz],
                                    verbose=False,
                                    show=False,
                                    max_det=1,
                                    #iou
                                    conf=conf)
        return results[0]

    def get_detection(self, results:Results)->tuple:
        """
        Returns the confidence, class name, and the bounding box coordinates 
        of the box with the highest confidence from the detection results.

        :param results: Detection results from the YOLO model
        :type results: Results object
        :param frame: Input image/frame (not used in this modified function but kept for consistency)
        :type frame: ndarray
        :return: conf, cls_name, xyxy of the box with the highest confidence
        :rtype: tuple
        """

        max_conf = 0.  # Initialize with a very low value
        best_cls = None
        best_cls_name = None
        best_xyxy = None
        boxes = results.boxes
        
        # Process each detected bounding box
        for box in boxes:
            # Extract bounding box coordinates, confidence score, and class label
            xyxy = box.xyxy.cpu().detach().numpy().copy()[0]
            conf = box.conf.cpu().detach().numpy().copy()[0]
            cls = box.cls.cpu().detach().numpy().copy()[0]
            cls_name = results.names[cls]

            # Check if the current box's confidence is the highest so far
            if conf > max_conf:
                max_conf = conf
                best_cls = cls
                best_cls_name = cls_name
                best_xyxy = xyxy

        return max_conf, best_cls, best_cls_name, best_xyxy

    def plot_bbox(self,frame,xyxy,cls_name,conf):
        """
        Plot a bounding box of detection on an image
        
        :param frame: input frame to be annotated
        :type results: ndarray
        :param xyxy: coordinates of the bounding box
        :type xyxy: ndarray
        :param cls_name: Label of the detection
        :type cls_name: string
        :param conf: Confidence percentage of the detection
        :type conf: float
        :return: Annotated frame with bounding boxes
        :rtype: ndarray
        """
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.75
        font_thickness = 1

        ## colors BGR
        color = (0, 0, 255)  # red 
        text_color = (255, 255, 255)  # white color for the text
        background_color = (0, 0, 255)  # red color for the background of the text

        label = "{} | {:.2f}%".format(cls_name, conf * 100)

        cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])),
                    (int(xyxy[2]), int(xyxy[3])), color, 2)

        # Calculate width and height of the text box
        (w, h), _ = cv2.getTextSize(label, font, font_scale, font_thickness)
        
        # Draw a filled rectangle to put text inside
        cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1]-30)),
                    (int(xyxy[0]+w), int(xyxy[1])), background_color, -1)
        
        # Put the text inside the rectangle
        cv2.putText(frame, label, (int(xyxy[0]), int(xyxy[1]-5)), font,
                    font_scale, text_color, font_thickness, cv2.LINE_AA)

        return frame

    def set_detection2d_msg(self, image, conf=0., cls_id=None, xyxy=None, header=None)->Detection2D:
        """
        Constructs a Detection2D message using the given detection parameters.
        
        :param conf: Confidence score of the detected object
        :type conf: float
        :param cls_name: Name/ID of the detected class
        :type cls_name: str or int
        :param xyxy: Bounding box coordinates in the format [x1, y1, x2, y2]
        :type xyxy: list or ndarray
        :param header: Header information (e.g., timestamp, frame_id) for the message
        :type header: Header or similar datatype
        :return: Constructed Detection2D message with the given detection parameters
        :rtype: Detection2D
        """
        
        # Initialize a Detection2D object
        detection = Detection2D()
        # Set the header information
        if header is not None: detection.header = header 

        if self.is_detected:
            
            # Calculate and set the bounding box dimensions
            detection.bbox.size_x = float(abs(xyxy[0] - xyxy[2]))
            detection.bbox.size_y = float(abs(xyxy[1] - xyxy[3]))
            detection.bbox.center.x = float(xyxy[0] + abs(xyxy[0] - xyxy[2])/2)
            detection.bbox.center.y = float(xyxy[1] + abs(xyxy[1] - xyxy[3])/2)
        
            # Initialize an ObjectHypothesis object to store detected class and its confidence
            hypo = ObjectHypothesisWithPose()
            hypo.id = str(cls_id)
            hypo.score = float(conf)
            # Add the hypothesis to the Detection2D message
            detection.results.append(hypo)
            detection.source_img = image

        return detection

    def publish(self, img_msg:Image, det_msg:Detection2D()):
        """
        Publishes the image message containing detected objects with bounding boxes and the detection2d data.
        
        :param img_msg: Image message containing detections
        :type img_msg: Image
        :param det_msg: Detection2D message containing detection data
        :type det_msg: Detection2D
        """
        
        self.detection_img_pub.publish(img_msg)
        self.detection_data_pub.publish(det_msg)

def main(args=None):

    # Initializing Node
    rclpy.init(args=args)
    detect_node = StairDetectorNode()

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


