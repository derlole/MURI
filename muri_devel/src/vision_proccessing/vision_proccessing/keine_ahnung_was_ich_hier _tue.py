from muri_dev_interfaces.msg import PictureData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node

class ImageProcessing(Node):

    def __init__(self):
        super().__init__('image_processing')

        self.cv_raw_image = None
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/muri_image_raw',
            self.listener_callback,
            10)
        self.subscription
        self.publisher = self.create_publisher(
            PictureData,
            '/muri_image_raw',
            10) 
        timer_time = 0.1  
        self.data = self.create_timer(timer_time, self.timer_callback)
        
    def timer_callback(self):
        pass

    def listener_calback(self, msg):
        self.data = msg
        self.cv_raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info('Bild empfangen!')

    def krasseBerechnungen():
        pass
