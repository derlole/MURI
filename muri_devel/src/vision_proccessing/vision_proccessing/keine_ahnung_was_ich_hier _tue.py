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

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/muri_image_raw',
            self.listener_callback,
            10)
        self.subscription
        self.publisher = self.create_publisher(
            PictureData,
            '/muri_picture_data',
            10)

    def listener_calback(self, msg):
        self.data = msg
        cv_raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info('Bild empfangen!')
        final_data = self.krasseBerechnungen(cv_raw_image)
        pub_pic_data = PictureData()

        # TODO Daten in pub_pic_data füllen 
        pub_pic_data.success = bool(None)
        pub_pic_data.excenter = float(None)
        # weitere Daten folgen

        self.publisher.publish(pub_pic_data)
        self.get_logger().info('OpenCV-Daten werden gepublished...')

    def krasseBerechnungen(self, data_img):
        pass
