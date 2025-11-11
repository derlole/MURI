from muri_dev_interfaces.msg import PictureData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node

from vision.aruco_marker_detection import AMD

class ImageProcessing(Node):

    def __init__(self):
        super().__init__('image_processing')

        self.bridge = CvBridge()

        self.distance_in_meters, self.distance_in_milimeters, self.angle_in_rad, self.error = None, None, None, False
        self.error_counter = 0
        self.proc_AMD = AMD()

        self.subscription = self.create_subscription(
            Image,
            '/muri_image_raw',
            self.listener_callback,
            10)
        self.subscription

        self.get_logger().info('Subscription getätigt')
        
        self.publisher = self.create_publisher(
            PictureData,
            '/muri_picture_data',
            10)

    def listener_callback(self, msg):
        self.data = msg
        cv_raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info('Bild empfangen!')
        self.get_logger().info(f'data: {cv_raw_image[0][0]}')
        self.pic_to_data(cv_raw_image)
        pub_pic_data = PictureData()

        # PictureData.msg
            #std_msgs/Header header
            #bool error
            #float32 angle_in_rad
            #float32 distance_in_meters

        pub_pic_data.error = self.error
        pub_pic_data.angle_in_rad = float(self.angle_in_rad)
        pub_pic_data.distance_in_meters = float(self.distance_in_meters)

        self.publisher.publish(pub_pic_data)
        self.get_logger().info('OpenCV-Daten werden gepublished...')

    def pic_to_data(self, data_img):
        if data_img is None:
            self.get_logger().info('Kein Frame erhalten!')
            self.error_counter += 1

        if self.error_counter > 10:
            self.error = True

        self.distance_in_milimeters, self.angle_in_rad = self.proc_AMD.aruco_detection(data_img)
        self.distance_in_meters = self.distance_in_milimeters/1000


def main(args=None):
    rclpy.init(args=args)
    camera_read_out = ImageProcessing()
    rclpy.spin(camera_read_out)
    camera_read_out.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()