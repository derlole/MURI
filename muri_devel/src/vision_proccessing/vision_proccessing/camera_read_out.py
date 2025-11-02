from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rlcpy.node import Node

class CameraReadOut(Node):
    def __init__(self):
        super().__init__('camera_read_out')
        self.publisher = self.create_publisher(Image, '/muri_image_raw', 10)
        timer_time = 0.1
        self.data = self.create_timer(timer_time, self.timer_callback)



    def timer_callback(self):
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(self.read_camera(), encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info('Bild wird verschickt...')

    def read_camera(self):

        path_camera = 0 # '/dev/video0' 

        img = cv.VideoCapture(path_camera)
        success, frame = img.read()
        if not success:
            self.get_logger().info('Bild konnte nicht gelesen werden')
            return None
        return frame
    
def main(args=None):
    rclpy.init(args=args)
    camera_read_out = CameraReadOut()
    rclpy.spin(camera_read_out)
    camera_read_out.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()