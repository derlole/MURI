from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node

class CameraReadOut(Node):
    """
    ROS 2 node that streams raw camera frames to `/muri_image_raw`.
    """
    def __init__(self):
        """
        Initialise the node and its publisher.  A timer is created
        that triggers every x * seconds to capture a frame and send it
        downstream.
        """
        super().__init__('camera_read_out')
        self.publisher = self.create_publisher(Image, '/muri_image_raw', 10)
        timer_time = 1/30 # sek

        path_camera = 0 # '/dev/video0' 

        self.img = cv.VideoCapture(path_camera)
        self.img.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
        self.img.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
        self.img.set(cv.CAP_PROP_BUFFERSIZE, 1)
        
        self.data = self.create_timer(timer_time, self.timer_callback)

        self.get_logger().info('CameraReadOut-Node gestartet')

    def timer_callback(self):
        """
        Timer callback that captures a frame from the camera and publishes it.
        """
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(self.read_camera(), encoding='mono8')
        self.publisher.publish(msg)
        #self.get_logger().info('Bild wird verschickt...')

    def read_camera(self):
        """
        Read a single frame from the default video device.
        """

        success, frame = self.img.read()

        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        if not success:
            self.get_logger().info('Bild konnte nicht gelesen werden')
            return None
        return frame_gray
    
def main(args=None):
    rclpy.init(args=args)
    camera_read_out = CameraReadOut()
    rclpy.spin(camera_read_out)
    camera_read_out.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()