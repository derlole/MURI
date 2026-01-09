from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node

class CameraReadOut(Node):
    """ROS2-Node: Erfasst Kamerabilder (30 Hz) und publiziert sie als Grayscale auf '/muri_image_raw'."""
    def __init__(self):
        """Initialisiert ROS2-Publisher, Timer (30 Hz) und Kamera (/dev/video0)."""
        super().__init__('camera_read_out')
        self.publisher = self.create_publisher(Image, '/muri_image_raw', 10)
        timer_time = 1/30   # sek

        path_camera = 0     # /dev/video0 

        try:
            self.img = cv.VideoCapture(path_camera)
        except Exception as e:
            self.get_logger().error(f'Fehler beim Initialisieren der Kamera: {str(e)}')
            raise e
        
        self.img.set(cv.CAP_PROP_BUFFERSIZE, 1)
        
        self.data = self.create_timer(timer_time, self.timer_callback)

        self.get_logger().info('CameraReadOut-Node gestartet')

    def timer_callback(self):
        """Wird von Timer aufgerufen (30 Hz): Erfasst Frame, konvertiert zu Grayscale, publiziert.
        
        Exceptions:
            AttributeError: Wenn read_camera() None zurückgibt (ROS2-inkompatibel)
        """
        try:
            bridge = CvBridge()
            msg = bridge.cv2_to_imgmsg(self.read_camera(), encoding='mono8')
            self.publisher.publish(msg)
            #self.get_logger().info('Bild wird verschickt...')
        except AttributeError as e:
            self.get_logger().error("Falscher Wert wurde versucht über die Bridge zu senden")

    def read_camera(self):
        """Liest Frame von Kamera und konvertiert BGR → Grayscale.
        
        Returns:
            numpy.ndarray: Grayscale-Bild (shape=(h,w), dtype=uint8) oder None bei Fehler
        """
        success, frame = self.img.read()

        if not success:
            self.get_logger().error('Bild konnte nicht gelesen werden')
            return None
        
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        return frame_gray
    
def main(args=None):
    rclpy.init(args=args)
    camera_read_out = CameraReadOut()
    rclpy.spin(camera_read_out)
    camera_read_out.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()