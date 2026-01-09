from muri_dev_interfaces.msg import PictureData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from vision.aruco_marker_detection import AMD

class ImageProcessing(Node):
    """ROS2-Node: Detektiert ArUco-Marker, filtert Distanz, publiziert Ergebnisse auf '/muri_picture_data'."""
    def __init__(self):
        """Initialisiert ROS2-Subscriber/Publisher, AMD-Detektor, Distanz-Buffer und Error-Counter."""
        super().__init__('image_processing')

        self.bridge = CvBridge()

        self.distance_in_meters_unfiltered = None
        self.distance_in_meters_filtered = None
        self.distance_in_milimeters = None
        self.angle_in_rad = None
        self.marker_id = 9999
        self.error = False
        self.error_counter = 0
        self.first_data = -1.0
        self.second_data = -1.0
        self.third_data = -1.0
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
        """ROS2-Callback: Konvertiert Image-Message, führt Detektion durch, publiziert PictureData.
        
        Args:
            msg (sensor_msgs/Image): Eingehendes Grayscale-Bild von '/muri_image_raw'
        """
        try:
            cv_raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error("Fehler bei der Bildübertragung")
        #self.get_logger().info('Bild empfangen!')
        self.pic_to_data(cv_raw_image)
        pub_pic_data = PictureData()

        pub_pic_data.error = self.error
        pub_pic_data.angle_in_rad = float(self.angle_in_rad)
        pub_pic_data.distance_in_meters = float(self.distance_in_meters_filtered)  # gefilterte Distanz verwenden
        pub_pic_data.dominant_aruco_id = int(self.marker_id)

        self.publisher.publish(pub_pic_data)
        #self.get_logger().info('OpenCV-Daten wurden gepublished')

    def pic_to_data(self, data_img):
        """Ruft AMD auf, konvertiert mm→m, filtert Distanz mit 3-Wert-Buffer.
        
        Args:
            data_img (numpy.ndarray | None): OpenCV-Grayscale-Bild von Bridge oder None
        """
        if data_img is None:
            self.get_logger().info('Kein Frame erhalten!')
            self.error_counter += 1

        if self.error_counter > 10:
            self.error = True

        self.distance_in_milimeters, self.angle_in_rad, self.marker_id = self.proc_AMD.aruco_detection(data_img)
        self.distance_in_meters_unfiltered = self.distance_in_milimeters/1000
        self.distance_in_meters_filtered = self.filter_distance()

    def filter_distance(self):
        """Last-Valid-Value-Filter: Gibt neuesten gültigen Distanzwert aus 3-Wert-Buffer.
        
        Returns:
            float: Gefilterte Distanz in Metern oder -1.0 wenn alle ungültig
        """
        # 1. Buffer aktualisieren
        self.third_data  = self.second_data
        self.second_data = self.first_data
        self.first_data  = self.distance_in_meters_unfiltered

        # 2. Das neuste gültige Ergebnis zurückgeben
        for v in (self.first_data, self.second_data, self.third_data):
            if v != -1.0:          # gibt den neuesten Wert zurück, ungleich null
                return v
        return -1.0                 # alle drei == -1.0

def main(args=None):
    rclpy.init(args=args)
    camera_read_out = ImageProcessing()
    rclpy.spin(camera_read_out)
    camera_read_out.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()