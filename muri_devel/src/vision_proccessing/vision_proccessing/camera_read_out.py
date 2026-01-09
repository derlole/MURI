from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node

class CameraReadOut(Node):
    """ROS2-Node für kontinuierliche Kamerabilderfassung.
    
    Erfasst Kamerabilder (30 Hz) von /dev/video0 und publiziert sie als
    Grayscale-Images auf '/muri_image_raw' für Bildverarbeitungsprozesse.
    
    Komponenten:
    - Publisher: '/muri_image_raw' (sensor_msgs/Image, Queue=10)
    - Timer: 30 Hz (1/30 Sekunden) für periodische Bilderfassung
    - Kamera: /dev/video0 via OpenCV VideoCapture
    - Encoding: mono8 (Grayscale, 66% weniger Daten als RGB)
    - Buffer: Size=1 zur Vermeidung veralteter Frames
    """
    def __init__(self):
        """Initialisiert den Kamera-Node mit Publisher und Timer.
        
        Setzt auf:
        - ROS2-Publisher für '/muri_image_raw' (sensor_msgs/Image)
        - Timer mit 30 Hz Frequenz
        - Kamera-Device /dev/video0
        - OpenCV-Buffer minimiert auf Größe 1
        """
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
        """Wird von Timer alle 1/30 Sekunden aufgerufen (30 Hz).
        
        Ablauf:
        1. CvBridge-Instanz erstellen (OpenCV ↔ ROS-Konvertierung)
        2. Bild lesen via read_camera() und in Image-Message konvertieren
        3. Message mit Encoding 'mono8' publishen auf '/muri_image_raw'
        
        Exceptions:
            AttributeError: None-Wert von read_camera() ist ROS2-inkompatibel
        """
        try:
            bridge = CvBridge()
            msg = bridge.cv2_to_imgmsg(self.read_camera(), encoding='mono8')
            self.publisher.publish(msg)
            #self.get_logger().info('Bild wird verschickt...')
        except AttributeError as e:
            self.get_logger().error("Falscher Wert wurde versucht über die Bridge zu senden")

    def read_camera(self):
        """Liest Frame von Kamera und konvertiert zu Grayscale.
        
        Ablauf:
        1. Frame via cv.VideoCapture.read() erfassen
        2. Bei Fehler (success=False): Error-Log und None zurückgeben
        3. BGR → Grayscale konvertieren (cv.COLOR_BGR2GRAY)
        
        Returns:
            numpy.ndarray: Grayscale-Bild (shape=(height,width), dtype=uint8)
            None: bei Lesefehler (Kamera nicht verfügbar/Verbindung verloren)
        
        Fehlerbehandlung:
        - Kamera-Fehler werden geloggt, None wird zurückgegeben
        - None führt zu AttributeError im timer_callback()
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