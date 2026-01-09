from muri_dev_interfaces.msg import PictureData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from vision.aruco_marker_detection import AMD

class ImageProcessing(Node):
    """ROS2-Node für ArUco-Marker-Detektion und Bildverarbeitung.
    
    Empfängt Grayscale-Bilder vom CameraReadOut-Node über '/muri_image_raw',
    führt ArUco-Marker-Detektion durch und publiziert verarbeitete Daten
    (Distanz, Winkel, Marker-ID, Fehler) auf '/muri_picture_data'.
    
    Komponenten:
    - Subscriber: '/muri_image_raw' (sensor_msgs/Image, Queue=10)
    - Publisher: '/muri_picture_data' (muri_dev_interfaces/PictureData, Queue=10)
    - AMD-Detektor: ArUco Marker Detection für Bildverarbeitung
    - Buffer: 3-Wert-Filter für Distanzstabilität
    - Fehlerüberwachung: error=True nach >10 konsekutiven Frame-Verlusten
    """
    def __init__(self):
        """Initialisiert den Image-Processing-Node und AMD-Detektor.
        
        Setzt auf:
        - ROS2-Subscriber für '/muri_image_raw' (sensor_msgs/Image)
        - ROS2-Publisher für '/muri_picture_data' (PictureData)
        - AMD-Detektor für ArUco-Marker-Erkennung
        - Distanz-Buffer mit 3 Werten für Last-Valid-Value-Filter
        - Error-Counter mit Schwellwert von 10 Frames
        """
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
        """Callback für eingehende Kamerabilder, führt Detektion und Publikation durch.
        
        Ablauf:
        1. ROS Image-Message zu OpenCV-Array konvertieren (Grayscale, mono8)
        2. pic_to_data() aufrufen (ArUco-Detektion und Filterung)
        3. PictureData-Message erstellen und mit Ergebnissen befüllen
        4. Message auf '/muri_picture_data' publishen
        
        Args:
            msg (sensor_msgs.msg.Image): Eingehendes Grayscale-Bild
        
        Fehlerbehandlung:
        - Exception bei Bildkonvertierung wird geloggt, aber fortgesetzt
        - Gefilterte Distanz wird immer publiziert (auch bei Fehler)
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
        """Führt ArUco-Detektion durch, konvertiert Einheiten und filtert Distanz.
        
        Ablauf:
        1. Fehlerüberwachung: Bei data_img=None Error-Counter inkrementieren
        2. Error-Flag setzen wenn Counter > 10 (permanente Fehler erkennen)
        3. AMD.aruco_detection() aufrufen (gibt mm, rad, ID zurück)
        4. Einheitenkonvertierung: Millimeter → Meter (/ 1000)
        5. filter_distance() aufrufen (3-Wert-Buffer-Filter)
        
        Args:
            data_img (numpy.ndarray | None): OpenCV-Grayscale-Bild oder None
        
        Fehlerbehandlung:
        - Auch bei data_img=None wird aruco_detection() aufgerufen
        - Buffer-Filter sorgt für Stabilität bei kurzzeitigen Ausfällen
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
        """Buffer-Filter (Last-Valid-Value-Filter) für Distanzstabilität.
        
        Ablauf:
        1. Buffer-Shift: Werte wandern von neu nach alt (first→second→third)
        2. Finde neuesten gültigen Wert: Iteriere von neu nach alt
        3. Rückgabe: Ersten Wert ≠ -1.0, oder -1.0 falls alle ungültig
        
        Returns:
            float: Neueste gültige gefilterte Distanz [m] oder -1.0
        
        Fehlerbehandlung:
        - Stabilisierung bei kurzzeitigen Detektionsfehlern (Lichtverhältnisse)
        - Verhindert abrupte Distanzsprünge in der Robotersteuerung
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