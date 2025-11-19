# debug_aruco_node.py
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

MARKER_SIZE = 175.0

CAMERA_MATRIX = np.array([
    [485.98256352079204, 0.0, 326.8156596599855],
    [0.0, 481.9114474146188, 231.52240485863123],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

DIST_COEFFS = np.array([
    [-0.28206812200227427],
    [ 2.61448314076083],
    [ 0.020889912770543827],
    [ 0.017394594564323452],
    [-7.331368052181177]
], dtype=np.float32)

ARUCO_DICT = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
ARUCO_PARAMS = cv.aruco.DetectorParameters()
DETECTOR = cv.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

class DebugArucoNode(Node):
    def __init__(self):
        super().__init__('debug_aruco_node')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/muri_image_raw',
            self.image_callback,
            10)

        #self.pub = self.create_publisher(Image, '/debug_aruco_image', 10)

        self.get_logger().info('Debug‑Aruco‑Node gestartet, abonniert /muri_image_raw')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Bildkonvertierung fehlgeschlagen: {e}')
            return
        
        corners, ids, _ = DETECTOR.detectMarkers(cv_image)

        if ids is not None and len(corners) > 0:
            for i in range(len(ids)):
                
                obj_pts = np.array([
                    [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                    [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                    [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
                    [-MARKER_SIZE/2, -MARKER_SIZE/2, 0]
                ], dtype=np.float32)

                ok, rvec, tvec = cv.solvePnP(
                    obj_pts,
                    corners[i],
                    CAMERA_MATRIX,
                    DIST_COEFFS,
                    flags=cv.SOLVEPNP_ITERATIVE)

                if ok:
                    cv.drawFrameAxes(cv_image, CAMERA_MATRIX, DIST_COEFFS,
                                     rvec, tvec, MARKER_SIZE/2)

            cv.aruco.drawDetectedMarkers(cv_image, corners, ids)


        cv.imshow('Debug – ArUco Tracking', cv_image)
        cv.waitKey(1)   

def main(args=None):
    rclpy.init(args=args)
    node = DebugArucoNode()
    rclpy.spin(node)

    # Aufräumen
    cv.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()