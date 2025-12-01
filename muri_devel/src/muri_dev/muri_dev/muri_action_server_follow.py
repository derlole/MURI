import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from muri_dev_interfaces.action import FOLLOW
from muri_dev_interfaces.msg import PictureData
from muri_logics.logic_action_server_drive import FollowLogic, FollowStates
from muri_logics.logic_interface import LogicInterface
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup 
import time

class FollowActionServer(Node):
    def __init__(self, logic: LogicInterface):
        super().__init__('muri_follow_action_server')

        self.follow_logic: LogicInterface = logic

        self._action_server = ActionServer(
            self,
            FOLLOW,
            'muri_follow',
            execute_callback = self.execute_callback,
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel',
             10
        )
        self.picture_data_sub = self.create_subscription(
            PictureData,
            '/muri_picture_data',  
            self.listener_callback_picture_data_asf,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  
            self.listener_callback_odom_asf,
            10
        )
        self._timer = self.create_timer(0.1, self.timer_callback_asf, MutuallyExclusiveCallbackGroup())
        self._goal_handle = None
        self._last_picture_data = None
        self._last_odom = None

    def timer_callback_asf(self):
        pass

    def execute_callback(self, goal_handle):
        pass

    def goal_callback(self, goal_request):
        pass

    def cancel_callback(self, goal_handle):
        pass

    def listener_callback_picture_data_asf(self, msg):
        self._last_picture_data = msg
        self.follow_logic.setCameraData(msg.angle_in_rad, msg.distance_in_meters, msg.dominant_aruco_id)

    def listener_callback_odom_asf(self, msg):
        self._last_odom = msg
        self.follow_logic.setOdomData(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation)

def main(args=None):
    rclpy.init(args=args)
    
    follow_action_server = FollowActionServer(FollowLogic())
    executor = MultiThreadedExecutor()
    executor.add_node(follow_action_server)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        follow_action_server.get_logger().info('Interrupt receivedat DriveActionServer, shutting down.')
    finally:
        follow_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':    
    main()