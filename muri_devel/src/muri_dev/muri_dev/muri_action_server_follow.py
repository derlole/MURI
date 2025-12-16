import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from muri_dev_interfaces.action import FOLLOW
from muri_dev_interfaces.msg import PictureData
from muri_logics.logic_action_server_follow import FollowLogic, FollowStates
from muri_logics.logic_interface import ExtendedLogicInterface
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup 
import time

class FollowActionServer(Node):
    def __init__(self, logic: ExtendedLogicInterface):
        super().__init__('muri_follow_action_server')

        self.follow_logic: ExtendedLogicInterface = logic

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
        self.schpieth_sup = self.create_subscription(
            Float32,
            '/schpieth_supervision',
            self.listener_callback_schpieth_asf,
            10
        )
        self._timer = self.create_timer(0.1, self.timer_callback_asf, MutuallyExclusiveCallbackGroup())
        self._goal_handle = None
        self._last_picture_data = None
        self._last_odom = None

    def timer_callback_asf(self):
        # print(str(self._goal_handle is None or not self._goal_handle.is_active))
        if self._goal_handle is None or not self._goal_handle.is_active:
            return
        
        if self._goal_handle.is_cancel_requested:
            self.get_logger().info('Canc: follow-goal.')
            self._goal_handle.canceled()

            self._goal_result = FOLLOW.Result()
            self._goal_result.success = False
            self._goal_exiting = True
            self._goal_handle = None
            return
        # print("exec state_machine")
        self.follow_logic.state_machine()
        out = self.follow_logic.getOut()

        if not out.outValid():
            out.resetOut()
            return
        
        cmd_vel = Twist()
        cmd_vel.linear.x = float(out.values['linear_velocity_x'])
        cmd_vel.linear.y = float(out.values['linear_velocity_y'])
        cmd_vel.angular.z = float(out.values['angular_velocity_z'])
        # print(str(cmd_vel))

        self.cmd_vel_pub.publish(cmd_vel)

        feedback_msg = FOLLOW.Feedback()
        feedback_msg.distance_to_target = float(out.values['distance_to_robot'])

        self._goal_handle.publish_feedback(feedback_msg)

        if self.follow_logic.getActiveState() == FollowStates.SUCCESS:
            self.get_logger().info('Succ: follow-goal.')
            self._goal_handle.succeed()

            self._goal_result = FOLLOW.Result()
            self._goal_result.success = True
            self._goal_exiting = True
 
            self._goal_handle = None

        elif self.follow_logic.getActiveState() == FollowStates.FAILED:
            self.get_logger().info('Fail: follow-goal.')
            self._goal_handle.abort()

            self._goal_result = FOLLOW.Result()
            self._goal_result.success = False
            self._goal_exiting = True

            self._goal_handle = None

        out.resetOut()


    def execute_callback(self, goal_handle):
        self.get_logger().info('Exec: follow-goal.')

        self._goal_handle = goal_handle

        self._goal_exiting = False
        self._goal_result = None

        while not self._goal_exiting:
            time.sleep(0.05)

        return self._goal_result

    def goal_callback(self, goal_request):
        self.get_logger().info('Rec: follow-goal.')

        if self._goal_handle is not None and self._goal_handle.is_active:
            self.get_logger().info('Rej: follow-goal, already active.')
            return GoalResponse.REJECT
        
        self.get_logger().info('Acc: follow-goal.')

        self.follow_logic.reset()
        self.follow_logic.setActive()

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Rec: cancel follow-goal.')
        return CancelResponse.ACCEPT

    def listener_callback_picture_data_asf(self, msg):
        self._last_picture_data = msg
        self.follow_logic.setCameraData(msg.angle_in_rad, msg.distance_in_meters)
        self.follow_logic.setArucoData(msg.dominant_aruco_id)

    def listener_callback_odom_asf(self, msg):
        self._last_odom = msg
        self.follow_logic.setOdomData(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation)
        
    def listener_callback_schpieth_asf(self, msg):
        value = max(0.0, min(msg.data, 0.2)) # TODO constants
        self.follow_logic.setSchpieth(value)

def main(args=None):
    rclpy.init(args=args)
    
    follow_action_server = FollowActionServer(FollowLogic())
    executor = MultiThreadedExecutor(num_threads=4)
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