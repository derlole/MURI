import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from muri_dev_interfaces.action import TURN
from muri_dev_interfaces.msg import PictureData
from muri_logics.logic_action_server_turn import TurnLogic, TurnStates
from muri_logics.logic_interface import LogicInterface

class TurnActionServer(Node):
    def __init__(self, locic: LogicInterface):
        super().__init__('muri_turn_action_server')

        self.turn_logic: LogicInterface = locic

        self._action_server = ActionServer(
            self,
            TURN,
            'muri_turn',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_acc_callback
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel',
             10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  
            self.listener_callback_odom_ast,
            10
        )
        self.picture_data_sub = self.create_subscription(
            PictureData,
            '/muri_picture_data',  
            self.listener_callback_picture_data_ast,
            10
        )
        self._goal_handle = None
        self._last_picture_data = None
        self._last_odom = None
        self._timer = self.create_timer(0.1, self.timer_callback_ast)

    def timer_callback_ast(self):
        if self._goal_handle is None or not self._goal_handle.is_active:
            return
        
        if self._goal_handle.is_cancel_requested:
            self.get_logger().info('Canc: turn-goal.')
            self._goal_handle.canceled()

            self._goal_result = TURN.Result()
            self._goal_result.success = False
            self._goal_exiting = True

            return 
        
        self.turn_logic.state_machine()
        out = self.turn_logic.getOut()

        if not out.outValid():
            out.resetOut()
            return
        
        cmd_vel = Twist()
        cmd_vel.linear.x = float(out.values['linear_velocity_x'])
        cmd_vel.linear.y = float(out.values['linear_velocity_y'])
        cmd_vel.angular.z = float(out.values['angular_velocity_z'])
        self.cmd_vel_pub.publish(cmd_vel)

        feedback_msg = TURN.Feedback()
        feedback_msg.moved_angle = float(out.values['turned_angle'])

        self._goal_handle.publish_feedback(feedback_msg)

        if self.turn_logic.getActiveState() == TurnStates.SUCCESS:
            self.get_logger().info('succ: turn-goal.')
            self._goal_handle.succeed()

            self._goal_result = TURN.Result()
            self._goal_result.success = True
            self._goal_exiting = True

            self._goal_handle = None

        elif self.turn_logic.getActiveState() == TurnStates.FAILED:
            self.get_logger().info('fail: turn-goal.')
            self._goal_handle.abort()

            self._goal_result = TURN.Result()
            self._goal_result.success = False
            self._goal_exiting = True

            self._goal_handle = None

        out.resetOut()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Exec: turn-goal')
        
        self._goal_exiting = False
        self._goal_result = None

        while not self._goal_exiting:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self._goal_result

    def goal_callback(self, goal_request):
        self.get_logger().info('Rec: turn-goal')

        if self._goal_handle is not None and self._goal_handle.is_active:
            self.get_logger().info('Rej: turn-goal')
            return GoalResponse.REJECT
        
        self.get_logger().info('Acc: turn-goal')
        return GoalResponse.ACCEPT

    def handle_acc_callback(self, goal_handle):
        self._goal_handle = goal_handle
        self.turn_logic.reset()
        self.turn_logic.setActive()
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Rec: cancel turn-goal')
        return CancelResponse.ACCEPT

    def listener_callback_odom_ast(self, msg):
        self._last_odom = msg
        self.turn_logic.setOdomData(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation)

    def listener_callback_picture_data_ast(self, msg):
        self._last_picture_data = msg
        self.turn_logic.setCameraData(msg.angle_in_rad, msg.distance_in_meters)
def main(args=None):
    rclpy.init(args=args)

    turn_action_server = TurnActionServer(TurnLogic())
    try:
        rclpy.spin(turn_action_server)
    except (KeyboardInterrupt, ExternalShutdownException):
        turn_action_server.get_logger().info('Interrupt received at TurnActionServer, shutting down.')
        # cmd_vel wird vom main controller auf null gesetzt
    finally:
        turn_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()