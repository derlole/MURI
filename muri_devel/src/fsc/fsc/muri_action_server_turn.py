import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from muri_dev_interfaces.action import TURN
from muri_dev_interfaces.msg import PictureData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from muri_logics.logic_action_server_turn import TurnLogic
from muri_logics.logic_interface import LogicInterface

class TurnActionServer(Node):
    
    def __init__(self, logic: LogicInterface):
        super().__init__('muri_turn_action_server')
        self._action_server = ActionServer(
            self,
            TURN,
            'muri_turn',
            self.execute_callback
        )
        self.goal_handler = None

        self.publishers_ =self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback_odom_data_ast,
            10
        )
        self.picture_data_sub = self.create_subscription(
            PictureData,
            '/muri_picture_data',
            self.listener_callback_picture_data_ast,
            10
        )
        self.turn_logic = logic 
        self._last_picture_data = None
        self._last_odom = None
        self.create_timer(0.1, self.timer_callback)

    def count_to_ten(self):
        self.get_logger().info('Counting to ten...')
        if not hasattr(self, '_count'):
            self._count = 0
        self._count += 1
        self.get_logger().info(f'Count: {self._count}')
        if self._count >= 10:
            self._count = 0
            return True
        return False

    def timer_callback(self):
        done = False
        if self.goal_handler is not None:
            done = self.count_to_ten()

            feedback = TURN.Feedback()
            feedback.moved_angle = float(self._count)
            self.goal_handler.publish_feedback(feedback)


        if done:
            result = TURN.Result()
            result.success = True
            self.goal_handler.succeed()

            self._goal_finished = True
            self._goal_result = result
            self.goal_handler = None

    def execute_callback(self, goal_handle):
        self.get_logger().info('Exec: turn-goal')
        self.goal_handler = goal_handle

        self._goal_finished = False
        self._goal_result = None

        while not self._goal_finished:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self._goal_result

    def listener_callback(self, msg: Odometry):
        self.get_logger().info(f"Pos X: {msg.pose.pose.position.x:.3f}")

    def listener_callback_odom_data_ast(self, msg: Odometry):
        self._last_odom = msg

    def listener_callback_picture_data_ast(self, msg: PictureData):
        self._last_picture_data = msg
       

def main(args = None):
    rclpy.init(args=args)

    ac = TurnActionServer(logic=TurnLogic())
    
    rclpy.spin(ac)
    rclpy.shutdown() 
    
if __name__ == '__main__':
    main()
    