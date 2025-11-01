import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from muri_dev_interfaces.action import TURN
from muri_logics.logic_action_server_turn import TurnLogic
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
            cancel_callback=self.cancel_callback
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
            bool,  # TODO Hier echten typ angeben, sobald existiert
            '/muri_picture_data',  
            self.listener_callback_picture_data_ast,
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback_ast)
        self.goal_handler = None

    def timer_callback_ast(self):
        pass

    def execute_callback(self, goal_handle):
        pass

    def goal_callback(self, goal_request):
        pass

    def cancel_callback(self, goal_handle):
        pass

    def listener_callback_odom_ast(self, msg):
        pass

    def listener_callback_picture_data_ast(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    turn_action_server = TurnActionServer(TurnLogic())
    try:
        rclpy.spin(turn_action_server)
    except KeyboardInterrupt:
        turn_action_server.get_logger().info('Interrupt received at TurnActionServer, shutting down.')
        # hier mayeb noch nen /cmd_vel auf 0
    finally:
        turn_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()