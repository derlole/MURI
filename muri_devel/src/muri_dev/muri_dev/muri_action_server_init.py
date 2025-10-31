import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from muri_dev_interfaces.action import INIT
from muri_logics.logic_action_server_init import InitLogic
from muri_logics.logic_interface import LogicInterface

class InitActionServer(Node):
    def __init__(self, logic: LogicInterface):
        super().__init__('muri_init_action_server')

        self.init_logic: LogicInterface = logic

        self._action_server = ActionServer(
            self,
            INIT,
            'muri_init',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel',
             10
        )
        self.picture_data_sub = self.create_subscription(
            bool,  # Hier echten typ angeben, sobald existiert
            '/muri_picture_data',  
            self.listener_callback_picture_data_asi,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  
            self.listener_callback_odom_asi,
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.goal_handler = None

    def timer_callback(self):
        pass

    def execute_callback(self, goal_handle):
        pass

    def goal_callback(self, goal_request):
        pass

    def cancel_callback(self, goal_handle):
        pass

    def listener_callback_odom_asi(self, msg):
        pass

    def listener_callback_picture_data_asi(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    init_action_server = InitActionServer(InitLogic())

    try:
        rclpy.spin(init_action_server)
    except KeyboardInterrupt:
        init_action_server.get_logger().info('Interrupt received at InitActionServer, shutting down.')
        # hier mayeb noch nen /cmd_vel auf 0
    finally:
        init_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()