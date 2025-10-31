import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from muri_dev_interfaces.action import DRIVE, TURN, INIT

from muri_logics.main_controller import MainController
from muri_logics.logic_interface import LogicInterface


class MuriActionHandler(Node):
    def __init__(self, logic: LogicInterface):
        super().__init__('muri_action_handler')

        self._action_client_drive = ActionClient(self, DRIVE, 'muri_drive')
        self._action_client_turn = ActionClient(self, TURN, 'muri_turn')
        self._action_client_init = ActionClient(self, INIT, 'muri_init')


        self.main_controller: LogicInterface = logic

        self.odom_sub_picture = self.create_subscription(
            bool,   # Hier echten typ angeben, sobald existiert
            '/muri_picture_data',  
            self.listener_callback_picture_data_ah,
            10
        )

    def listener_callback_picture_data_ah(self, msg):
        pass

    def send_drive_goal():
        pass

    def send_turn_goal():
        pass

    def send_init_goal():
        pass

    


def main(args=None):
    rclpy.init(args=args)

    logic = MainController()
    muri_action_handler = MuriActionHandler(logic)

    rclpy.spin(muri_action_handler)

    muri_action_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()