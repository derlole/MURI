import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
from muri_dev_interfaces.action import DRIVE, TURN, INIT
from muri_dev_interfaces.msg import PictureData
from rclpy.executors import ExternalShutdownException
from muri_logics.main_controller import MainController
from muri_logics.logic_interface import LogicInterface


class MuriActionHandler(Node):
    def __init__(self, logic: LogicInterface):
        super().__init__('muri_action_handler')

        self._action_client_drive = ActionClient(self, DRIVE, 'muri_drive')
        self._action_client_turn = ActionClient(self, TURN, 'muri_turn')
        self._action_client_init = ActionClient(self, INIT, 'muri_init')

        self.last_odom = None
        self.last_picture_data = None
        self.main_controller: LogicInterface = logic

        self.picture_sub = self.create_subscription(
            PictureData,   # TODO Hier echten typ angeben, sobald existiert
            '/muri_picture_data',
            self.listener_callback_picture_data_ah,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback_odom_ah,
            10
        )
        self.timer = self.create_timer(0.1, self.main_loop_ah)

    def main_loop_ah(self):
        pass # TODO

    def listener_callback_picture_data_ah(self, msg):
        self.last_picture_data = msg

    def listener_callback_odom_ah(self, msg):
        self.last_odom = msg

    def send_drive_goal(self):
        self.get_logger().info('Sending drive goal...')

        gx, gy, gt = self.main_controller.calculate_estimated_goal_pose(last_odom_x=self.last_odom.pose.pose.position.x, last_odom_y=self.last_odom.pose.pose.position.y, last_odom_quaternion=self.last_odom.pose.pose.orientation)  
        drive_goal = DRIVE.Goal()
        drive_goal.target_pose.x = float(gx)
        drive_goal.target_pose.y = float(gy)
        drive_goal.target_pose.theta = float(gt)

        self._action_client_drive.wait_for_server()
        self._drive_send_promise = self._action_client_drive.send_goal_async(drive_goal, feedback_callback=self.drive_feedback_callback)
        self._drive_send_promise.add_done_callback(self.drive_goal_response_callback)


    def send_turn_goal(self):
        self.get_logger().info('Sending turn goal...')

        gx, gy, gt = self.main_controller.calculate_estimated_goal_pose(last_odom_x=self.last_odom.pose.pose.position.x, last_odom_y=self.last_odom.pose.pose.position.y, last_odom_quaternion=self.last_odom.pose.pose.orientation)  
        turn_goal = TURN.Goal()
        turn_goal.target_angle = float(gt)

        self._action_client_turn.wait_for_server()
        self._turn_send_promise = self._action_client_turn.send_goal_async(turn_goal, feedback_callback=self.turn_feedback_callback)
        self._turn_send_promise.add_done_callback(self.turn_goal_response_callback)

    def send_init_goal(self):
        self.get_logger().info('Sending init goal...')

        init_goal = INIT.Goal()

        self._action_client_init.wait_for_server()
        self._init_send_promise = self._action_client_init.send_goal_async(init_goal, feedback_callback=self.init_feedback_callback)
        self._init_send_promise.add_done_callback(self.init_goal_response_callback)

    def drive_feedback_callback(self, feedback_msg):
        self.get_logger().info('Drive: ' + str(feedback_msg))

    def turn_feedback_callback(self, feedback_msg):
        self.get_logger().info('Turn: ' + str(feedback_msg))

    def init_feedback_callback(self, feedback_msg):
        self.get_logger().info('Init: ' + str(feedback_msg))

    def drive_goal_response_callback(self, promise):
        goal_handle = promise.result()
        if not goal_handle.accepted:
            self.get_logger().info('Rej: drive-goal')
            return
        
        self.get_logger().info('Acc: drive-goal')

        self._drive_result_promise = goal_handle.get_result_async()
        self._drive_result_promise.add_done_callback(self.drive_result_callback)

    def turn_goal_response_callback(self, promise):
        goal_handle = promise.result()
        if not goal_handle.accepted:
            self.get_logger().info('Rej: turn-goal')
            return
        
        self.get_logger().info('Acc: turn-goal')

        self._turn_result_promise = goal_handle.get_result_async()
        self._turn_result_promise.add_done_callback(self.turn_result_callback)

    def init_goal_response_callback(self, promise):
        goal_handle = promise.result()
        if not goal_handle.accepted:
            self.get_logger().info('Rej: init-goal')
            return
        
        self.get_logger().info('Acc: init-goal')

        self._init_result_promise = goal_handle.get_result_async()
        self._init_result_promise.add_done_callback(self.init_result_callback)

    # TODO BETTER PRINTING
    def drive_result_callback(self, promise):
        result = promise.result().result
        self.get_logger().info('Drive result: {0}'.format(result))

    def turn_result_callback(self, promise):
        result = promise.result().result
        self.get_logger().info('Turn result: {0}'.format(result))

    def init_result_callback(self, promise):
        result = promise.result().result
        self.get_logger().info('Init result: {0}'.format(result))

def main(args=None):
    rclpy.init(args=args)

    logic = MainController()
    muri_action_handler = MuriActionHandler(logic)
    try:
        rclpy.spin(muri_action_handler)
    except (KeyboardInterrupt, ExternalShutdownException):
        muri_action_handler.get_logger().info('Interrupt received at MuriActionHandler, shutting down.')
    finally:
        muri_action_handler.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()