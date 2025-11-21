import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
from muri_dev_interfaces.action import DRIVE, TURN, INIT
from muri_dev_interfaces.msg import PictureData
from rclpy.executors import ExternalShutdownException
from muri_logics.main_controller import MainController, MainStates
from muri_logics.logic_interface import LogicInterface
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class MuriActionHandler(Node):
    def __init__(self, logic: LogicInterface):
        super().__init__('muri_action_handler')

        client_cbg = MutuallyExclusiveCallbackGroup()
        timer_cbg = MutuallyExclusiveCallbackGroup()

        self._action_client_turn = ActionClient(self, TURN, 'muri_turn', callback_group=client_cbg)


        self.last_odom = None
        self.last_picture_data = None
        self.main_controller: LogicInterface = logic

        self.picture_sub = self.create_subscription(
            PictureData,
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
        self.timer = self.create_timer(0.1, self.main_loop_ah, callback_group=timer_cbg)
        self.calledSomething = False

    def main_loop_ah(self):
        if not self.calledSomething:
            self.calledSomething = True
            self.send_turn_goal()

    def listener_callback_picture_data_ah(self, msg):
        self.last_picture_data = msg
        self.main_controller.setCameraData(msg.angle_in_rad, msg.distance_in_meters)

    def listener_callback_odom_ah(self, msg):
        self.last_odom = msg
        self.main_controller.setOdomData(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation)

    def send_turn_goal(self):
        self.get_logger().info('Sending turn goal...')
        goal_msg = TURN.Goal()
        goal_msg.target_angle = float(0.0)

        self.get_logger().info("test1")
        #self._action_client_turn.wait_for_server()
        self.get_logger().info("test2")
        self._send_turn_goal_future = self._action_client_turn.send_goal_async(goal_msg, feedback_callback=self.feedback_callback_turn)
        self._send_turn_goal_future.add_done_callback(self.goal_response_callback_turn)
        self.get_logger().info("test3")

    def goal_response_callback_turn(self, future):
        goal_handler = future.result()
        if not goal_handler.accepted:
            self.get_logger().info('Turn goal rejected :(')
            return

        self.get_logger().info('Turn goal accepted :)')

        self._get_turn_result_future = goal_handler.get_result_async()
        self._get_turn_result_future.add_done_callback(self.get_turn_result_callback)

    def feedback_callback_turn(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Rec turn feedback: moved_angle={feedback.moved_angle}')

    def get_turn_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Turn result: success={result.success}')
        self.calledSomething = False

def main(args=None):
    rclpy.init(args=args)

    action_client = MuriActionHandler(MainController())
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(action_client)
    
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()