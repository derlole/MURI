import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from muri_dev_interfaces.action import INIT
from muri_dev_interfaces.msg import PictureData
from muri_logics.logic_action_server_init import InitLogic, InitStates
from muri_logics.logic_interface import LogicInterface

class InitActionServer(Node):
    def __init__(self, logic: LogicInterface):
        super().__init__('muri_init_action_server')
        self.init_logic: LogicInterface = logic

        # Callback group für parallele Verarbeitung
        self._cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            INIT,
            'muri_init',
            execute_callback=self.execute_callback,
            #cancel_callback=self.cancel_callback,
            callback_group=self._cb_group
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.picture_data_sub = self.create_subscription(
            PictureData, '/muri_picture_data', self.listener_callback_picture_data_asi, 10,
            callback_group=self._cb_group
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.listener_callback_odom_asi, 10,
            callback_group=self._cb_group
        )

        self._last_picture_data = None
        self._last_odom = None

    def execute_callback(self, goal_handle):
        self.get_logger().info('Exec: init-goal')
        self.init_logic.reset()
        self.init_logic.setActive()

        result = INIT.Result()
        err_out_counter = 0
        ERR_THRESHOLD = 5

        while rclpy.ok() and goal_handle.is_active:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Canc: init-goal.')
                goal_handle.canceled()
                result.success = False
                return result

            self.init_logic.state_machine()
            out = self.init_logic.getOut()

            if not out.outValid():
                out.resetOut()
                err_out_counter += 1
                if err_out_counter >= ERR_THRESHOLD:
                    self.get_logger().error('ERR_THRESHOLD reached -> abort init goal.')
                    cmd_vel = Twist()
                    self.cmd_vel_pub.publish(cmd_vel)
                    goal_handle.abort()
                    result.success = False
                    return result
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            cmd_vel = Twist()
            cmd_vel.linear.x = float(out.values.get('linear_velocity_x', 0.0))
            cmd_vel.linear.y = float(out.values.get('linear_velocity_y', 0.0))
            cmd_vel.angular.z = float(out.values.get('angular_velocity_z', 0.0))
            self.cmd_vel_pub.publish(cmd_vel)

            feedback_msg = INIT.Feedback()
            feedback_msg.turned_angle = float(out.values.get('turned_angle', 0.0))
            goal_handle.publish_feedback(feedback_msg)

            active_state = self.init_logic.getActiveState()
            if active_state == InitStates.SUCCESS:
                self.get_logger().info('succ: init-goal.')
                goal_handle.succeed()
                result.success = True
                return result

            if active_state == InitStates.FAILED:
                self.get_logger().info('fail: init-goal.')
                goal_handle.abort()
                result.success = False
                return result

            out.resetOut()
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('rclpy not OK or goal not active -> aborting init goal.')
        result.success = False
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Rec: cancel init-goal')
        return CancelResponse.ACCEPT

    def listener_callback_odom_asi(self, msg):
        self._last_odom = msg
        self.init_logic.setOdomData(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation)

    def listener_callback_picture_data_asi(self, msg):
        self._last_picture_data = msg
        self.init_logic.setCameraData(msg.angle_in_rad, msg.distance_in_meters)


def main(args=None):
    rclpy.init(args=args)
    init_action_server = InitActionServer(InitLogic())
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(init_action_server)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        init_action_server.get_logger().info('Interrupt received at InitActionServer, shutting down.')
    finally:
        init_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
