import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from muri_dev_interfaces.action import TURN
from muri_dev_interfaces.msg import PictureData
from muri_logics.logic_action_server_turn import TurnLogic, TurnStates
from muri_logics.logic_interface import LogicInterface

class TurnActionServer(Node):
    def __init__(self, logic: LogicInterface):
        super().__init__('muri_turn_action_server')

        self.turn_logic: LogicInterface = logic

        self._action_server = ActionServer(
            self,
            TURN,
            'muri_turn',
            execute_callback=self.execute_callback
            #cancel_callback=self.cancel_callback,
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
        self._last_picture_data = None
        self._last_odom = None

    def execute_callback(self, goal_handle):
        self.get_logger().info('Exec: turn-goal')
        self.turn_logic.reset()
        self.turn_logic.setActive()
        err_out_counter = 0
        ERR_THRESHOLD = 5

        result = TURN.Result()

        while rclpy.ok() and goal_handle.is_active:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Canc: turn-goal.')
                goal_handle.canceled()
                result.success = False
                return result

            self.turn_logic.state_machine()
            out = self.turn_logic.getOut()

            if not out.outValid():
                out.resetOut()
                err_out_counter += 1

                if err_out_counter >= ERR_THRESHOLD:
                    self.get_logger().error('ERR_THRESHOLD erreicht -> abort turn-goal.')
                    # stop robot
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

            feedback_msg = TURN.Feedback()
            feedback_msg.moved_angle = float(out.values.get('turned_angle', 0.0))
            goal_handle.publish_feedback(feedback_msg)

            active_state = self.turn_logic.getActiveState()
            if active_state == TurnStates.SUCCESS:
                self.get_logger().info('succ: turn-goal.')
                goal_handle.succeed()
                result.success = True
                return result

            if active_state == TurnStates.FAILED:
                self.get_logger().info('fail: turn-goal.')
                goal_handle.abort()
                result.success = False
                return result

            out.resetOut()
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('rclpy nicht ok oder Goal nicht aktiv -> abort turn-goal.')
        result.success = False
        return result

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
    finally:
        turn_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
