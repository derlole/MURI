import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from muri_dev_interfaces.action import DRIVE
from muri_dev_interfaces.msg import PictureData
from muri_logics.logic_action_server_drive import DriveLogic, DriveStates
from muri_logics.logic_interface import LogicInterface

ERR_THRESHOLD = 5

class DriveActionServer(Node):
    def __init__(self, logic: LogicInterface):
        super().__init__('muri_drive_action_server')
        self.drive_logic: LogicInterface = logic

        # Die ReentrantCallbackGroup ist das Wichtigste!
        self._cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            DRIVE,
            'muri_drive',
            execute_callback=self.execute_callback,
            #cancel_callback=self.cancel_callback,
            callback_group=self._cb_group
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        self.picture_data_sub = self.create_subscription(
            PictureData, '/muri_picture_data', self.listener_callback_picture_data_asd, 10,
            callback_group=self._cb_group
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.listener_callback_odom_asd, 10,
            callback_group=self._cb_group
        )
        self._last_picture_data = None
        self._last_odom = None

    def execute_callback(self, goal_handle):
        self.get_logger().info('Exe: drive goal')
        self.drive_logic.reset()
        self.drive_logic.setActive()
        err_out_counter = 0

        result = DRIVE.Result()

        while rclpy.ok() and goal_handle.is_active:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Canc: drive-goal.')
                goal_handle.canceled()
                result.success = False
                return result

            self.drive_logic.state_machine()
            out = self.drive_logic.getOut()

            if not out.outValid():
                out.resetOut()
                err_out_counter += 1
                if err_out_counter >= ERR_THRESHOLD:
                    self.get_logger().error('ERR_THRESHOLD reached -> aborting drive goal.')
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.0
                    cmd_vel.linear.y = 0.0
                    cmd_vel.angular.z = 0.0
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

            feedback_msg = DRIVE.Feedback()
            feedback_msg.distance_remaining = float(out.values.get('distance_remaining', 0.0))
            goal_handle.publish_feedback(feedback_msg)

            active_state = self.drive_logic.getActiveState()
            if active_state == DriveStates.SUCCESS:
                self.get_logger().info('succ: drive-goal.')
                goal_handle.succeed()
                result.success = True
                return result

            if active_state == DriveStates.FAILED:
                self.get_logger().info('fail: drive-goal.')
                goal_handle.abort()
                result.success = False
                return result

            out.resetOut()
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('rclpy not OK or goal not active -> aborting drive goal.')
        result.success = False
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Rec: cancel drive-goal')
        return CancelResponse.ACCEPT

    def listener_callback_odom_asd(self, msg):
        self._last_odom = msg
        self.drive_logic.setOdomData(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation)

    def listener_callback_picture_data_asd(self, msg):
        self._last_picture_data = msg
        self.drive_logic.setCameraData(msg.angle_in_rad, msg.distance_in_meters)

def main(args=None):
    rclpy.init(args=args)
    drive_action_server = DriveActionServer(DriveLogic())
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(drive_action_server)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        drive_action_server.get_logger().info('Interrupt received at DriveActionServer, shutting down.')
    finally:
        drive_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
