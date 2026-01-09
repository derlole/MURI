import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from muri_dev_interfaces.action import DRIVE
from muri_dev_interfaces.msg import PictureData
from muri_logics.logic_action_server_drive import DriveLogic, DriveStates
from muri_logics.logic_interface import LogicInterface
import config
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup 
import time

ERR_THRESHOLD = 5

class DriveActionServer(Node):
    """ROS2 Action Server for driving the robot."""

    def __init__(self, logic: LogicInterface):
        super().__init__('muri_drive_action_server')

        self.drive_logic: LogicInterface = logic

        self._action_server = ActionServer(
            self,
            DRIVE,
            'muri_drive',
            execute_callback = self.execute_callback,
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel',
             10
        )
        self.picture_data_sub = self.create_subscription(
            PictureData,
            '/muri_picture_data',  
            self.listener_callback_picture_data_asd,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  
            self.listener_callback_odom_asd,
            10
        )
        self.schpieth_sup = self.create_subscription(
            Float32,
            '/muri_speed',
            self.listener_callback_schpieth_asd,
            10
        )
        self._timer = self.create_timer(0.1, self.timer_callback_asd, MutuallyExclusiveCallbackGroup())
        self._goal_handle = None
        self._last_picture_data = None
        self._last_odom = None
        self.__err_out_counter = 0

    def timer_callback_asd(self):
        """Main timer callback for handling active drive goals."""
        if self._goal_handle is None or not self._goal_handle.is_active:
            return
        
        if self._last_picture_data.dominant_aruco_id == 69:
            return

        if self._goal_handle.is_cancel_requested:
            self.get_logger().info('Canc: drive-goal.')
            self._goal_handle.canceled()

            self._goal_result = DRIVE.Result()
            self._goal_result.success = False
            self._goal_exiting = True
            self._goal_handle = None
            return

        self.drive_logic.state_machine()
        out = self.drive_logic.getOut()
        #print(str(out.values['linear_velocity_x']) + str(out.outValid()) + str(out.values['angular_velocity_z']))
        if not out.outValid():
            out.resetOut()
            return
        
        cmd_vel = Twist()
        cmd_vel.linear.x = float(out.values['linear_velocity_x'])
        cmd_vel.linear.y = float(out.values['linear_velocity_y'])
        cmd_vel.angular.z = float(out.values['angular_velocity_z'])


        self.cmd_vel_pub.publish(cmd_vel)

        feedback_msg = DRIVE.Feedback()
        feedback_msg.distance_remaining = float(out.values['distance_remaining'])

        self._goal_handle.publish_feedback(feedback_msg)

        if self.drive_logic.getActiveState() == DriveStates.SUCCESS:
            self.get_logger().info('succ: drive-goal.')
            self._goal_handle.succeed()

            self._goal_result = DRIVE.Result()
            self._goal_result.success = True
            self._goal_exiting = True

            self._goal_handle = None

        elif self.drive_logic.getActiveState() == DriveStates.FAILED:
            self.get_logger().info('fail: drive-goal.')
            self._goal_handle.abort()

            self._goal_result = DRIVE.Result()
            self._goal_result.success = False
            self._goal_exiting = True

            self._goal_handle = None

        out.resetOut()

    def execute_callback(self, goal_handle):
        """Execute callback for drive action."""
        self.get_logger().info('Exe: drive goal')

        self._goal_handle = goal_handle

        self._goal_exiting = False
        self._goal_result = None

        while not self._goal_exiting:
            time.sleep(0.05)

        return self._goal_result

    def goal_callback(self, goal_request):
        """Goal callback for accepting/rejecting drive goals."""
        self.get_logger().info('Rec: drive-goal')

        if self._goal_handle is not None and self._goal_handle.is_active:
            self.get_logger().info('Rej: drive-goal')
            return GoalResponse.REJECT
        
        self.get_logger().info('Acc: drive-goal')

        self.drive_logic.reset()
        self.drive_logic.setActive()

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Cancel callback for drive goals."""
        self.get_logger().info('Rec: cancel drive-goal')
        return CancelResponse.ACCEPT

    def listener_callback_odom_asd(self, msg):
        """Callback for odometry data."""
        self._last_odom = msg
        self.drive_logic.setOdomData(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation)

    def listener_callback_picture_data_asd(self, msg):
        """Callback for picture data."""
        self._last_picture_data = msg
        self.drive_logic.setCameraData(msg.angle_in_rad, msg.distance_in_meters)

    def listener_callback_schpieth_asd(self, msg):
        """Callback for speed data."""
        value = max(config.MINIMAL_SPEED_TO_SET, min(msg.data, config.MAXIMAL_SPEED_TO_SET))
        self.drive_logic.setSchpieth(value)
        
def main(args=None):
    """Main function to run the DriveActionServer."""
    rclpy.init(args=args)

    drive_action_server = DriveActionServer(DriveLogic())
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(drive_action_server)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        drive_action_server.get_logger().info('Interrupt receivedat DriveActionServer, shutting down.')
        # cmd_vel wird vom main controller auf null gesetzt
    finally:
        drive_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':    
    main()
