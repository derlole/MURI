import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from muri_dev_interfaces.action import DRIVE
from muri_dev_interfaces.msg import PictureData
from muri_logics.logic_action_server_drive import DriveLogic, DriveStates
from muri_logics.logic_interface import LogicInterface

ERR_THRESHOLD = 5

class DriveActionServer(Node):
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
            handle_accepted_callback = self.handle_acc_callback
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
        self._timer = self.create_timer(0.1, self.timer_callback_asd)
        self._goal_handle = None
        self._last_picture_data = None
        self._last_odom = None
        self.__err_out_counter = 0

    def timer_callback_asd(self):
        if self._goal_handle is None or not self._goal_handle.is_active:
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

        if not out.outValid():
            out.resetOut()
            self.__err_out_counter += 1
        
        cmd_vel = Twist()
        cmd_vel.linear.x = float(out.values['linear_velocity_x'])
        cmd_vel.linear.y = float(out.values['linear_velocity_y'])
        cmd_vel.angular.z = float(out.values['angular_velocity_z'])

        if self.__err_out_counter >= ERR_THRESHOLD:
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            return
        else:
            self.cmd_vel_pub.publish(cmd_vel)

        feedback_msg = DRIVE.Feedback()
        feedback_msg.distance_remaining = float(out.values['distance_remaining'])

        self._goal_handle.publish_feedback(feedback_msg)

        if out.getState() == DriveStates.SUCCESS:
            self.get_logger().info('succ: drive-goal.')
            self._goal_handle.succeed()

            self._goal_result = DRIVE.Result()
            self._goal_result.success = True
            self._goal_exiting = True

            self._goal_handle = None

        elif out.getState() == DriveStates.FAILED:
            self.get_logger().info('fail: drive-goal.')
            self._goal_handle.abort()

            self._goal_result = DRIVE.Result()
            self._goal_result.success = False
            self._goal_exiting = True

            self._goal_handle = None

        out.resetOut()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Exe: drive goal')

        self._goal_exiting = False
        self._goal_result = None

        while not self._goal_exiting:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self._goal_result
        # return DRIVE.Result() 

    def goal_callback(self, goal_request):
        self.get_logger().info('Rec: drive-goal')

        if self._goal_handle is not None and self._goal_handle.is_active:
            self.get_logger().info('Rej: drive-goal')
            return GoalResponse.REJECT
        
        self.get_logger().info('Acc: drive-goal')
        return GoalResponse.ACCEPT
    
    def handle_acc_callback(self, goal_handle):
        self._goal_handle = goal_handle
        self.drive_logic.reset()
        self.drive_logic.setActive()
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Rec: cancel drive-goal')
        return CancelResponse.ACCEPT

    def listener_callback_odom_asd(self, msg):
        self._last_odom = msg
        self.drive_logic.setOdomData(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation)

    def listener_callback_picture_data_asd(self, msg):
        self._last_picture_data = msg
        self.drive_logic.setCameraData(msg.pixel_to_mid, msg.pixel_to_mid_prev, msg.pixel_height, msg.pixel_height_prev, msg.pic_width)

def main(args=None):
    rclpy.init(args=args)

    drive_action_server = DriveActionServer(DriveLogic())
    try:
        rclpy.spin(drive_action_server)
    except (KeyboardInterrupt, ExternalShutdownException):
        drive_action_server.get_logger().info('Interrupt receivedat DriveActionServer, shutting down.')
        # cmd_vel wird vom main controller auf null gesetzt
    finally:
        drive_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':    
    main()