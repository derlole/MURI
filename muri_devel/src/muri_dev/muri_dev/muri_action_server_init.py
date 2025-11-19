import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from muri_dev_interfaces.action import INIT
from muri_dev_interfaces.msg import PictureData
from muri_logics.logic_action_server_init import InitLogic, InitStates
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
            PictureData,
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
        self._timer = self.create_timer(0.1, self.timer_callback_asi)
        self._goal_handle = None
        self._last_picture_data = None
        self._last_odom = None

    def timer_callback_asi(self):
        if self._goal_handle is None or not self._goal_handle.is_active:
            return
        
        if self._goal_handle.is_cancel_requested:
            self.get_logger().info('Canc: init-goal.')
            self._goal_handle.canceled()

            self._goal_result = INIT.Result()
            self._goal_result.success = False
            self._goal_exiting = True

            self._goal_handle = None
            return 

        self.init_logic.state_machine()
        out = self.init_logic.getOut()

        if not out.outValid():
            out.resetOut()
            return
        print(str(out.values))
        cmd_vel = Twist()
        cmd_vel.linear.x = float(out.values['linear_velocity_x'])
        cmd_vel.linear.y = float(out.values['linear_velocity_y'])
        cmd_vel.angular.z = float(out.values['angular_velocity_z'])
        self.cmd_vel_pub.publish(cmd_vel)

        feedback_msg = INIT.Feedback()
        feedback_msg.turned_angle = float(out.values['turned_angle'])
        self._goal_handle.publish_feedback(feedback_msg)

        if self.init_logic.getActiveState() == InitStates.SUCCESS:
            self.get_logger().info('succ: init-goal.')
            self._goal_handle.succeed()

            self._goal_result = INIT.Result()
            self._goal_result.success = True
            self._goal_exiting = True

            self._goal_handle = None

        elif self.init_logic.getActiveState() == InitStates.FAILED:
            self.get_logger().info('fail: init-goal.')
            self._goal_handle.abort()

            self._goal_result = INIT.Result()
            self._goal_result.success = False
            self._goal_exiting = True

            self._goal_handle = None

        out.resetOut()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Exec: init-goal')

        self._goal_handle = goal_handle
        
        self._goal_exiting = False
        self._goal_result = None

        while not self._goal_result:
            rclpy.spin_once(self, timeout_sec=0.1)

        return self._goal_result

    def goal_callback(self, goal_request):
        self.get_logger().info('Rec: init-goal')

        if self._goal_handle is not None and self._goal_handle.is_active:
            self.get_logger().info('Rej: init-goal')
            return GoalResponse.REJECT
        
        self.get_logger().info('Acc: init-goal')

        self.init_logic.reset()
        self.init_logic.setActive()

        return GoalResponse.ACCEPT

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
        rclpy.spin(init_action_server)
    except (KeyboardInterrupt, ExternalShutdownException):
        init_action_server.get_logger().info('Interrupt received at InitActionServer, shutting down.')
        # cmd_vel wird vom main controller auf null gesetzt
    finally:
        init_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()