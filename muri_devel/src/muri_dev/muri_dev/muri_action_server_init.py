import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from muri_dev_interfaces.action import INIT
from muri_dev_interfaces.msg import PictureData
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
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_acc_callback
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
        self._goal_handle = None
        self._last_picture_data = None
        self._last_odom = None

    def timer_callback_asi(self):
        if self._goal_handle is None or not self._goal_handle.is_active:
            return
        
        if self._goal_handle.is_cancel_requested:
            self.get_logger().info('Canc: init-goal.')
            self._goal_handle.canceled()

            result = INIT.Result()
            result.success = False

            self._goal_handle.publish_result(result)
            self._timer.cancel()
            self._goal_handle = None
            return # TODO handle canc here?

        self.init_logic.state_machine()
        out = self.init_logic.getOut()

        if not out.outValid():
            out.resetOut()
            return
        
        cmd_vel = Twist()
        cmd_vel.linear.x = float(out.values['linear_velocity_x']) # TODO communicate to Louis that to put there
        cmd_vel.linear.y = float(out.values['linear_velocity_y']) # TODO communicate to Louis that to put there
        cmd_vel.angular.z = float(out.values['angular_velocity_z']) # TODO communicate to Louis that to put there
        self.cmd_vel_pub.publish(cmd_vel)

        feedback_msg = INIT.Feedback()
        feedback_msg.turned_angle = float(out.values['turned_angle'])  # TODO communicate to Louis that to put there
        self._goal_handle.publish_feedback(feedback_msg)

        if out.getState() == InitLogic.State.SUCCESS:
            self.get_logger().info('succ: init-goal.')
            self._goal_handle.succeed()

            result = INIT.Result()
            result.success = True

            self._goal_handle.publish_result(result)
            self._timer.cancel()
            self._goal_handle = None

        elif out.getState() == InitLogic.State.FAILURE:
            self.get_logger().info('fail: init-goal.')
            self._goal_handle.abort()

            result = INIT.Result()
            result.success = False

            self._goal_handle.publish_result(result)
            self._timer.cancel()
            self._goal_handle = None

        out.resetOut()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Exec: init-goal')
        self._timer = self.create_timer(0.1, self.timer_callback_asi)

        return INIT.Result() 

    def goal_callback(self, goal_request):
        self.get_logger().info('Rec: init-goal')

        if self._goal_handle is not None and self._goal_handle.is_active:
            self.get_logger().info('Rej: init-goal')
            return GoalResponse.REJECT
        
        self.get_logger().info('Acc: init-goal')
        return GoalResponse.ACCEPT

    def handle_acc_callback(self, goal_handle):
        self._goal_handle = goal_handle
        self.init_logic.reset()
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Rec: cancel init-goal')
        return CancelResponse.ACCEPT

    def listener_callback_odom_asi(self, msg):
        self._last_odom = msg

    def listener_callback_picture_data_asi(self, msg):
        self._last_picture_data = msg

def main(args=None):
    rclpy.init(args=args)

    init_action_server = InitActionServer(InitLogic())

    try:
        rclpy.spin(init_action_server)
    except (KeyboardInterrupt, ExternalShutdownException):
        init_action_server.get_logger().info('Interrupt received at InitActionServer, shutting down.')
        # TODO hier mayeb noch nen /cmd_vel auf 0
    finally:
        init_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()