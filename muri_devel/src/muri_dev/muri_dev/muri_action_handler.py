# Written, maintained and owned by Louis Moser, Linus Braun, Benjamin Keppler (MURI DEVELOPMENT TEAM)

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Odometry
from muri_dev_interfaces.action import DRIVE, TURN, INIT, FOLLOW
from muri_dev_interfaces.msg import PictureData
from rclpy.executors import ExternalShutdownException
from muri_logics.main_controller import MainController, MainStates
from muri_logics.logic_interface import ExtendedLogicInterface
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup 


class MuriActionHandler(Node):
    def __init__(self, logic: ExtendedLogicInterface):
        super().__init__('muri_action_handler')

        self._action_client_drive = ActionClient(self, DRIVE, 'muri_drive')
        self._action_client_turn = ActionClient(self, TURN, 'muri_turn')
        self._action_client_init = ActionClient(self, INIT, 'muri_init')
        self._action_client_follow = ActionClient(self, FOLLOW, 'muri_follow')

        self.last_odom = None
        self.last_picture_data = None
        self.main_controller: ExtendedLogicInterface = logic
        self.ignor_next_drive_goal_status = False

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
        self.timer = self.create_timer(0.1, self.main_loop_ah, MutuallyExclusiveCallbackGroup())

    def main_loop_ah(self):
        """Main loop for handling action coordination."""
        self.main_controller.state_machine()
        out = self.main_controller.getOut()

        if self.main_controller.getActiveState() == MainStates.IDLE:
            self.main_controller.postInit()
            out = self.main_controller.getOut()

        if not out.outValid():
            return 
        
        if not out.values == {}:
            if out.values['ASToCall'] == 0:
                self.send_init_goal()

            if out.values['ASToCall'] == 1:
                self.send_drive_goal()

            if out.values['ASToCall'] == 2:
                self.send_turn_goal()

            if out.values['ASToCall'] == 3:
                self.send_follow_goal()

            if out.values['ASToCall'] == 4:
                self.cancle_drive_goal()


    def listener_callback_picture_data_ah(self, msg):
        """Callback for picture data."""
        self.last_picture_data = msg
        self.main_controller.setCameraData(msg.angle_in_rad, msg.distance_in_meters)
        self.main_controller.setArucoData(msg.dominant_aruco_id)
    
    def listener_callback_odom_ah(self, msg):
        """Callback for odometry data."""
        self.last_odom = msg
        self.main_controller.setOdomData(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation)


    def send_drive_goal(self):
        """Send a drive goal to the action server."""
        self.get_logger().info('Sending drive goal...')

        gx, gy, gt = self.main_controller.calculate_estimated_goal_pose(last_odom_x=self.last_odom.pose.pose.position.x, last_odom_y=self.last_odom.pose.pose.position.y, last_odom_quaternion=self.last_odom.pose.pose.orientation)  
        drive_goal = DRIVE.Goal()
        drive_goal.target_pose.x = float(gx)
        drive_goal.target_pose.y = float(gy)
        drive_goal.target_pose.theta = float(gt)

        self._action_client_drive.wait_for_server()

        self._drive_send_promise = self._action_client_drive.send_goal_async(drive_goal, feedback_callback=self.drive_feedback_callback)
        self._drive_send_promise.add_done_callback(self.drive_goal_response_callback)
        self.get_logger().info(f"Result of send_goal_async: {self._drive_send_promise}")

    def send_follow_goal(self):
        """Send a follow goal to the action server."""
        self.get_logger().info('Sending follow goal...')
        follow_goal = FOLLOW.Goal()

        self._action_client_follow.wait_for_server()
        self._follow_send_promise = self._action_client_follow.send_goal_async(follow_goal, feedback_callback=self.follow_feedback_callback)
        self._follow_send_promise.add_done_callback(self.follow_goal_response_callback)

    def send_turn_goal(self):
        """Send a turn goal to the action server."""
        self.get_logger().info('Sending turn goal...')

        gx, gy, gt = self.main_controller.calculate_estimated_goal_pose(last_odom_x=self.last_odom.pose.pose.position.x, last_odom_y=self.last_odom.pose.pose.position.y, last_odom_quaternion=self.last_odom.pose.pose.orientation)  
        turn_goal = TURN.Goal()
        turn_goal.target_angle = float(gt)

        self._action_client_turn.wait_for_server()
        self._turn_send_promise = self._action_client_turn.send_goal_async(turn_goal, feedback_callback=self.turn_feedback_callback)
        self._turn_send_promise.add_done_callback(self.turn_goal_response_callback)

    def send_init_goal(self):
        """Send an init goal to the action server."""
        self.get_logger().info('Sending init goal...')

        init_goal = INIT.Goal()

        self._action_client_init.wait_for_server()
        self._init_send_promise = self._action_client_init.send_goal_async(init_goal, feedback_callback=self.init_feedback_callback)
        self._init_send_promise.add_done_callback(self.init_goal_response_callback)


    def follow_feedback_callback(self, feedback_msg):
        """Feedback callback for follow action."""
        # self.get_logger().info('Follow: ' + str(feedback_msg))
        pass

    def drive_feedback_callback(self, feedback_msg):
        """Feedback callback for drive action."""
        pass
        # self.get_logger().info('Drive: ' + str(feedback_msg))

    def turn_feedback_callback(self, feedback_msg):
        """Feedback callback for turn action."""
        pass
        # self.get_logger().info('Turn: ' + str(feedback_msg))

    def init_feedback_callback(self, feedback_msg):
        """Feedback callback for init action."""
        pass 
        # self.get_logger().info('Init: ' + str(feedback_msg))


    def drive_goal_response_callback(self, promise):
        """Goal response callback for drive action."""
        self._drive_goal_handle = promise.result()
        if not self._drive_goal_handle.accepted:
            self.get_logger().info('Rej: drive-goal')
            return
        
        self.get_logger().info('Acc: drive-goal')

        self._drive_result_promise = self._drive_goal_handle.get_result_async()
        self._drive_result_promise.add_done_callback(self.drive_result_callback)

    def turn_goal_response_callback(self, promise):
        """Goal response callback for turn action."""
        self._turn_goal_handle = promise.result()
        if not self._turn_goal_handle.accepted:
            self.get_logger().info('Rej: turn-goal')
            return
        
        self.get_logger().info('Acc: turn-goal')

        self._turn_result_promise = self._turn_goal_handle.get_result_async()
        self._turn_result_promise.add_done_callback(self.turn_result_callback)

    def init_goal_response_callback(self, promise):
        """Goal response callback for init action."""
        self._init_goal_handle = promise.result()
        if not self._init_goal_handle.accepted:
            self.get_logger().info('Rej: init-goal')
            return
        
        self.get_logger().info('Acc: init-goal')

        self._init_result_promise = self._init_goal_handle.get_result_async()
        self._init_result_promise.add_done_callback(self.init_result_callback)

    def follow_goal_response_callback(self, promise):
        """Goal response callback for follow action."""
        self._follow_goal_handle = promise.result()
        if not self._follow_goal_handle.accepted:
            self.get_logger().info('Rej: follow-goal')
            return
        
        self.get_logger().info('Acc: follow-goal')

        self._follow_result_promise = self._follow_goal_handle.get_result_async()
        self._follow_result_promise.add_done_callback(self.follow_result_callback)


    def follow_result_callback(self, promise):
        """Result callback for follow action."""
        result = promise.result().result
        self.main_controller.setGoalSuccess(result.success)
        self.main_controller.setGoalStautusFinished(True)
        self.get_logger().info('Follow result: {0}'.format(result))

    def drive_result_callback(self, promise):
        """Result callback for drive action."""
        result = promise.result().result
        self.get_logger().info('Drive result: {0}'.format(result))
        if not self.ignor_next_drive_goal_status:       
            self.main_controller.setGoalSuccess(result.success)
            self.main_controller.setGoalStautusFinished(True)

        self.ignor_next_drive_goal_status = False
        
    def turn_result_callback(self, promise):
        """Result callback for turn action."""
        result = promise.result().result
        self.main_controller.setGoalSuccess(result.success)
        self.main_controller.setGoalStautusFinished(True)
        self.get_logger().info('Turn result: {0}'.format(result))

    def init_result_callback(self, promise):
        """Result callback for init action."""
        result = promise.result().result
        self.main_controller.setGoalSuccess(result.success)
        self.main_controller.setGoalStautusFinished(True)
        self.get_logger().info('Init result: {0}'.format(result))


    def cancle_drive_goal(self):
        """Cancel the current drive goal."""
        if hasattr(self, "_drive_goal_handle") and self._drive_goal_handle is not None:
            self.get_logger().info("Requesting drive-goal cancel...")
            future = self._drive_goal_handle.cancel_goal_async()
            future.add_done_callback(self.finish_drive_state)

    def finish_drive_state(self, promise):
        """Finish the drive state after cancellation."""
        self.get_logger().info("Canceled drive Goal.")
        self.main_controller.setGoalSuccess(False)
        self.main_controller.setGoalStautusFinished(True)
        self.ignor_next_drive_goal_status = True


def main(args=None):
    """Main function to run the MuriActionHandler."""
    rclpy.init(args=args)

    logic = MainController()
    muri_action_handler = MuriActionHandler(logic)
    executor = MultiThreadedExecutor(num_threads=4)
    
    try:    
        rclpy.spin(muri_action_handler, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        muri_action_handler.get_logger().info('Interrupt received at MuriActionHandler, shutting down.')
    finally:
        muri_action_handler.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()