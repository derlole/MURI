import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MuriControll(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.pub = self.create_publisher(String, 'keyboard/command', 10)
        self.run()

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                ch = sys.stdin.read(1)
                msg = String()
                if ch == 's':
                    msg.data = "start"
                elif ch == 'p':
                    msg.data = "pause"
                elif ch == 'x':
                    msg.data = "stop"
                else:
                    continue
                self.pub.publish(msg)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            
def main(args=None):
    rclpy.init(args=args)
    keyboard_node = MuriControll()
    rclpy.spin(keyboard_node)
    keyboard_node.destroy_node()
    rclpy.shutdown()
