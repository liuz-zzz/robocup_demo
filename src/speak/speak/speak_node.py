import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class SpeakNode(Node):
    def __init__(self):
        super().__init__('speak_node')
        self.sub_speak = self.create_subscription(
            String,
            '/speak',
            self.callback_speak,
            10)

    def callback_speak(self, msg):
        self.get_logger().info(f'Received speak command: \"{msg.data}\"')
        subprocess.run(['espeak', msg.data])

def main(args=None):
    rclpy.init(args=args)
    node = SpeakNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()