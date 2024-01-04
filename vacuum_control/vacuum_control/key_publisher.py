import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import getch

class KeyPublisher(Node):
    def __init__(self):
        super().__init__('key_publisher')
        self.publisher_ = self.create_publisher(String, 'key_pressed', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = String()
        try:
            key = getch.getch()
            # Ensure that the key is a single character
            if len(key) == 1:
                msg.data = key
                self.publisher_.publish(msg)
                self.get_logger().info(f'Key {msg.data} pressed')
            else:
                self.get_logger().warn('Invalid key pressed. Ignoring.')

        except Exception as e:
            self.get_logger().error(f'Error while getting key: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = KeyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

