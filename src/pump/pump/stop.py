import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SignalNode(Node):
    def __init__(self):
        super().__init__('stop')
        self.publisher_ = self.create_publisher(String, 'stop_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'stop'
        self.publisher_.publish(msg)
        self.get_logger().info('stop')

        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = SignalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()