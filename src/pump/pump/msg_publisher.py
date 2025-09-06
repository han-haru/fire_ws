import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MsgPublisher(Node):
    def __init__(self):
        super().__init__('msg_publisher')
        # Publisher 선언
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # 주기적으로 입력을 받아 퍼블리시
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # 사용자 입력 받기
        msg_input = input("메시지를 입력하세요: ")
        msg = String()
        msg.data = msg_input
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = MsgPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()