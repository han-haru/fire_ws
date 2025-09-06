# 이건 로컬 PC의 터미널에서 실행하면 됨
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CompressedViewer(Node):
    def __init__(self):
        super().__init__('compressed_viewer')

        # Sensor-data QoS (BestEffort, Volatile) to match camera publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        topic = '/imx219_cam/image_raw/compressed'
        self.subscription = self.create_subscription(
            CompressedImage, topic, self.listener_callback, qos)
        self.get_logger().info(f"Subscribed to {topic}")

    def listener_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("Failed to decode frame!")
            return
        cv2.imshow("Robot Camera (decompressed)", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CompressedViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
