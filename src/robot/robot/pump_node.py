import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lgpio as lg
import time

class PumpNode(Node):
    def __init__(self):
        super().__init__('pump_node')

        # ── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('gpiochip', 4)
        self.declare_parameter('pump_pin', 16)
        self.declare_parameter('valve1_pin', 20)
        self.declare_parameter('valve2_pin', 21)
        self.declare_parameter('valve_settle_s', 1.0)
        # ────────────────────────────────────────────────────────

        chip = self.get_parameter('gpiochip').get_parameter_value().integer_value
        self.h = lg.gpiochip_open(chip)

        self.PUMP  = self.get_parameter('pump_pin').get_parameter_value().integer_value
        self.V1    = self.get_parameter('valve1_pin').get_parameter_value().integer_value
        self.V2    = self.get_parameter('valve2_pin').get_parameter_value().integer_value
        self.T_SET = float(self.get_parameter('valve_settle_s').get_parameter_value().double_value)

        # 핀 모드
        lg.gpio_claim_output(self.h, self.PUMP, 0)
        lg.gpio_claim_output(self.h, self.V1, 0)
        lg.gpio_claim_output(self.h, self.V2, 0)

        self.sub = self.create_subscription(String, '/pump/cmd', self.cb, 10)
        self.get_logger().info("[Pump] ready")

    def _all_off(self):
        lg.gpio_write(self.h, self.PUMP, 0)
        lg.gpio_write(self.h, self.V1, 0)
        lg.gpio_write(self.h, self.V2, 0)

    def cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "upright":
            # V2 열고 펌프 ON
            lg.gpio_write(self.h, self.V1, 0)
            lg.gpio_write(self.h, self.V2, 1)
            time.sleep(self.T_SET)
            lg.gpio_write(self.h, self.PUMP, 1)
            self.get_logger().info("[Pump] ON (valve2)")
        elif cmd == "upside_down":
            # V1 열고 펌프 ON
            lg.gpio_write(self.h, self.V2, 0)
            lg.gpio_write(self.h, self.V1, 1)
            time.sleep(self.T_SET)
            lg.gpio_write(self.h, self.PUMP, 1)
            self.get_logger().info("[Pump] ON (valve1)")
        elif cmd == "stop":
            self._all_off()
            self.get_logger().info("[Pump] STOP")
        else:
            self.get_logger().warn(f"[Pump] unknown cmd: {cmd}")

    def destroy_node(self):
        try:
            self._all_off()
            lg.gpiochip_close(self.h)
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = PumpNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
