import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import gpiod
import time

CHIP = "/dev/gpiochip4"
GPIO_PUMP = 16
GPIO_VALVE_1 = 20
GPIO_VALVE_2 = 21

class PumpNode(Node):
    def __init__(self):
        super().__init__('pump_node')

        # ---- gpiod 라인 요청 ----
        self.chip = gpiod.Chip(CHIP)

        settings = gpiod.LineSettings(
            direction=gpiod.line.Direction.OUTPUT,
            output_value=gpiod.line.Value.INACTIVE   # 초기값 모두 OFF
        )
        config = {
            GPIO_PUMP: settings,
            GPIO_VALVE_1: settings,
            GPIO_VALVE_2: settings,
        }

        self.request = self.chip.request_lines(
            config=config,
            consumer='pump_control'
        )

        # ---- 파라미터 ----
        self.declare_parameter('valve_settle_s', 1.0)
        self.T_SET = float(self.get_parameter('valve_settle_s').get_parameter_value().double_value)

        # ---- 구독 ----
        self.sub = self.create_subscription(
            String, '/pump/cmd', self.cb, 10
        )
        self.get_logger().info("[Pump] ready")

    # ---- 모두 OFF ----
    def _all_off(self):
        self.request.set_value(GPIO_PUMP, gpiod.line.Value.INACTIVE)
        self.request.set_value(GPIO_VALVE_1, gpiod.line.Value.INACTIVE)
        self.request.set_value(GPIO_VALVE_2, gpiod.line.Value.INACTIVE)

    # ---- 콜백 ----
    def cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "valve1":
            self.request.set_value(GPIO_PUMP, gpiod.line.Value.ACTIVE)
            self.get_logger().info("[Pump] ON (valve1)")
            self.request.set_value(GPIO_VALVE_1, gpiod.line.Value.INACTIVE)
            self.request.set_value(GPIO_VALVE_2, gpiod.line.Value.ACTIVE)
            time.sleep(self.T_SET)
            
        elif cmd == "valve2":
            self.request.set_value(GPIO_PUMP, gpiod.line.Value.INACTIVE)
            self.get_logger().info("[Pump] ON (valve2)")
            self.request.set_value(GPIO_VALVE_1, gpiod.line.Value.ACTIVE)
            self.request.set_value(GPIO_VALVE_2, gpiod.line.Value.ACTIVE)
            time.sleep(self.T_SET)
            
        elif cmd == "stop":
            self._all_off()
            self.get_logger().info("[Pump] STOP")
        else:
            self.get_logger().warn(f"[Pump] unknown cmd: {cmd}")

    # ---- 정리 ----
    def destroy_node(self):
        try:
            self._all_off()
            self.request.release()
        except Exception as e:
            self.get_logger().warn(f'Cleanup error: {e}')
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = PumpNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
