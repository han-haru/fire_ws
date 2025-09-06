import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import lgpio as lg

class BladeNode(Node):
    def __init__(self):
        super().__init__('blade_node')

        # ── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('gpiochip', 4)   # 보통 RPi5는 0
        self.declare_parameter('pwm_pin', 19)   # PWM 핀
        self.declare_parameter('dir_pin', 26)   # 방향 핀
        self.declare_parameter('pwm_hz', 1000)  # PWM 주파수
        # ────────────────────────────────────────────────────────

        self.h = None
        self.percent = 0.0
        self.on = False

        # 핸들/핀 준비
        chip = self.get_parameter('gpiochip').get_parameter_value().integer_value
        self.h = lg.gpiochip_open(chip)

        self.PWM_PIN = self.get_parameter('pwm_pin').get_parameter_value().integer_value
        self.DIR_PIN = self.get_parameter('dir_pin').get_parameter_value().integer_value
        self.PWM_HZ  = self.get_parameter('pwm_hz').get_parameter_value().integer_value

        lg.gpio_claim_output(self.h, self.DIR_PIN, 0)
        lg.gpio_claim_output(self.h, self.PWM_PIN, 0)

        self.sub = self.create_subscription(Float32, '/blade/cmd', self.cb_speed, 10)
        self.get_logger().info("[Blade] ready")

    def _apply(self):
        duty = abs(self.percent) / 100.0
        lg.gpio_write(self.h, self.DIR_PIN, 1 if self.percent >= 0 else 0)
        lg.tx_pwm(self.h, self.PWM_PIN, self.PWM_HZ, duty if self.on and duty > 0 else 0.0)

    def cb_speed(self, msg: Float32):
        # 규약:  -100~100 → 속도/방향, 0은 정지
        val = max(-100.0, min(100.0, float(msg.data)))
        self.percent = val
        self.on = (val != 0.0)
        self._apply()
        self.get_logger().info(f"[Blade] {self.percent:.1f}%")

    def destroy_node(self):
        try:
            lg.tx_pwm(self.h, self.PWM_PIN, self.PWM_HZ, 0.0)
            lg.gpio_write(self.h, self.DIR_PIN, 0)
            lg.gpiochip_close(self.h)
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = BladeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
