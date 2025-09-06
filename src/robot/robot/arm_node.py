import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lgpio as lg
import time, threading

class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')

        # ── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('gpiochip', 4)
        self.declare_parameter('relay_pin', 5)     # ENA- 릴레이
        self.declare_parameter('pul_pin', 6)       # PUL-
        self.declare_parameter('dir_pin', 13)      # DIR-
        self.declare_parameter('active_high', True)
        self.declare_parameter('freq_hz', 800)     # 펄스 주파수
        self.declare_parameter('pulse_low_us', 20)
        self.declare_parameter('min_gap_us', 6)
        self.declare_parameter('microstep', 8)     # 마이크로스텝 설정
        self.declare_parameter('step_angle', 1.8)  # ✅ 스텝 각도 (도/step)
        # ────────────────────────────────────────────────────────

        chip = self.get_parameter('gpiochip').get_parameter_value().integer_value
        self.h = lg.gpiochip_open(chip)

        self.PIN_RELAY = self.get_parameter('relay_pin').get_parameter_value().integer_value
        self.PIN_PUL   = self.get_parameter('pul_pin').get_parameter_value().integer_value
        self.PIN_DIR   = self.get_parameter('dir_pin').get_parameter_value().integer_value

        self.ON_LEVEL  = 1 if self.get_parameter('active_high').get_parameter_value().bool_value else 0
        self.OFF_LEVEL = 1 - self.ON_LEVEL

        self.F_HZ  = float(self.get_parameter('freq_hz').get_parameter_value().integer_value)
        self.T_LOW = max(self.get_parameter('pulse_low_us').get_parameter_value().integer_value, 10) * 1e-6
        self.T_MIN = max(self.get_parameter('min_gap_us').get_parameter_value().integer_value, 6) * 1e-6

        self.MICROSTEP  = int(self.get_parameter('microstep').get_parameter_value().integer_value)
        self.STEP_ANGLE = float(self.get_parameter('step_angle').get_parameter_value().double_value)

        # 핀 모드
        lg.gpio_claim_output(self.h, self.PIN_RELAY, self.OFF_LEVEL)
        lg.gpio_claim_output(self.h, self.PIN_DIR, 0)
        lg.gpio_claim_output(self.h, self.PIN_PUL, 1)

        self.sub = self.create_subscription(String, '/arm/cmd', self.cb, 10)

        self.stop_event = threading.Event()
        self.th = None
        self.get_logger().info("[Arm] ready")

    # Hz → RPM 변환
    def hz_to_rpm(self, hz: float) -> float:
        steps_per_rev = (360.0 / self.STEP_ANGLE) * self.MICROSTEP
        return hz * 60.0 / steps_per_rev
        
    def cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd.startswith("up"):
            freq = self._parse_freq(cmd)
            self.start(cw=True, freq=freq)
        elif cmd.startswith("down"):
            freq = self._parse_freq(cmd)
            self.start(cw=False, freq=freq)
        elif cmd == "stop":
            self.stop()
        elif cmd == "release":
            self.release()
        else:
            self.get_logger().warn(f"[Arm] unknown cmd: {cmd}")

    def _parse_freq(self, cmd: str) -> float:
        try:
            if ":" in cmd:
                return float(cmd.split(":")[1])
        except Exception:
            pass
        return self.F_HZ  # 기본 주파수 유지

    def start(self, cw=True, freq=None):
        if self.th and self.th.is_alive():
            return
        if freq:
            self.F_HZ = freq
        self.stop_event.clear()
        self.th = threading.Thread(target=self._run, args=(cw,), daemon=True)
        self.th.start()
        rpm = self.hz_to_rpm(self.F_HZ)
        self.get_logger().info(
            f"[Arm] START {'UP' if cw else 'DOWN'} at {self.F_HZ:.1f} Hz ({rpm:.2f} rpm)"
        )

    def _run(self, cw: bool):
        # Enable
        lg.gpio_write(self.h, self.PIN_RELAY, self.ON_LEVEL)
        # Direction
        lg.gpio_write(self.h, self.PIN_DIR, 0 if cw else 1)

        period = 1.0 / self.F_HZ
        high_s = max(period - self.T_LOW, self.T_MIN)

        while not self.stop_event.is_set():
            # LOW
            lg.gpio_write(self.h, self.PIN_PUL, 0)
            time.sleep(self.T_LOW)
            # HIGH
            lg.gpio_write(self.h, self.PIN_PUL, 1)
            time.sleep(high_s)

        self.get_logger().info("[Arm] STOP (holding torque active)")

    def stop(self):
        self.stop_event.set()

    def release(self):
        self.stop()
        # torque free
        lg.gpio_write(self.h, self.PIN_RELAY, self.OFF_LEVEL)
        self.get_logger().info("[Arm] RELEASED")

    def destroy_node(self):
        try:
            self.stop_event.set()
            lg.gpio_write(self.h, self.PIN_RELAY, self.OFF_LEVEL)
            lg.gpiochip_close(self.h)
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = ArmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
