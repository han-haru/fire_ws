#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import RPi.GPIO as GPIO

class BladeNode(Node):
    def __init__(self):
        super().__init__('blade_node')
        # ── 파라미터 ───────────────────────────────────────────
        self.declare_parameter('pwm_pin', 19)
        self.declare_parameter('dir_pin', 26)
        self.declare_parameter('pwm_hz', 500)
        self.declare_parameter('dir_invert', False)

        # 램프(가속/감속) 파라미터
        self.declare_parameter('ramp_pct_per_s', 90.0)      # 초당 %p 변화량
        self.declare_parameter('ramp_dt', 0.01)             # 램프 타이머 주기(s)
        self.declare_parameter('dir_flip_deadband_pct', 3.0) # DIR 바꾸기 전 듀티 허용치
        self.declare_parameter('stop_eps_pct', 0.5)          # 소프트 스톱 임계(%)

        # ── 파라미터 로드 ─────────────────────────────────────
        self.PWM_PIN   = int(self.get_parameter('pwm_pin').value)
        self.DIR_PIN   = int(self.get_parameter('dir_pin').value)
        self.PWM_HZ    = int(self.get_parameter('pwm_hz').value)
        self.DIR_INV   = bool(self.get_parameter('dir_invert').value)

        self.RAMP_RATE = float(self.get_parameter('ramp_pct_per_s').value)
        self.RAMP_DT   = float(self.get_parameter('ramp_dt').value)
        self.DIR_DB    = float(self.get_parameter('dir_flip_deadband_pct').value)
        self.STOP_EPS  = float(self.get_parameter('stop_eps_pct').value)

        # ── GPIO init (RPi.GPIO) ───────────────────────────────
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PWM_PIN, GPIO.OUT)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        self._pwm = GPIO.PWM(self.PWM_PIN, self.PWM_HZ)
        self._pwm.start(0)

        # ── 상태 ───────────────────────────────────────────────
        self.target_percent = 0.0   # 명령 목표(-100~100)
        self.actual_percent = 0.0   # 현재 적용값(램프)
        self.on = False             # 토글 ON/OFF
        self.dir_state_forward = True

        # ── 구독 ───────────────────────────────────────────────
        self.create_subscription(Float32, '/blade/cmd', self.cb_speed, 10)
        self.create_subscription(String,  '/blade/ctrl', self.cb_ctrl, 10)

        # 램프 타이머
        self.create_timer(self.RAMP_DT, self._ramp_tick)

        self.get_logger().info(
            f"[Blade] ready (RPi.GPIO PWM {self.PWM_HZ} Hz, ramp={self.RAMP_RATE:.1f}%/s)"
        )

    # ── 내부 헬퍼 ──────────────────────────────────────────────
    def _set_dir_level(self, forward: bool):
        level = 0 if forward else 1
        if self.DIR_INV:
            level = 1 - level
        GPIO.output(self.DIR_PIN, level)

    def _apply_output(self):
        """현재 actual_percent를 PWM에 반영.
           on=False여도 ramp가 0% 근처까지 내려갈 때까지는 듀티를 유지(소프트 스톱)."""
        duty = abs(self.actual_percent)

        # 완전히 멈출 임계 이하이면 PWM 0
        if duty <= self.STOP_EPS:
            self._pwm.ChangeDutyCycle(0.0)
            return

        # 그 외에는 on 여부와 무관하게 램프 값 그대로 적용 (부드러운 감속/가속)
        self._pwm.ChangeDutyCycle(duty)

    # ── 램프(가속/감속/리버스) 로직 ────────────────────────────
    def _ramp_tick(self):
        # 1) 효과적 목표: OFF면 0으로 수렴
        eff_target = self.target_percent if self.on else 0.0

        # 2) 방향 전환 보호(0% 근처에서만 DIR 스위칭)
        if (self.actual_percent > 0 and eff_target < 0) or (self.actual_percent < 0 and eff_target > 0):
            if abs(self.actual_percent) > self.DIR_DB:
                eff_target = 0.0

        # 3) 한 틱 이동량 제한
        max_step = self.RAMP_RATE * self.RAMP_DT
        delta = eff_target - self.actual_percent
        if abs(delta) <= max_step:
            self.actual_percent = eff_target
        else:
            self.actual_percent += max_step if delta > 0 else -max_step

        # 4) DIR 갱신: 0% 근처에서만 변경
        desired_forward = (eff_target >= 0.0)
        if abs(self.actual_percent) <= self.DIR_DB:
            if self.dir_state_forward != desired_forward:
                self.dir_state_forward = desired_forward
                self._set_dir_level(self.dir_state_forward)

        # 5) 출력 반영
        self._apply_output()

    # ── 콜백 ───────────────────────────────────────────────────
    def cb_speed(self, msg: Float32):
        try:
            val = float(msg.data)
        except Exception:
            self.get_logger().error("[Blade] bad Float32 data")
            return
        self.target_percent = max(-100.0, min(100.0, val))
        self.get_logger().info(f"[Blade] target → {self.target_percent:.1f}% "
                               f"({'ON' if self.on else 'OFF'})")

    def cb_ctrl(self, msg: String):
        cmd = str(msg.data).strip().lower()
        if cmd == "toggle":
            self.on = not self.on
            if not self.on:
                self.get_logger().info("[Blade] TOGGLE → OFF (soft stop)")
            else:
                self.get_logger().info("[Blade] TOGGLE → ON")
        elif cmd == "reverse":
            # 목표 부호만 반전 (램프가 0% 통과 후 안전하게 DIR 변경)
            self.target_percent = -self.target_percent
            self.get_logger().info(f"[Blade] REVERSE cmd → target {self.target_percent:.1f}%")
        else:
            self.get_logger().warn(f"[Blade] unknown ctrl: {cmd}")

    # ── 종료 정리 ──────────────────────────────────────────────
    def destroy_node(self):
        try:
            self._pwm.ChangeDutyCycle(0)
            self._pwm.stop()
            GPIO.cleanup()
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
