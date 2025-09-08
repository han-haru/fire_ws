#!/usr/bin/env python3
import os, sys, tty, termios, select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

HELP = """
=== Control Keys ===
Blade: w=+10%, s=-10%, a=reverse, d=toggle ON/OFF
Arm:   i=up, k=down, space=stop, l=release
       u=RPM +10,  o=RPM -10   (현재 RPM은 콘솔에 표시)
Pump:  y=정방향, j=역방향, m=stop   (※ u 키와 충돌 피해서 y로 변경)
Exit:  q
====================
"""

# ARM 기어 파라미터를 teleop에서 몰라도 되지만,
# 콘솔에 참고로 현재 rpm만 찍어준다 (Hz 환산은 ArmNode에서 처리)
class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.pub_blade = self.create_publisher(Float32, '/blade/cmd', 10)
        self.pub_bctrl = self.create_publisher(String,  '/blade/ctrl', 10)
        self.pub_arm   = self.create_publisher(String,  '/arm/cmd',   10)
        self.pub_pump  = self.create_publisher(String,  '/pump/cmd',  10)

        # Blade 속도 %
        self.percent = 0.0
        # ARM RPM (ArmNode 기본 400Hz ≈ 120 rpm @1.8°,1x 기준)
        self.arm_rpm = 30.0

        print(HELP)
        print(f"[Arm] init rpm = {self.arm_rpm:.1f} rpm")
        # 시작 시 현재 rpm을 ArmNode에 알려두고 시작 (선택)
        self.pub_arm.publish(String(data=f"rpm:{self.arm_rpm:.1f}"))

    def _open_tty(self):
        if sys.stdin.isatty():
            return sys.stdin, sys.stdin.fileno(), None
        try:
            f = open('/dev/tty', 'rb', buffering=0)
            return f, f.fileno(), f
        except Exception as e:
            self.get_logger().error(f"No TTY available: {e}")
            return None, None, None

    def keyloop(self):
        f, fd, extra_file = self._open_tty()
        if fd is None:
            rclpy.shutdown()
            return

        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while rclpy.ok():
                r, _, _ = select.select([fd], [], [], 0.1)
                if not r:
                    rclpy.spin_once(self, timeout_sec=0.0)
                    continue

                ch = os.read(fd, 1).decode(errors='ignore')
                if not ch:
                    continue

                if ch == 'q':
                    break

                # --- Blade ---
                # 바꿀 부분만 발췌
                elif ch == 'w':
                    self.percent = min(float(self.percent) + 10.0, 100.0)
                    self.pub_blade.publish(Float32(data=float(self.percent)))
                elif ch == 's':
                    self.percent = max(float(self.percent) - 10.0, -100.0)
                    self.pub_blade.publish(Float32(data=float(self.percent)))
                elif ch == 'a':
                    self.pub_bctrl.publish(String(data="reverse"))
                elif ch == 'd':
                    self.pub_bctrl.publish(String(data="toggle"))
                    self.get_logger().info(f"현재 속도: {self.percent}")

                # --- Arm: 동작 ---
                elif ch == 'i':
                    self.pub_arm.publish(String(data="up"))
                elif ch == 'k':
                    self.pub_arm.publish(String(data="down"))
                elif ch == ' ':
                    self.pub_arm.publish(String(data="stop"))
                elif ch == 'l':
                    self.pub_arm.publish(String(data="release"))

                # --- Arm: RPM 조절 ---
                elif ch == 'u':
                    self.arm_rpm = min(self.arm_rpm + 10, 300)
                    print(f"[Arm] rpm = {self.arm_rpm:.1f}")
                    self.pub_arm.publish(String(data=f"rpm:{self.arm_rpm:.1f}"))
                elif ch == 'o':
                    self.arm_rpm = max(self.arm_rpm - 10, 10)
                    print(f"[Arm] rpm = {self.arm_rpm:.1f}")
                    self.pub_arm.publish(String(data=f"rpm:{self.arm_rpm:.1f}"))

                # --- Pump (키 충돌 피해서 upright=y 로 변경) ---
                elif ch == 'y':
                    self.pub_pump.publish(String(data="upright"))
                elif ch == 'h':
                    self.pub_pump.publish(String(data="upside_down"))
                elif ch == 'j':
                    self.pub_pump.publish(String(data="stop"))

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            if extra_file is not None:
                extra_file.close()

def main():
    rclpy.init()
    node = TeleopNode()
    node.keyloop()
    node.destroy_node()
    rclpy.shutdown()
