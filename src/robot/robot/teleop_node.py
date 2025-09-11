#!/usr/bin/env python3
import os, sys, tty, termios, select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist

HELP = """
=== Control Keys ===
Cater: ↑=전진, ↓=후진, ←=왼쪽 회전, →=오른쪽 회전
        z=속도 절대값 증가, x=속도 절대값 감소; lx+=0.02, az=0.15
Blade: w=+10%, s=-10%, a=reverse, d=toggle ON/OFF
Arm:   j=up, k=down, i=stop, l=release
       u=RPM +10,  o=RPM -10   (현재 RPM은 콘솔에 표시)
Pump:  b=정방향, n=역방향, m=stop   (※ u 키와 충돌 피해서 y로 변경)
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
        self.pub_cmdvel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Blade 속도 %
        self.percent = 0.0
        # ARM RPM (ArmNode 기본 400Hz ≈ 120 rpm @1.8°,1x 기준)
        self.arm_rpm = 30.0
        # Cmd Vel parameter
        self.linear_speed = 0.01 # m/s  maximum 0.26
        self.angular_speed = 0.1 # rad/s; maximum 1.04
        self.linear_add = 0.02
        self.angular_add = 0.05 

        self.last_cmd = None # 마지막 명령 저장

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

    def send_cmdvel(self, lx, az):
        twist = Twist()
        twist.linear.x = lx
        twist.angular.z = az
        self.pub_cmdvel.publish(twist)

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
                
                if ch == '\x1b':  # ESC 시작 (방향키 등)
                    seq = os.read(fd, 2).decode(errors='ignore')  # '['와 그 뒤 글자
                    ch += seq

                if ch == 'q':
                    break

                # --- Caterpillar Cmd_vel ---
                elif ch == ' ': # space
                    self.send_cmdvel(0.0, 0.0)
                    self.linear_speed = 0.0
                    self.angular_speed = 0.0
                    self.get_logger().info("캐터필러 중지!!!")

                elif ch == '\x1b[A': # ↑
                    self.send_cmdvel(self.linear_speed, 0.0)
                    self.get_logger().info(f"캐터필러 현재 선속도: {self.linear_speed} m/s")
                    self.last_cmd = ("linear", +1)
                elif ch == '\x1b[B': # ↓
                    self.send_cmdvel(-self.linear_speed, 0.0)
                    self.get_logger().info(f"캐터필러 현재 선속도: {-self.linear_speed} m/s")
                    self.last_cmd = ("linear", -1)
                elif ch == '\x1b[D': # ←
                    self.send_cmdvel(0.0, self.angular_speed)
                    self.get_logger().info(f"캐터필러 현재 회전속도: {self.angular_speed} rad/s")
                    self.last_cmd = ("angular", +1)
                elif ch == '\x1b[C': # →
                    self.send_cmdvel(0.0, -self.angular_speed)
                    self.get_logger().info(f"캐터필러 현재 회전속도: {-self.angular_speed} rad/s")
                    self.last_cmd = ("angular", -1)

                # --- Cmd_vel 속도 증감 (절대값 기준) ---
                elif ch == 'z': # 속도 증가
                    self.linear_speed += self.linear_add
                    self.angular_speed += self.angular_add
                    if self.last_cmd:
                        mode, sign = self.last_cmd
                        if mode == "linear":
                            self.send_cmdvel(sign * self.linear_speed, 0.0)
                            self.get_logger().info(f"캐터필러 현재 선속도: {sign * self.linear_speed} m/s")
                        else:
                            self.send_cmdvel(0.0, sign * self.angular_speed)
                            self.get_logger().info(f"캐터필러 현재 회전속도: {sign * self.linear_speed} rad/s")
                elif ch == 'x': # 속도 감소
                    self.linear_speed = max(0.0, self.linear_speed - self.linear_add)
                    self.angular_speed = max(0.0, self.angular_speed - self.angular_add)
                    if self.last_cmd:
                        mode, sign = self.last_cmd
                        if mode == "linear":
                            self.send_cmdvel(sign * self.linear_speed, 0.0)
                            self.get_logger().info(f"캐터필러 현재 선속도: {sign * self.linear_speed} m/s")
                        else:
                            self.send_cmdvel(0.0, sign * self.angular_speed)
                            self.get_logger().info(f"캐터필러 현재 회전속도: {sign * self.linear_speed} rad/s")

                # --- Blade ---
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
                elif ch == 'j':
                    self.pub_arm.publish(String(data="up"))
                elif ch == 'k':
                    self.pub_arm.publish(String(data="down"))
                elif ch == 'i':
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
                elif ch == 'b':
                    self.pub_pump.publish(String(data="valve1"))
                elif ch == 'n':
                    self.pub_pump.publish(String(data="valve2"))
                elif ch == 'm':
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
