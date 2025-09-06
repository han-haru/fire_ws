import os, sys, tty, termios, select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

HELP = """
=== Control Keys ===
Blade: w=+10%, s=-10%, a=reverse, d=OFF(0%)
Arm:   i=up, k=down, space=stop, l=release
       o=arm speed UP (+100Hz), p=arm speed DOWN (-100Hz)
Pump:  u=정방향, j=역방향, m=stop
Exit:  q
====================
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.pub_blade = self.create_publisher(Float32, '/blade/cmd', 10)
        self.pub_arm   = self.create_publisher(String,  '/arm/cmd',   10)
        self.pub_pump  = self.create_publisher(String,  '/pump/cmd',  10)

        # Blade
        self.percent = 0.0
        # Arm speed (Hz 단위)
        self.arm_speed = 400  

        # Arm 파라미터 (ArmNode와 동일하게 맞춰줌)
        self.declare_parameter('microstep', 8)     # 기본 8분할
        self.declare_parameter('step_angle', 1.8)  # 기본 1.8도 스텝모터
        self.microstep = int(self.get_parameter('microstep').get_parameter_value().integer_value)
        self.step_angle = float(self.get_parameter('step_angle').get_parameter_value().double_value)

        print(HELP)

    def hz_to_rpm(self, hz: float) -> float:
        steps_per_rev = (360.0 / self.step_angle) * self.microstep
        return hz * 60.0 / steps_per_rev

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
                elif ch == 'w':
                    self.percent = min(self.percent + 10, 100)
                    self.pub_blade.publish(Float32(data=self.percent))
                elif ch == 's':
                    self.percent = max(self.percent - 10, -100)
                    self.pub_blade.publish(Float32(data=self.percent))
                elif ch == 'a':
                    self.percent = -self.percent
                    self.pub_blade.publish(Float32(data=self.percent))
                elif ch == 'd':
                    self.percent = 0.0
                    self.pub_blade.publish(Float32(data=0.0))
                elif ch == 'i':  # Arm UP (현재 속도)
                    self.pub_arm.publish(String(data=f"up:{self.arm_speed}"))
                elif ch == 'k':  # Arm DOWN (현재 속도)
                    self.pub_arm.publish(String(data=f"down:{self.arm_speed}"))
                elif ch == ' ':  # Arm STOP
                    self.pub_arm.publish(String(data="stop"))
                elif ch == 'l':  # Arm RELEASE
                    self.pub_arm.publish(String(data="release"))
                elif ch == 'o':  # Arm speed up
                    self.arm_speed += 100
                    rpm = self.hz_to_rpm(self.arm_speed)
                    print(f"[Arm] speed = {self.arm_speed} Hz ({rpm:.2f} rpm)")
                elif ch == 'p':  # Arm speed down
                    self.arm_speed = max(50, self.arm_speed - 100)
                    rpm = self.hz_to_rpm(self.arm_speed)
                    print(f"[Arm] speed = {self.arm_speed} Hz ({rpm:.2f} rpm)")
                elif ch == 'u':
                    self.pub_pump.publish(String(data="upright"))
                elif ch == 'j':
                    self.pub_pump.publish(String(data="upside_down"))
                elif ch == 'm':  # Pump STOP
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
