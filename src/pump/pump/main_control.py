import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import gpiod
import time

CHIP = "/dev/gpiochip4"
GPIO_PUMP = 16
GPIO_VALVE_1 = 20
GPIO_VALVE_2 = 21

class MainControlNode(Node):
    def __init__(self):
        super().__init__('dual_valve_control_node')

        # ---- gpiod 라인 요청 ----
        self.chip = gpiod.Chip(CHIP)

        settings = gpiod.LineSettings(
            direction=gpiod.line.Direction.OUTPUT,
            output_value=gpiod.line.Value.INACTIVE  # 기본은 모두 OFF(안전)
        )
        config = {
            GPIO_PUMP: settings,
            GPIO_VALVE_1: settings,
            GPIO_VALVE_2: settings,
        }

        self.request = self.chip.request_lines(
            config=config,
            consumer='main_control'
        )

        # ---- 게이트/타임아웃 상태 초기화 (누락 보완) ----
        self.await_angle = False
        self._angle_timeout_timer = None
        self._angle_timeout_sec = 100.0

        # ---- 구독 ----
        self.sub_pump = self.create_subscription(
            String, 'pump_need_topic', self.pump_cb, 10
        )
        self.sub_valve = self.create_subscription(
            String, 'msg_publisher_topic', self.angle_gate_cb, 10
        )

        self.sub_stop = self.create_subscription(
            String, 'stop_topic', self.stop_all_cb, 10
        )

        # ---- 핸들러 매핑 ----
        self.handlers = {
            "upright": self.upright,
            "cannot_work": self.cannot_work,
            "upside_down": self.upside_down,
            "stop": self.cannot_work
        }

    # ===== 메시지 분류/실행 =====
    def angle_cb(self, msg: String):
        data_str = msg.data.strip().lower()

        if data_str == "stop":
            self.handlers["stop"]()
            return

        try:
            angle = float(data_str) % 360
        except ValueError:
            self.get_logger().warn(f"잘못된 데이터: {msg.data}")
            return

        if (0 <= angle < 45) or (315 <= angle < 360):
            state = "upright"
        elif 45 <= angle < 135:
            state = "cannot_work"
        elif 135 <= angle < 225:
            state = "upside_down"
        elif 225 <= angle < 315:
            state = "cannot_work"
        else:
            state = None

        if state and state in self.handlers:
            self.handlers[state]()
        else:
            self.get_logger().warn("Unknown state")

    # ===== 트리거: pump_need → 다음 각도 메시지 1건만 통과 =====
    def pump_cb(self, msg: String):
        if msg.data.strip().lower() == 'pump_need':
            self.await_angle = True
            self.get_logger().info("ready")

            # 기존 타이머 정리
            if self._angle_timeout_timer is not None:
                self._angle_timeout_timer.cancel()
                self._angle_timeout_timer = None

            # 타임아웃 타이머 생성 (누락 보완)
            self._angle_timeout_timer = self.create_timer(
                self._angle_timeout_sec, self._angle_timeout_cb
            )
        else:
            self.get_logger().warn(f'Unknown command: "{msg.data}"')

    def stop_all_cb(self, msg: String):
        if msg.data.strip().lower() == 'stop':
            self.request.set_value(GPIO_PUMP, gpiod.line.Value.INACTIVE)
            self.request.set_value(GPIO_VALVE_1, gpiod.line.Value.INACTIVE)
            self.request.set_value(GPIO_VALVE_2, gpiod.line.Value.INACTIVE)
            self.get_logger().info('STOP: pump OFF, valves OFF')

    def angle_gate_cb(self, msg: String):
        if not self.await_angle:
            return

        # 원샷
        self.await_angle = False

        if self._angle_timeout_timer is not None:
            self._angle_timeout_timer.cancel()
            self._angle_timeout_timer = None

        self.angle_cb(msg)

    def _angle_timeout_cb(self):
        # 지정 시간 내 각도/stop 미수신 시 게이트 닫기
        self.await_angle = False
        if self._angle_timeout_timer is not None:
            self._angle_timeout_timer.cancel()
            self._angle_timeout_timer = None
        self.get_logger().warn(f'{self._angle_timeout_sec:.0f}s 내 입력 없음: 게이트 취소')

    # ===== 동작 핸들러 =====
    def upright(self):
        self.request.set_value(GPIO_VALVE_1, gpiod.line.Value.INACTIVE)
        self.request.set_value(GPIO_VALVE_2, gpiod.line.Value.ACTIVE)
        self.get_logger().info('open_valve_2')

        time.sleep(2.0)
        self.request.set_value(GPIO_PUMP, gpiod.line.Value.ACTIVE)
        self.get_logger().info('pump ON')

    def cannot_work(self):
        self.request.set_value(GPIO_PUMP, gpiod.line.Value.INACTIVE)
        self.request.set_value(GPIO_VALVE_1, gpiod.line.Value.INACTIVE)
        self.request.set_value(GPIO_VALVE_2, gpiod.line.Value.INACTIVE)
        self.get_logger().info('cannot_work')

    def upside_down(self):
        self.request.set_value(GPIO_VALVE_1, gpiod.line.Value.ACTIVE)
        self.request.set_value(GPIO_VALVE_2, gpiod.line.Value.INACTIVE)
        self.get_logger().info('open_valve_1')

        time.sleep(2.0)
        self.request.set_value(GPIO_PUMP, gpiod.line.Value.ACTIVE)
        self.get_logger().info('pump ON')

    # ===== 정리 =====
    def destroy_node(self):
        try:
            self.request.set_value(GPIO_PUMP, gpiod.line.Value.INACTIVE)
            self.request.set_value(GPIO_VALVE_1, gpiod.line.Value.INACTIVE)
            self.request.set_value(GPIO_VALVE_2, gpiod.line.Value.INACTIVE)
            self.request.release()
        except Exception as e:
            self.get_logger().warn(f'Cleanup error: {e}')
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MainControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()