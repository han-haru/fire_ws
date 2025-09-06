import time
from typing import List

import numpy as np
from scipy.optimize import curve_fit
import serial

import rclpy
from rclpy.node import Node
from co_msg.msg import CoMonitorEvent


def exp_func(x, a, b):
    return a * np.exp(b * x)


def r_squared(y: np.ndarray, y_fit: np.ndarray) -> float:
    ss_res = float(np.sum((y - y_fit) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    return 1.0 - ss_res / ss_tot if ss_tot != 0.0 else float("nan")


class CoMonitorNode(Node):
    """
    - 아두이노가 출력하는 12개 Δ값(탭 구분, 1Hz)을 읽어서 처리
    - 코너: indices [0,5,6,11] 임계치 이상이면 corner 이벤트 퍼블리시
    - Group1: [1,2,3,4], Group2: [7,8,9,10]
      · 트리거 발생 시 모니터 기간만큼 수집 → 평균 → 지수피팅 → R² 계산 → fit 이벤트 퍼블리시
    - 모든 이벤트는 /co_monitor/event 하나의 토픽으로 전달(type 필드로 구분)
    """

    def __init__(self):
        super().__init__("co_monitor_node")

        # ---- 파라미터 (필요 시 런타임에 --ros-args -p 로 덮어쓰기) ----
        self.declare_parameter("port", "/dev/ttyACM0")      # 환경에 맞게 기본값 ACM0로
        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("timeout", 1.0)              # serial readline timeout
        self.declare_parameter("log_threshold", 200)        # 코너 & 트리거 임계치
        self.declare_parameter("monitor_duration", 10)      # 트리거 시 수집 샘플 개수(프레임 수)
        self.declare_parameter("trigger_timeout_sec", 30.0) # 트리거 타임아웃

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = int(self.get_parameter("baudrate").get_parameter_value().integer_value)
        self.timeout = float(self.get_parameter("timeout").get_parameter_value().double_value)
        self.log_threshold = int(self.get_parameter("log_threshold").get_parameter_value().integer_value)
        self.monitor_duration = int(self.get_parameter("monitor_duration").get_parameter_value().integer_value)
        self.trigger_timeout_sec = float(self.get_parameter("trigger_timeout_sec").get_parameter_value().double_value)

        # ---- 그룹 정의 ----
        self.group3_indices = [0, 5, 6, 11]   # Corner: FR, RR, FL, RL
        self.group1_indices = [1, 2, 3, 4]    # Center group 1
        self.group2_indices = [7, 8, 9, 10]   # Center group 2
        self.labels1 = [f"CO{i+1}" for i in self.group1_indices]
        self.labels2 = [f"CO{i+1}" for i in self.group2_indices]

        # 코너 센서 위치 설명(로그/메시지용)
        self.sensor_log_map = {
            0: "Front_Right (CO1)",
            5: "Rear_Right (CO6)",
            6: "Front_Left (CO7)",
            11: "Rear_Left (CO12)"
        }

        # ---- 퍼블리셔 (단일 토픽) ----
        self.event_pub = self.create_publisher(CoMonitorEvent, "co_monitor/event", 10)

        # ---- 트리거 상태 ----
        self.trigger1 = False
        self.trigger2 = False
        self.trigger_time1 = None
        self.trigger_time2 = None
        self.collected1: List[List[int]] = [[] for _ in self.group1_indices]
        self.collected2: List[List[int]] = [[] for _ in self.group2_indices]

        # ---- 시리얼 ----
        self.ser = None
        self._open_serial()

        # 50ms 주기로 폴링 (아두이노 출력이 1Hz여도 여유 있게 읽기 위함)
        self.timer = self.create_timer(0.05, self._spin_once)

        self.get_logger().info(
            f"Serial='{self.port}' {self.baud}bps | "
            f"threshold={self.log_threshold}, monitor_duration={self.monitor_duration}, "
            f"timeout={self.timeout}s"
        )

    # =============== 유틸 ===============
    def _new_event(self, etype: int) -> CoMonitorEvent:
        msg = CoMonitorEvent()
        msg.type = etype
        return msg

    # =============== Serial ===============
    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            self.get_logger().info(f"Opened serial port: {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial '{self.port}': {e}")
            self.ser = None

    # =============== Main loop ===============
    def _spin_once(self):
        if self.ser is None:
            # 주기적으로 재시도
            self._open_serial()
            return
        try:
            raw = self.ser.readline().decode(errors="ignore").strip()
            if not raw:
                return

            parts = raw.split("\t")
            if len(parts) != 12:
                return

            try:
                values = [int(p) for p in parts]
            except ValueError:
                return

            now = time.time()

            # (1) Corner 이벤트
            for idx in self.group3_indices:
                val = values[idx]
                if val >= self.log_threshold:
                    pos = self.sensor_log_map.get(idx, f"Idx{idx}")
                    evt = self._new_event(CoMonitorEvent.TYPE_CORNER)
                    evt.pos = pos
                    self.event_pub.publish(evt)
                    self.get_logger().info(f"[Corner] {pos}")

            # (2) 그룹 처리
            self._process_group(
                group_name="G1",
                indices=self.group1_indices,
                labels=self.labels1,
                vals=[values[i] for i in self.group1_indices],
                trigger_flag_attr="trigger1",
                trigger_time_attr="trigger_time1",
                collected_attr="collected1",
                now=now
            )
            self._process_group(
                group_name="G2",
                indices=self.group2_indices,
                labels=self.labels2,
                vals=[values[i] for i in self.group2_indices],
                trigger_flag_attr="trigger2",
                trigger_time_attr="trigger_time2",
                collected_attr="collected2",
                now=now
            )

        except Exception as e:
            self.get_logger().error(f"spin_once error: {e}")

    # 공통 그룹 처리 로직
    def _process_group(self, group_name: str, indices: List[int], labels: List[str],
                       vals: List[int], trigger_flag_attr: str, trigger_time_attr: str,
                       collected_attr: str, now: float):

        trigger_flag = getattr(self, trigger_flag_attr)
        trigger_time = getattr(self, trigger_time_attr)
        collected = getattr(self, collected_attr)

        # Trigger ON
        if not trigger_flag and any(v >= self.log_threshold for v in vals):
            setattr(self, trigger_flag_attr, True)
            setattr(self, trigger_time_attr, now)
            setattr(self, collected_attr, [[] for _ in indices])

            src_local = int(np.argmax(vals))
            src_idx = indices[src_local]
            src_label = f"CO{src_idx+1}"
            src_val = int(vals[src_local])

            # 이벤트 퍼블리시
            evt = self._new_event(CoMonitorEvent.TYPE_TRIGGER)
            self.event_pub.publish(evt)

            self.get_logger().info(f"[{group_name}] Trigger ON")

            trigger_flag = True
            trigger_time = now
            collected = getattr(self, collected_attr)

        # 수집/완료/타임아웃
        if trigger_flag:
            for i in range(len(indices)):
                collected[i].append(vals[i])

            # 수집 완료 → 평균→피팅→R²
            if len(collected[0]) >= self.monitor_duration:
                avg = [float(np.mean(collected[i])) for i in range(len(indices))]
                x = np.arange(len(avg))
                y = np.array(avg, dtype=float)

                try:
                    popt, _ = curve_fit(exp_func, x, y, p0=(1.0, 0.1), maxfev=10000)
                    y_fit = exp_func(x, *popt)
                    r2 = r_squared(y, y_fit)
                except Exception as e:
                    self.get_logger().warn(f"[{group_name}] Curve fit failed: {e}")
                    r2 = float("nan")

                avg_round = [float(f"{v:.1f}") for v in avg]  # float32 호환성 고려한 반올림
                r2_disp = "N/A" if not (r2 == r2) else f"{r2:.3f}"
                self.get_logger().info(f"[{group_name}] R²={r2_disp}")

                # FIT 이벤트 퍼블리시
                evt = self._new_event(CoMonitorEvent.TYPE_FIT)
                evt.r2 = float("nan") if not (r2 == r2) else float(r2)
                self.event_pub.publish(evt)

                # 리셋
                setattr(self, trigger_flag_attr, False)
                setattr(self, trigger_time_attr, None)
                setattr(self, collected_attr, [[] for _ in indices])
                return

            # 타임아웃
            if (now - (trigger_time or now)) > self.trigger_timeout_sec:
                self.get_logger().info(f"[{group_name}] Timeout, clearing buffer")
                setattr(self, trigger_flag_attr, False)
                setattr(self, trigger_time_attr, None)
                setattr(self, collected_attr, [[] for _ in indices])

    # 종료 시 시리얼 닫기
    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CoMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()