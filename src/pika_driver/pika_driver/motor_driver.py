import math
import queue
import re
import struct
import threading
import time

import numpy as np
import rclpy
import serial
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


class SendFlag:
    DISABLE = 10
    ENABLE = 11
    SET_ZERO = 12
    VELOCITY_CTRL = 13
    EFFORT_CTRL = 15
    POSITION_CTRL_MIT = 22
    POSITION_CTRL_POS_VEL = 23
    LIGHT_CTRL = 50
    VIBRATE_CTRL = 51


class Color:
    WHITE = 0
    RED = 1
    GREEN = 2
    BLUE = 3
    YELLOW = 4
    SIZE = 5


class Vibrate:
    NONE = 0
    ONE = 1
    SIZE = 2


class RosOperator(Node):
    def __init__(self):
        super().__init__('pika_high_speed_operator')
        self.get_logger().info('Initializing Pika High-Speed Operator...')

        # ------------------------------------------------------------------
        # 1) Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.serial_port_name = self.get_parameter("serial_port").value

        self.declare_parameter("joint_name", "center_joint")
        self.joint_name = self.get_parameter("joint_name").value

        self.declare_parameter("ctrl_rate", 100.0)
        self.ctrl_rate = float(self.get_parameter("ctrl_rate").value)
        self.ctrl_period = 1.0 / self.ctrl_rate

        self.declare_parameter("mit_mode", True)
        self.mit_mode = bool(self.get_parameter("mit_mode").value)

        # ------------------------------------------------------------------
        # 2) Regex
        # ------------------------------------------------------------------
        self.rad_pattern = re.compile(br'"rad"\s*:\s*([-+]?\d+(?:\.\d+)?)')
        self.pos_pattern = re.compile(br'"Position"\s*:\s*([-+]?\d+(?:\.\d+)?)')
        self.status_pattern = re.compile(br'"Status"\s*:\s*"0x([0-9A-Fa-f]{2})"')

        # ------------------------------------------------------------------
        # 3) Kinematics / LUT
        # ------------------------------------------------------------------
        self._dist_zero = self.get_distance(0.0)
        self._init_lut()

        # ------------------------------------------------------------------
        # 4) State
        # ------------------------------------------------------------------
        self.running = True
        self.enable = False

        self.distance = 0.0
        self.angle = 0.0

        self.target_pos = 0.0
        self.target_lock = threading.Lock()

        self._last_enable_send = 0.0

        # latest command only
        self.serial_queue = queue.Queue(maxsize=1)

        # stats
        self.rx_count = 0
        self.ctrl_count = 0
        self.tx_count = 0
        self.last_stat_time = time.monotonic()

        # ------------------------------------------------------------------
        # 5) Serial
        # ------------------------------------------------------------------
        try:
            # timeout=0 완전 non-blocking 대신 아주 작은 blocking timeout 사용
            self.serial = serial.Serial(
                port=self.serial_port_name,
                baudrate=460800,
                timeout=0.002,
            )
            self.get_logger().info(f"[SERIAL] Connected to {self.serial_port_name}")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            raise

        # ------------------------------------------------------------------
        # 6) Pub/Sub
        # ------------------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_js = self.create_publisher(JointState, "gripper/joint_state", qos)
        self.sub_js_ctrl = self.create_subscription(
            JointState,
            "gripper/joint_state_ctrl",
            self.joint_state_ctrl_handler,
            qos,
        )

        # ------------------------------------------------------------------
        # 7) Threads + Timers
        # ------------------------------------------------------------------
        self.recv_thread = threading.Thread(target=self.receiving_thread, daemon=True)
        self.send_thread = threading.Thread(target=self.serial_sending_thread, daemon=True)
        self.recv_thread.start()
        self.send_thread.start()

        # 제어를 subscriber callback이 아니라 timer에서 고정 주기로 수행
        self.control_timer = self.create_timer(self.ctrl_period, self.control_loop)
        self.stat_timer = self.create_timer(2.0, self.print_stats)

    # ----------------------------------------------------------------------
    # Kinematics
    # ----------------------------------------------------------------------
    def get_distance(self, angle: float) -> float:
        theta = (136.01 / 180.0) * math.pi - angle
        height = 0.0325 * math.sin(theta)
        term = max(0.0, (0.058 ** 2) - (height - 0.01456) ** 2)
        return math.sqrt(term) + 0.0325 * math.cos(theta)

    def _init_lut(self):
        self._lut_angles = np.linspace(0.0, 1.67, 2000)
        self._lut_widths = np.array([self.get_distance(a) for a in self._lut_angles])

    def get_angle_from_width(self, target_width: float) -> float:
        return float(np.interp(target_width, self._lut_widths, self._lut_angles))

    # ----------------------------------------------------------------------
    # ROS Callbacks
    # ----------------------------------------------------------------------
    def joint_state_ctrl_handler(self, msg: JointState):
        # subscriber callback에서는 최신 목표값만 저장
        if msg.position:
            pos = float(msg.position[-1])
        else:
            pos = 0.0

        pos = max(0.0, min(pos, 0.098))
        with self.target_lock:
            self.target_pos = pos

    def control_loop(self):
        # 제어는 고정 주기 timer에서만 수행
        self.ctrl_count += 1

        if not self.serial.is_open:
            return

        # enable 안 되어 있으면 너무 자주 보내지 말고 5Hz 정도로만 enable 요청
        if not self.enable:
            now = time.monotonic()
            if now - self._last_enable_send >= 0.2:
                self.send_serial_cmd(SendFlag.ENABLE, [0.0])
                self._last_enable_send = now
            return

        with self.target_lock:
            pos = self.target_pos

        target_width = pos * 0.5 + self._dist_zero - 0.005
        angle = self.get_angle_from_width(target_width)
        angle = max(0.0, min(angle, 1.67))

        flag = SendFlag.POSITION_CTRL_MIT if self.mit_mode else SendFlag.POSITION_CTRL_POS_VEL
        self.send_serial_cmd(flag, [angle])

    def print_stats(self):
        now = time.monotonic()
        dt = now - self.last_stat_time
        if dt <= 0.0:
            return

        rx_hz = self.rx_count / dt
        ctrl_hz = self.ctrl_count / dt
        tx_hz = self.tx_count / dt

        self.get_logger().info(
            f"RX:{rx_hz:.1f}Hz | CTRL:{ctrl_hz:.1f}Hz | TX:{tx_hz:.1f}Hz | Enable:{self.enable}"
        )

        self.rx_count = 0
        self.ctrl_count = 0
        self.tx_count = 0
        self.last_stat_time = now

    # ----------------------------------------------------------------------
    # Serial I/O
    # ----------------------------------------------------------------------
    def serial_sending_thread(self):
        while self.running:
            try:
                data = self.serial_queue.get(timeout=0.05)
            except queue.Empty:
                continue

            try:
                if self.serial.is_open:
                    self.serial.write(data)
                    self.tx_count += 1
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")

    def send_serial_cmd(self, cmd: int, values):
        packed = bytearray([cmd])
        for v in values:
            packed.extend(struct.pack('<f', float(v)))
        packed.extend(b'\r\n')
        data = bytes(packed)

        # 최신 명령만 유지
        try:
            while True:
                self.serial_queue.get_nowait()
        except queue.Empty:
            pass

        try:
            self.serial_queue.put_nowait(data)
        except queue.Full:
            pass

    def receiving_thread(self):
        """
        시리얼 수신 스레드
        - busy polling 최소화
        - 줄 단위 파싱
        - buffer 전체 findall() 대신 line별 search() 사용
        """
        buffer = b""

        while self.running:
            if not self.serial.is_open:
                time.sleep(0.05)
                continue

            try:
                # timeout=0.002 이므로 read가 약간 blocking 되며 CPU 점유를 줄여줌
                chunk = self.serial.read(self.serial.in_waiting or 1)
                if not chunk:
                    continue

                buffer += chunk

                # 너무 길어지면 뒤만 남김
                if len(buffer) > 4096:
                    buffer = buffer[-2048:]

                # 줄 단위 처리
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    line = line.strip()
                    if not line:
                        continue

                    self._parse_serial_line(line)

            except Exception as e:
                self.get_logger().warn(f"Serial read failed: {e}")
                time.sleep(0.01)

    def _parse_serial_line(self, line: bytes):
        rad_match = self.rad_pattern.search(line)
        pos_match = self.pos_pattern.search(line)
        status_match = self.status_pattern.search(line)

        if status_match:
            try:
                status_hex = status_match.group(1).decode('ascii')
                status_byte = int(status_hex, 16)
                self.enable = bool(status_byte & 0x40)
            except Exception:
                pass

        active_val = None

        if rad_match:
            try:
                active_val = float(rad_match.group(1))
            except Exception:
                active_val = None
        elif pos_match:
            try:
                active_val = float(pos_match.group(1))
            except Exception:
                active_val = None

        if active_val is None:
            return

        self.angle = active_val
        self.distance = 2.0 * (self.get_distance(active_val) - self._dist_zero)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        js.position = [float(self.distance)]
        self.pub_js.publish(js)
        self.rx_count += 1

    # ----------------------------------------------------------------------
    # Shutdown
    # ----------------------------------------------------------------------
    def destroy_node(self):
        self.running = False

        try:
            if hasattr(self, 'serial') and self.serial.is_open:
                self.serial.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosOperator()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()