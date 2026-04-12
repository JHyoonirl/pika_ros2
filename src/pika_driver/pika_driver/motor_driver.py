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


class RosOperator(Node):
    def __init__(self):
        super().__init__('pika_high_speed_operator')
        self.get_logger().info('Initializing Pika High-Speed Operator (Angle Control Mode)...')

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
        # 3) Kinematics (Feedback 전용)
        # ------------------------------------------------------------------
        self._dist_zero = self.get_distance(0.0)

        # ------------------------------------------------------------------
        # 4) State
        # ------------------------------------------------------------------
        self.running = True
        self.enable = False

        self.distance = 0.0
        self.angle = 0.0

        self.target_angle = 0.0  # 목표 위치를 '각도'로 관리
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

        self.control_timer = self.create_timer(self.ctrl_period, self.control_loop)
        self.stat_timer = self.create_timer(2.0, self.print_stats)

        # 시작 시 라이트를 파란색으로 변경
        time.sleep(0.1)
        self.send_serial_cmd(SendFlag.LIGHT_CTRL, [Color.BLUE])
    def get_distance(self, angle: float) -> float:
        """각도를 거리로 변환 (피드백 출력용)"""
        theta = (136.01 / 180.0) * math.pi - angle
        height = 0.0325 * math.sin(theta)
        term = max(0.0, (0.058 ** 2) - (height - 0.01456) ** 2)
        return math.sqrt(term) + 0.0325 * math.cos(theta)

    # ----------------------------------------------------------------------
    # ROS Callbacks
    # ----------------------------------------------------------------------
    def joint_state_ctrl_handler(self, msg: JointState):
        """Subscriber로부터 목표 각도(rad)를 직접 받음"""
        if msg.position:
            target = float(msg.position[-1])
        else:
            target = 0.0

        # Pika 하드웨어 각도 가동 범위 제한 (0.0 ~ 1.67 rad)
        # target = max(0.0, min(target, 3.14))
        # self.get_logger().info(f"Received target angle: {target:.3f} rad")
        
        with self.target_lock:
            self.target_angle = target

    def control_loop(self):
        self.ctrl_count += 1
        if not self.serial.is_open:
            return

        if not self.enable:
            now = time.monotonic()
            if now - self._last_enable_send >= 0.2:
                self.send_serial_cmd(SendFlag.ENABLE, [0.0])
                self._last_enable_send = now
            return

        with self.target_lock:
            angle_to_send = self.target_angle

        # 기구학 변환 없이 각도를 즉시 시리얼로 전송
        flag = SendFlag.POSITION_CTRL_MIT if self.mit_mode else SendFlag.POSITION_CTRL_POS_VEL
        # self.get_logger().info(f"Sending angle command: {angle_to_send:.3f} rad with flag {flag}")
        self.send_serial_cmd(flag, [angle_to_send])
        # now = time.monotonic()
        # if self.ctrl_count % 10 == 0: # 너무 자주 보낼 필요 없으므로 10번에 한 번꼴
        #     self.send_serial_cmd(SendFlag.DISABLE, [0.0])

    def print_stats(self):
        now = time.monotonic()
        dt = now - self.last_stat_time
        if dt <= 0.0:
            return
        rx_hz = self.rx_count / dt
        ctrl_hz = self.ctrl_count / dt
        tx_hz = self.tx_count / dt

        self.get_logger().info(
            f"RX:{rx_hz:.1f}Hz | CTRL:{ctrl_hz:.1f}Hz | TX:{tx_hz:.1f}Hz | Angle:{self.angle:.3f}rad | En:{self.enable}"
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
        buffer = b""
        while self.running:
            if not self.serial.is_open:
                time.sleep(0.05)
                continue
            try:
                chunk = self.serial.read(self.serial.in_waiting or 1)
                if not chunk:
                    continue
                buffer += chunk
                if len(buffer) > 4096:
                    buffer = buffer[-2048:]
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
        # 피드백 출력을 위해 거리를 계산만 해둠
        self.distance = 2.0 * (self.get_distance(active_val) - self._dist_zero)
        # self.get_logger().info(f"Angle recorded: {self.angle}")

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        # 피드백 position에도 각도를 넣어 발행 (로봇 팔과 일관성 유지)
        js.position = [float(self.angle)]
        self.pub_js.publish(js)
        self.rx_count += 1

    def destroy_node(self):
        """노드 종료 시 안전하게 하드웨어를 정리하고 상태를 표시"""
        self.get_logger().info("Shutting down... Disabling motor and setting light to GREEN.")
        self.running = False

        try:
            if hasattr(self, 'serial') and self.serial.is_open:
                # 1. 모터 비활성화 명령 (DISABLE = 10)
                # 종료 후에도 모터가 뜨거워지거나 손상되는 것을 방지합니다.
                disable_pkt = bytearray([SendFlag.DISABLE])
                disable_pkt.extend(struct.pack('<f', 0.0))
                disable_pkt.extend(b'\r\n')
                self.serial.write(bytes(disable_pkt))
                
                # 2. 종료 라이트 명령 (GREEN = 2)
                # 작업이 성공적으로 종료되었음을 알리는 신호입니다.
                light_pkt = bytearray([SendFlag.LIGHT_CTRL])
                light_pkt.extend(struct.pack('<f', float(Color.GREEN)))
                light_pkt.extend(b'\r\n')
                self.serial.write(bytes(light_pkt))
                
                # 3. 버퍼 비우기 및 대기
                # 명령이 실제로 날아갈 시간을 벌어줍니다.
                self.serial.flush()
                time.sleep(0.1) 
                
                self.serial.close()
                self.get_logger().info("Successfully cleaned up hardware states.")
        except Exception as e:
            self.get_logger().error(f"Error during shutdown process: {e}")

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