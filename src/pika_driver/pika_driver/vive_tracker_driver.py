#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
- pysurvive 기반으로 Vive Tracker pose 수집
- tracker serial number(LHR-...) 기준으로 pose publish
- 1개 또는 2개 tracker 모두 지원
"""

import sys
import time
import math
import queue
import threading
import logging
import ctypes
import ctypes.util
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, List

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from pika_driver.utils.pose_utils import (
    xyzQuaternion2matrix,
    xyzrpy2Mat,
    matrixToXYZQuaternion,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("pika.vive_tracker_driver")

try:
    print(f"ROS2 is using: {sys.executable}")
    import pysurvive
except ImportError as e:
    raise ImportError("pysurvive가 설치되어 있지 않습니다. 먼저 설치하세요.") from e


@dataclass
class PoseData:
    device_name: str
    serial_number: Optional[str]
    timestamp: float
    position: List[float]
    rotation: List[float]
    raw_position: List[float]
    raw_rotation: List[float]


class ViveTrackerDriver:
    def __init__(
        self,
        config_path: Optional[str] = None,
        lh_config: Optional[str] = None,
        args: Optional[List[str]] = None,
    ):
        self.config_path = config_path
        self.lh_config = lh_config
        self.args = args if args is not None else []

        self.running = False
        self.context = None

        self.pose_queue = queue.Queue(maxsize=100)
        self.data_lock = threading.Lock()

        self.latest_poses: Dict[str, PoseData] = {}
        self.devices_info: Dict[str, Dict] = {}
        self.serial_to_name: Dict[str, str] = {}
        self.name_to_serial: Dict[str, str] = {}

        self.collector_thread = None
        self.processor_thread = None
        self.monitor_thread = None

        self.libsurvive = self._load_libsurvive()

    # -------------------------------------------------
    # libsurvive serial helper
    # -------------------------------------------------
    def _find_libsurvive(self) -> str:
        env_path = None
        try:
            env_path = str(Path(__file__).resolve().parent / "libsurvive")
        except Exception:
            pass

        found = ctypes.util.find_library("survive")
        if found:
            return found

        candidates = [
            "libsurvive.so",
            "libsurvive.so.0",
            "/usr/local/lib/libsurvive.so",
            "/usr/local/lib/libsurvive.so.0",
            "/usr/lib/libsurvive.so",
            "/usr/lib/libsurvive.so.0",
        ]

        if env_path:
            local_dir = Path(env_path)
            if local_dir.exists():
                for pattern in ["**/libsurvive.so", "**/libsurvive.so.0"]:
                    for p in local_dir.glob(pattern):
                        if p.exists():
                            return str(p)

        for cand in candidates:
            if Path(cand).exists():
                return cand

        raise RuntimeError("libsurvive 공유 라이브러리를 찾지 못했습니다.")

    def _load_libsurvive(self):
        try:
            lib_path = self._find_libsurvive()
            logger.info(f"libsurvive 로드 경로: {lib_path}")
            lib = ctypes.CDLL(lib_path)
            lib.survive_simple_serial_number.argtypes = [ctypes.c_void_p]
            lib.survive_simple_serial_number.restype = ctypes.c_char_p
            return lib
        except Exception as e:
            logger.error(f"libsurvive 로드 실패: {e}")
            return None

    def _ptr_to_void_p(self, ptr_obj) -> ctypes.c_void_p:
        if isinstance(ptr_obj, int):
            return ctypes.c_void_p(ptr_obj)

        try:
            return ctypes.cast(ptr_obj, ctypes.c_void_p)
        except Exception:
            pass

        try:
            return ctypes.c_void_p(int(ptr_obj))
        except Exception as e:
            raise TypeError(
                f"dev.ptr를 c_void_p로 변환할 수 없습니다: {ptr_obj} ({type(ptr_obj)})"
            ) from e

    def _get_device_serial(self, dev) -> Optional[str]:
        if self.libsurvive is None:
            return None

        try:
            ptr = self._ptr_to_void_p(dev.ptr)
            raw = self.libsurvive.survive_simple_serial_number(ptr)
            if raw is None:
                return None
            return raw.decode("utf-8")
        except Exception:
            return None

    # -------------------------------------------------
    # survive context
    # -------------------------------------------------
    def connect(self) -> bool:
        if self.running:
            logger.warning("이미 연결되어 있습니다.")
            return True

        survive_args = sys.argv[:1]
        if self.config_path:
            survive_args.extend(["--config", self.config_path])
        if self.lh_config:
            survive_args.extend(["--lh", self.lh_config])
        survive_args.extend(self.args)

        try:
            self.context = pysurvive.SimpleContext(survive_args)
            if not self.context:
                logger.error("pysurvive context 초기화 실패")
                return False

            self.running = True

            self.collector_thread = threading.Thread(target=self._pose_collector, daemon=True)
            self.processor_thread = threading.Thread(target=self._pose_processor, daemon=True)
            self.monitor_thread = threading.Thread(target=self._device_monitor, daemon=True)

            self.collector_thread.start()
            self.processor_thread.start()
            self.monitor_thread.start()

            logger.info("ViveTrackerDriver 시작 완료")
            time.sleep(1.0)
            return True

        except Exception as e:
            logger.error(f"connect 실패: {e}")
            self.running = False
            return False

    def disconnect(self):
        if not self.running:
            return

        self.running = False

        for t in (self.collector_thread, self.processor_thread, self.monitor_thread):
            if t and t.is_alive():
                t.join(timeout=2.0)

        self.context = None
        logger.info("ViveTrackerDriver 종료 완료")

    def _device_monitor(self):
        while self.running and self.context and self.context.Running():
            self._update_device_list_from_objects()
            time.sleep(1.0)

    def _update_device_list_from_objects(self):
        try:
            devices = list(self.context.Objects())
            with self.data_lock:
                for dev in devices:
                    name = str(dev.Name(), "utf-8")
                    serial = self._get_device_serial(dev)

                    if name not in self.devices_info:
                        self.devices_info[name] = {
                            "updates": 0,
                            "last_update": 0.0,
                            "serial": serial,
                        }
                        logger.info(f"신규 장치 감지: name={name}, serial={serial}")
                    else:
                        self.devices_info[name]["serial"] = serial

                    if serial:
                        self.serial_to_name[serial] = name
                        self.name_to_serial[name] = serial

        except Exception as e:
            logger.error(f"_update_device_list_from_objects 오류: {e}")

    def _pose_collector(self):
        while self.running and self.context and self.context.Running():
            updated = self.context.NextUpdated()
            if not updated:
                continue

            try:
                device_name = str(updated.Name(), "utf-8")
                serial_number = self._get_device_serial(updated)

                pose_obj = updated.Pose()
                pose_data = pose_obj[0]
                timestamp = float(pose_obj[1])

                raw_x = pose_data.Pos[0]
                raw_y = pose_data.Pos[1]
                raw_z = pose_data.Pos[2]
                raw_qx = pose_data.Rot[1]
                raw_qy = pose_data.Rot[2]
                raw_qz = pose_data.Rot[3]
                raw_qw = pose_data.Rot[0]

                origin_mat = xyzQuaternion2matrix(
                    raw_x, raw_y, raw_z,
                    raw_qx, raw_qy, raw_qz, raw_qw
                )

                initial_rotation = xyzrpy2Mat(0, 0, 0, -(20.0 / 180.0 * math.pi), 0, 0)
                alignment_rotation = xyzrpy2Mat(0, 0, 0, -90 / 180 * math.pi, -90 / 180 * math.pi, 0)
                rotate_matrix = np.dot(initial_rotation, alignment_rotation)
                transform_matrix = xyzrpy2Mat(0.172, 0, -0.076, 0, 0, 0)

                result_mat = np.matmul(np.matmul(origin_mat, rotate_matrix), transform_matrix)
                x, y, z, qx, qy, qz, qw = matrixToXYZQuaternion(result_mat)

                pose = PoseData(
                    device_name=device_name,
                    serial_number=serial_number,
                    timestamp=timestamp,
                    position=[x, y, z],
                    rotation=[qx, qy, qz, qw],
                    raw_position=[raw_x, raw_y, raw_z],
                    raw_rotation=[raw_qx, raw_qy, raw_qz, raw_qw],
                )

                with self.data_lock:
                    if device_name not in self.devices_info:
                        self.devices_info[device_name] = {
                            "updates": 0,
                            "last_update": 0.0,
                            "serial": serial_number,
                        }

                    self.devices_info[device_name]["updates"] += 1
                    self.devices_info[device_name]["last_update"] = time.time()
                    self.devices_info[device_name]["serial"] = serial_number

                    if serial_number:
                        self.serial_to_name[serial_number] = device_name
                        self.name_to_serial[device_name] = serial_number

                try:
                    self.pose_queue.put_nowait(pose)
                except queue.Full:
                    _ = self.pose_queue.get_nowait()
                    self.pose_queue.put_nowait(pose)

            except Exception as e:
                logger.error(f"_pose_collector 오류: {e}")

    def _pose_processor(self):
        while self.running:
            try:
                pose = self.pose_queue.get(timeout=0.1)
                with self.data_lock:
                    self.latest_poses[pose.device_name] = pose
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"_pose_processor 오류: {e}")

    def get_devices(self) -> List[str]:
        self._update_device_list_from_objects()
        with self.data_lock:
            return list(self.devices_info.keys())

    def get_pose(self, device_name: Optional[str] = None):
        with self.data_lock:
            if device_name:
                return self.latest_poses.get(device_name)
            return dict(self.latest_poses)

    def get_pose_by_serial(self, serial_number: str) -> Optional[PoseData]:
        with self.data_lock:
            device_name = self.serial_to_name.get(serial_number)
            if not device_name:
                return None
            return self.latest_poses.get(device_name)

    def wait_for_serial(self, serial_number: str, timeout: float = 10.0) -> bool:
        start = time.time()
        while time.time() - start < timeout:
            with self.data_lock:
                if serial_number in self.serial_to_name:
                    return True
            self._update_device_list_from_objects()
            time.sleep(0.2)
        return False

    def __del__(self):
        self.disconnect()


class ViveTrackerNode(Node):
    """
    serial_numbers 파라미터로 1개 또는 2개 tracker를 받아 publish
    """

    def __init__(self):
        super().__init__("vive_tracker_node")

        # ------------------------------------------
        # Parameters
        # ------------------------------------------
        self.declare_parameter("serial_numbers", [""])
        self.declare_parameter("tracker_ids", [""])
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("config_path", "")
        self.declare_parameter("lh_config", "")

        self.serial_numbers = [s for s in list(self.get_parameter("serial_numbers").value) if s]
        self.tracker_ids = [s for s in list(self.get_parameter("tracker_ids").value) if s]
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        config_path = self.get_parameter("config_path").get_parameter_value().string_value or None
        lh_config = self.get_parameter("lh_config").get_parameter_value().string_value or None

        if len(self.serial_numbers) == 0:
            raise RuntimeError("serial_numbers 파라미터에 최소 1개의 tracker serial을 넣어야 합니다.")

        if len(self.serial_numbers) > 2:
            raise RuntimeError("현재 코드는 tracker 2개까지만 지원합니다.")

        if len(self.tracker_ids) == 0:
            self.tracker_ids = [f"tracker_{i}" for i in range(len(self.serial_numbers))]

        if len(self.tracker_ids) != len(self.serial_numbers):
            raise RuntimeError("tracker_ids 개수와 serial_numbers 개수는 같아야 합니다.")

        # serial -> tracker_id
        self.serial_to_tracker_id: Dict[str, str] = {
            serial: tracker_id for serial, tracker_id in zip(self.serial_numbers, self.tracker_ids)
        }

        # publishers
        self.pose_publishers: Dict[str, any] = {}
        self.raw_pose_publishers: Dict[str, any] = {}

        for serial, tracker_id in self.serial_to_tracker_id.items():
            topic = f"/vive_tracker/{tracker_id}/pose"
            topic_raw = f"/vive_tracker/{tracker_id}/pose_raw"

            self.pose_publishers[serial] = self.create_publisher(PoseStamped, topic, 10)
            self.raw_pose_publishers[serial] = self.create_publisher(PoseStamped, topic_raw, 10)

            self.get_logger().info(f"[{tracker_id}] serial={serial}")
            self.get_logger().info(f"[{tracker_id}] pose topic: {topic}")
            self.get_logger().info(f"[{tracker_id}] raw pose topic: {topic_raw}")

        # driver
        self.driver = ViveTrackerDriver(config_path=config_path, lh_config=lh_config)
        if not self.driver.connect():
            self.get_logger().error("ViveTrackerDriver 연결 실패")
            raise RuntimeError("ViveTrackerDriver 연결 실패")

        # wait for serials
        for serial in self.serial_numbers:
            ok = self.driver.wait_for_serial(serial, timeout=10.0)
            if ok:
                current_name = self.driver.serial_to_name.get(serial, "<unknown>")
                self.get_logger().info(f"serial {serial} 연결 확인됨 (current device name: {current_name})")
            else:
                self.get_logger().warn(f"serial {serial} 장치를 아직 찾지 못했습니다. 이후 업데이트를 기다립니다.")

        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self._publish_pose)
        self.get_logger().info(
            f"ViveTrackerNode 시작 — trackers={self.tracker_ids}, 주기={publish_rate:.1f} Hz"
        )

    def _publish_pose(self):
        now = self.get_clock().now().to_msg()

        for serial, tracker_id in self.serial_to_tracker_id.items():
            pose_data = self.driver.get_pose_by_serial(serial)
            if pose_data is None:
                continue

            # transformed pose
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = pose_data.position[0]
            msg.pose.position.y = pose_data.position[1]
            msg.pose.position.z = pose_data.position[2]
            msg.pose.orientation.x = pose_data.rotation[0]
            msg.pose.orientation.y = pose_data.rotation[1]
            msg.pose.orientation.z = pose_data.rotation[2]
            msg.pose.orientation.w = pose_data.rotation[3]
            self.pose_publishers[serial].publish(msg)

            # raw pose
            msg_raw = PoseStamped()
            msg_raw.header.stamp = now
            msg_raw.header.frame_id = self.frame_id
            msg_raw.pose.position.x = pose_data.raw_position[0]
            msg_raw.pose.position.y = pose_data.raw_position[1]
            msg_raw.pose.position.z = pose_data.raw_position[2]
            msg_raw.pose.orientation.x = pose_data.raw_rotation[0]
            msg_raw.pose.orientation.y = pose_data.raw_rotation[1]
            msg_raw.pose.orientation.z = pose_data.raw_rotation[2]
            msg_raw.pose.orientation.w = pose_data.raw_rotation[3]
            self.raw_pose_publishers[serial].publish(msg_raw)

    def destroy_node(self):
        self.driver.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ViveTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()