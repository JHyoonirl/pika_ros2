#!/usr/bin/env python3
import time
from typing import Optional

import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class RealSenseRobustPublisher(Node):
    def __init__(self) -> None:
        super().__init__("realsense_publisher")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("serial_no", "")
        # self.declare_parameter("topic_ns", "/camera")
        self.declare_parameter("enable_color", True)
        self.declare_parameter("enable_depth", True)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)

        self.declare_parameter("color_frame_id", "camera_color_optical_frame")
        self.declare_parameter("depth_frame_id", "camera_depth_optical_frame")

        self.declare_parameter("startup_retry_count", 20)
        self.declare_parameter("startup_retry_delay_sec", 1.0)
        self.declare_parameter("frame_timeout_ms", 3000)
        self.declare_parameter("warmup_frames", 15)
        self.declare_parameter("max_consecutive_failures", 10)

        self.serial_no: str = str(self.get_parameter("serial_no").value).lstrip("_")
        # self.topic_ns: str = str(self.get_parameter("topic_ns").value).rstrip("/")
        self.enable_color: bool = bool(self.get_parameter("enable_color").value)
        self.enable_depth: bool = bool(self.get_parameter("enable_depth").value)
        self.width: int = int(self.get_parameter("width").value)
        self.height: int = int(self.get_parameter("height").value)
        self.fps: int = int(self.get_parameter("fps").value)
        self.color_frame_id: str = str(self.get_parameter("color_frame_id").value)
        self.depth_frame_id: str = str(self.get_parameter("depth_frame_id").value)
        self.startup_retry_count: int = int(self.get_parameter("startup_retry_count").value)
        self.startup_retry_delay_sec: float = float(self.get_parameter("startup_retry_delay_sec").value)
        self.frame_timeout_ms: int = int(self.get_parameter("frame_timeout_ms").value)
        self.warmup_frames: int = int(self.get_parameter("warmup_frames").value)
        self.max_consecutive_failures: int = int(self.get_parameter("max_consecutive_failures").value)

        if not self.enable_color and not self.enable_depth:
            raise ValueError("enable_color 또는 enable_depth 중 하나는 True여야 합니다.")

        # if not self.topic_ns.startswith("/"):
        #     self.topic_ns = "/" + self.topic_ns

        self.color_topic = f"color/image_raw"
        self.depth_topic = f"depth/image_raw"
        self.color_info_topic = f"color/camera_info"
        self.depth_info_topic = f"depth/camera_info"

        self.bridge = CvBridge()

        self.color_pub = self.create_publisher(Image, self.color_topic, 10) if self.enable_color else None
        self.depth_pub = self.create_publisher(Image, self.depth_topic, 10) if self.enable_depth else None
        self.color_info_pub = self.create_publisher(CameraInfo, self.color_info_topic, 10) if self.enable_color else None
        self.depth_info_pub = self.create_publisher(CameraInfo, self.depth_info_topic, 10) if self.enable_depth else None

        self.pipeline: Optional[rs.pipeline] = None
        self.config: Optional[rs.config] = None
        self.profile: Optional[rs.pipeline_profile] = None

        self.color_camera_info: Optional[CameraInfo] = None
        self.depth_camera_info: Optional[CameraInfo] = None

        self.consecutive_failures = 0

        self._start_pipeline_with_retry()

        timer_period = 1.0 / max(self.fps, 1)
        self.timer = self.create_timer(timer_period, self._timer_callback)

        # self.get_logger().info(
        #     # f"RealSense publisher started. ns={self.topic_ns}, color={self.enable_color}, depth={self.enable_depth}"
        # )

    # -------------------------------------------------
    # Pipeline lifecycle
    # -------------------------------------------------
    def _stop_pipeline(self) -> None:
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception as e:
                self.get_logger().warn(f"pipeline.stop() failed: {e}")

        self.pipeline = None
        self.config = None
        self.profile = None

    def _build_config(self) -> rs.config:
        config = rs.config()

        if self.serial_no:
            config.enable_device(self.serial_no)
            self.get_logger().info(f"Using serial number: {self.serial_no}")

        if self.enable_color:
            config.enable_stream(
                rs.stream.color,
                self.width,
                self.height,
                rs.format.bgr8,
                self.fps,
            )

        if self.enable_depth:
            config.enable_stream(
                rs.stream.depth,
                self.width,
                self.height,
                rs.format.z16,
                self.fps,
            )

        return config

    def _start_pipeline_once(self) -> None:
        self._stop_pipeline()

        self.pipeline = rs.pipeline()
        self.config = self._build_config()
        self.profile = self.pipeline.start(self.config)

        self._cache_camera_info()

        # Warm-up: 첫 frameset이 안정적으로 들어올 때까지 기다림
        good_frames = 0
        deadline = time.time() + 12.0

        while time.time() < deadline:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=self.frame_timeout_ms)
            except RuntimeError:
                continue

            if not frames:
                continue

            ok = True
            if self.enable_color and not frames.get_color_frame():
                ok = False
            if self.enable_depth and not frames.get_depth_frame():
                ok = False

            if ok:
                good_frames += 1
            else:
                good_frames = 0

            if good_frames >= self.warmup_frames:
                self.get_logger().info("RealSense pipeline started successfully.")
                self.consecutive_failures = 0
                return

        raise RuntimeError("Warm-up 중 안정적인 첫 프레임 세트를 확보하지 못했습니다.")

    def _start_pipeline_with_retry(self) -> None:
        last_error = None

        for attempt in range(1, self.startup_retry_count + 1):
            try:
                self.get_logger().info(f"Starting pipeline... attempt {attempt}/{self.startup_retry_count}")
                self._start_pipeline_once()
                return
            except Exception as e:
                last_error = e
                self.get_logger().warn(f"Pipeline start failed: {e}")
                self._stop_pipeline()
                time.sleep(self.startup_retry_delay_sec)

        raise RuntimeError(f"Pipeline start failed after retries. Last error: {last_error}")

    def _restart_pipeline(self) -> None:
        self.get_logger().warn("Restarting RealSense pipeline...")
        self._start_pipeline_with_retry()

    # -------------------------------------------------
    # Camera info
    # -------------------------------------------------
    def _get_camera_info_msg(self, profile, frame_id: str) -> CameraInfo:
        intr = profile.as_video_stream_profile().get_intrinsics()

        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width = intr.width
        msg.height = intr.height
        msg.distortion_model = "plumb_bob"
        msg.d = list(intr.coeffs)

        msg.k = [
            intr.fx, 0.0, intr.ppx,
            0.0, intr.fy, intr.ppy,
            0.0, 0.0, 1.0,
        ]

        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        msg.p = [
            intr.fx, 0.0, intr.ppx, 0.0,
            0.0, intr.fy, intr.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        return msg

    def _cache_camera_info(self) -> None:
        if self.profile is None:
            return

        if self.enable_color:
            color_stream = self.profile.get_stream(rs.stream.color)
            self.color_camera_info = self._get_camera_info_msg(color_stream, self.color_frame_id)

        if self.enable_depth:
            depth_stream = self.profile.get_stream(rs.stream.depth)
            self.depth_camera_info = self._get_camera_info_msg(depth_stream, self.depth_frame_id)

    # -------------------------------------------------
    # Timer
    # -------------------------------------------------
    def _timer_callback(self) -> None:
        if self.pipeline is None:
            return

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=self.frame_timeout_ms)
        except RuntimeError as e:
            self.consecutive_failures += 1
            self.get_logger().warn(
                f"wait_for_frames failed ({self.consecutive_failures}/{self.max_consecutive_failures}): {e}"
            )

            if self.consecutive_failures >= self.max_consecutive_failures:
                self._restart_pipeline()

            return
        except Exception as e:
            self.consecutive_failures += 1
            self.get_logger().warn(
                f"Unexpected frame acquisition error ({self.consecutive_failures}/{self.max_consecutive_failures}): {e}"
            )
            if self.consecutive_failures >= self.max_consecutive_failures:
                self._restart_pipeline()
            return

        if not frames:
            self.consecutive_failures += 1
            if self.consecutive_failures >= self.max_consecutive_failures:
                self._restart_pipeline()
            return

        self.consecutive_failures = 0
        stamp = self.get_clock().now().to_msg()

        if self.enable_color and self.color_pub is not None:
            color_frame = frames.get_color_frame()
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                color_msg.header.stamp = stamp
                color_msg.header.frame_id = self.color_frame_id
                self.color_pub.publish(color_msg)

                if self.color_camera_info is not None and self.color_info_pub is not None:
                    color_info = CameraInfo()
                    color_info.header.stamp = stamp
                    color_info.header.frame_id = self.color_camera_info.header.frame_id
                    color_info.width = self.color_camera_info.width
                    color_info.height = self.color_camera_info.height
                    color_info.distortion_model = self.color_camera_info.distortion_model
                    color_info.d = list(self.color_camera_info.d)
                    color_info.k = list(self.color_camera_info.k)
                    color_info.r = list(self.color_camera_info.r)
                    color_info.p = list(self.color_camera_info.p)
                    self.color_info_pub.publish(color_info)

        if self.enable_depth and self.depth_pub is not None:
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = self.depth_frame_id
                self.depth_pub.publish(depth_msg)

                if self.depth_camera_info is not None and self.depth_info_pub is not None:
                    depth_info = CameraInfo()
                    depth_info.header.stamp = stamp
                    depth_info.header.frame_id = self.depth_camera_info.header.frame_id
                    depth_info.width = self.depth_camera_info.width
                    depth_info.height = self.depth_camera_info.height
                    depth_info.distortion_model = self.depth_camera_info.distortion_model
                    depth_info.d = list(self.depth_camera_info.d)
                    depth_info.k = list(self.depth_camera_info.k)
                    depth_info.r = list(self.depth_camera_info.r)
                    depth_info.p = list(self.depth_camera_info.p)
                    self.depth_info_pub.publish(depth_info)

    def destroy_node(self) -> bool:
        self.get_logger().info("Stopping RealSense pipeline...")
        self._stop_pipeline()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = RealSenseRobustPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unhandled exception: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()