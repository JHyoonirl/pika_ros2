#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import threading 
import signal
import sys
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import subprocess


class RosOperator(Node):
    def __init__(self): 
        super().__init__('camera_fisheye')
        self.cap = None
        # self.camera_port = None
        self.camera_device = None
        self.camera_hz = None
        self.camera_height = 480
        self.camera_width = 640
        self.bridge = None
        self.camera_color_publisher = None
        self.camera_config_publisher = None
        self.camera_frame_id = None
        self.topic_ns = '/fisheye'
        self.tf_broadcaster = None
        self.running = False
        self.camera_thread = None
        self.init_ros()

    def init_ros(self):
        # self.declare_parameter('camera_port', 22)
        self.declare_parameter('camera_device', '/dev/pika_gripper_left_video')
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_frame_id', "camera_rgb")
        self.declare_parameter('topic_ns', '/fisheye')
        self.camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        self.camera_hz = int(self.get_parameter('camera_fps').get_parameter_value().integer_value)
        self.camera_height = int(self.get_parameter('camera_height').get_parameter_value().integer_value)
        self.camera_width = int(self.get_parameter('camera_width').get_parameter_value().integer_value)
        self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        self.topic_ns = self.get_parameter('topic_ns').get_parameter_value().string_value

        self.topic_ns = self.topic_ns.rstrip('/')
        if not self.topic_ns:
            self.topic_ns = '/fisheye'
        if not self.topic_ns.startswith('/'):
            self.topic_ns = '/' + self.topic_ns

        if self.camera_frame_id.startswith('/'):
            self.camera_frame_id = self.camera_frame_id[1:]

        color_topic = f'{self.topic_ns}/color/image_raw'
        camera_info_topic = f'{self.topic_ns}/color/camera_info'

        self.bridge = CvBridge()
        self.camera_color_publisher = self.create_publisher(Image, color_topic, 10)
        self.camera_config_publisher = self.create_publisher(CameraInfo, camera_info_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(f'camera_device: {self.camera_device}')
        self.get_logger().info(f'camera_fps: {self.camera_hz}')
        self.get_logger().info(f'camera_size: {self.camera_width}x{self.camera_height}')
        self.get_logger().info(f'topic_ns: {self.topic_ns}')


    def init_camera(self):
        device_path = self.camera_device

        if not os.path.exists(device_path):
            self.get_logger().error(f'Camera device not found: {device_path}')
            return False

        real_device_path = os.path.realpath(device_path)
        self.get_logger().info(f'Using camera device: {device_path} -> {real_device_path}')

        try:
            subprocess.run(
                [
                    'v4l2-ctl',
                    '-d',
                    real_device_path,
                    f'--set-fmt-video=width={self.camera_width},height={self.camera_height},pixelformat=MJPG'
                ],
                check=False
            )
            subprocess.run(
                [
                    'v4l2-ctl',
                    '-d',
                    real_device_path,
                    f'--set-parm={self.camera_hz}'
                ],
                check=False
            )
            self.get_logger().info(f'[v4l2-ctl] {real_device_path} 하드웨어 세팅 시도 완료')
        except Exception as e:
            self.get_logger().warning(f'v4l2-ctl 실행 실패: {e}')

        # 2. V4L2 드라이버를 명시하여 OpenCV 실행
        self.cap = cv2.VideoCapture(real_device_path, cv2.CAP_V4L2)
        self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.cap.set(cv2.CAP_PROP_FOURCC, self.fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_hz)
        
        # 3. 실제 적용된 해상도 터미널에 출력하여 확인
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"✅ 최종 적용된 카메라 해상도: {actual_w} x {actual_h}")

        if self.cap.isOpened():
            return True
        else:
            if self.cap:
                self.cap.release()
            return False
    
    def run(self):
        rate = self.create_rate(self.camera_hz)
        self.running = True
        try:
            # rclpy.ok()를 체크하여 셧다운 시 루프 즉시 탈출
            while rclpy.ok() and self.running and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    # 컨텍스트 유효성 체크 후 퍼블리시
                    if rclpy.ok():
                        self.publish_camera_color(frame)
                rate.sleep()
        except Exception as e:
            if rclpy.ok(): # 셧다운 중 발생하는 에러는 무시
                self.get_logger().error(f"Camera error: {e}")
        finally:
            self.cleanup_camera()

    def cleanup_camera(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
            # 노드가 살아있을 때만 로깅
            if rclpy.ok():
                self.get_logger().info("Camera released")
    
    def stop(self):
        """停止摄像头操作"""
        self.running = False
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=2.0)

    def publish_camera_color(self, color):
        img = self.bridge.cv2_to_imgmsg(color, "bgr8")
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = self.camera_frame_id + "_color"
        self.camera_color_publisher.publish(img)
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.camera_frame_id + "_color"
        camera_info.header.stamp = self.get_clock().now().to_msg()
        self.camera_config_publisher.publish(camera_info)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame_id
        t.child_frame_id = self.camera_frame_id + "_color"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)


ros_operator_instance = None

def signal_handler(signum, frame):
    """신호가 오면 플래그만 변경하여 안전하게 종료 유도"""
    global ros_operator_instance
    if ros_operator_instance:
        ros_operator_instance.running = False


def main():
    global ros_operator_instance
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    rclpy.init()
    ros_operator_instance = RosOperator()
    
    try:
        if ros_operator_instance.init_camera():
            print("camera opened")
            ros_operator_instance.camera_thread = threading.Thread(target=ros_operator_instance.run)
            # daemon=True는 유지하되, stop()으로 명시적 join 권장
            ros_operator_instance.camera_thread.daemon = True 
            ros_operator_instance.camera_thread.start()
            
            rclpy.spin(ros_operator_instance)
        else:
            print("camera error")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        print(f"Main Error: {e}")
    finally:
        # 종료 처리는 오직 여기서 딱 한 번만!
        if ros_operator_instance:
            ros_operator_instance.stop()
        
        if rclpy.ok():
            rclpy.shutdown()
        print("Program terminated safely")


if __name__ == '__main__':
    main()
