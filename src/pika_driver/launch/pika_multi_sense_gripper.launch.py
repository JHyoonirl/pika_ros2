from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

# TODO: namespace, remapping 등으로 좀 더 깔끔하게 정리하기
# sense_gripper.launch.py 참고하여 수정하기

def generate_launch_description():

    def rs_camera(namespace, name, serial_no):
        return Node(
            package='pika_driver',
            executable='realsense_driver',
            name=name,
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'serial_no': f'_{serial_no}',
                'topic_ns': f'/{namespace}/{name}',

                'enable_color': True,
                'enable_depth': True,

                'width': 640,
                'height': 480,
                'fps': 30,

                'color_frame_id': f'{name}_color_optical_frame',
                'depth_frame_id': f'{name}_depth_optical_frame',

                'startup_retry_count': 20,
                'startup_retry_delay_sec': 1.0,
                'frame_timeout_ms': 3000,
                'warmup_frames': 15,
                'max_consecutive_failures': 10,
            }]
        )

    # ---------------------------
    # RealSense 설정
    # ---------------------------
    realsense_1 = rs_camera('realsense_1', 'camera_1', '315122271136')
    realsense_2 = rs_camera('realsense_2', 'camera_2', '315122270900')

    fisheye_node = Node(
        package='pika_driver',
        executable='fisheye_driver',
        name='fisheye_driver',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'camera_device': '/dev/pika_sensor_left_video',
            'camera_fps': 30,
            'camera_width': 640,
            'camera_height': 480,
            'camera_frame_id': 'fisheye_left',
        }]
    )

    motor_node = Node(
        package='pika_driver',
        executable='motor_driver',
        name='motor_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/pika_sensor_left_serial',
        }]
    )

    vive_tracker_node = Node(
        package='pika_driver',
        executable='vive_tracker_driver',
        name='vive_tracker_driver',
        output='screen'
    )

    return LaunchDescription([
        fisheye_node,
        motor_node,
        vive_tracker_node,

        # RealSense는 순차적으로 띄우는 게 안정적
        TimerAction(period=5.0, actions=[realsense_1]),
        TimerAction(period=8.0, actions=[realsense_2]),
    ])