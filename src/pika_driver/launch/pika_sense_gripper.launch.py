from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    def rs_camera(namespace, node_name, serial_no):
        return Node(
            package='pika_driver',
            executable='realsense_driver',
            name=node_name,
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'serial_no': f'_{serial_no}',
                # 'topic_ns': f'/{namespace}',

                'enable_color': True,
                'enable_depth': True,

                'width': 640,
                'height': 480,
                'fps': 30,

                'color_frame_id': f'color_optical_frame',
                'depth_frame_id': f'depth_optical_frame',

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
    realsense_sense = rs_camera('pika_sense', 'realsense_sense', '315122271136')
    realsense_gripper = rs_camera('pika_gripper', 'realsense_gripper', '315122270809')

    fisheye_node = Node(
        package='pika_driver',
        executable='fisheye_driver',
        name='fisheye_driver',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'camera_device': '/dev/pika_gripper_left_video',
            'camera_fps': 30,
            'camera_width': 640,
            'camera_height': 480,
            'camera_frame_id': 'fisheye',
            'topic_ns': '/fisheye',
        }]
    )
    sense_namespace = 'pika_sense'
    motor_sense_node = Node(
        package='pika_driver',
        executable='motor_driver',
        name='motor_sense_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/pika_sense_left_serial',
        }],
        remappings=[
            ('gripper/joint_state', f'{sense_namespace}/gripper/joint_state'),
        ]
    )
    gripper_namespace = 'pika_gripper'
    motor_gripper_node = Node(
        package='pika_driver',
        executable='motor_driver',
        name='motor_gripper_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/pika_gripper_left_serial',
        }],
        remappings=[
            ('gripper/joint_state_ctrl', f'{sense_namespace}/gripper/joint_state'),
            ('gripper/joint_state', f'{gripper_namespace}/gripper/joint_state'),
        ]

    )

    right_tracker= 'LHR-FBF3A347'
    left_tracker = 'LHR-63AAAF5B'
    vive_tracker_node = Node(
        package='pika_driver',
        executable='vive_tracker_driver',
        name='vive_tracker_driver',
        output='screen',
        parameters=[{
        'serial_numbers': [left_tracker],
        'tracker_ids': ['left'],
        'publish_rate': 100.0,
        'frame_id': 'world',
    }]
    )

    return LaunchDescription([
        fisheye_node,
        motor_sense_node,
        motor_gripper_node,
        # vive_tracker_node,
        # RealSense는 순차적으로 띄우는 게 안정적
        # TimerAction(period=5.0, actions=[realsense_sense]),
        TimerAction(period=8.0, actions=[realsense_gripper]),
    ])