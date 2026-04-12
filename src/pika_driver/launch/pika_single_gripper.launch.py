import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # 1. 실행 인자 평가 (left / right)
    side = LaunchConfiguration('side').perform(context)
    
    # 2. JSON 설정 로드
    pkg_share_dir = get_package_share_directory('pika_driver')
    config_path = os.path.join(pkg_share_dir, 'info', 'pika_info.json')

    with open(config_path, 'r') as f:
        config = json.load(f)

    # 3. 인자에 따른 키 매핑 (gripper_left 또는 gripper_right)
    key = f"gripper_{side}"
    cfg = config[key]
    ns = cfg['namespace']

    # RealSense 생성 헬퍼 함수
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
                'enable_color': True,
                'enable_depth': True,
                'width': 640, 'height': 480, 'fps': 30,
                'color_frame_id': 'color_optical_frame',
                'depth_frame_id': 'depth_optical_frame',
                'startup_retry_count': 20,
            }]
        )

    # 노드 정의
    realsense_gripper = rs_camera(ns, 'realsense_gripper', cfg['realsense_serial'])

    fisheye_node = Node(
        package='pika_driver',
        executable='fisheye_driver',
        name='fisheye_driver',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'camera_device': cfg['fisheye_device'],
            'camera_fps': 30,
            'camera_width': 640,
            'camera_height': 480,
            'camera_frame_id': 'fisheye',
            'namespace': ns,
        }]
    )

    motor_gripper_node = Node(
        package='pika_driver',
        executable='motor_driver',
        name='motor_gripper_driver',
        output='screen',
        parameters=[{'serial_port': cfg['motor_port']}],
        remappings=[
            ('gripper/joint_state', f'{ns}/gripper/joint_state'),
            ('gripper/joint_state_ctrl', f'{ns}/gripper/joint_state_ctrl'),
        ]
    )

    return [
        fisheye_node,
        motor_gripper_node,
        TimerAction(period=5.0, actions=[realsense_gripper]),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'side',
            default_value='left',
            description='Select gripper side: left or right'
        ),
        OpaqueFunction(function=launch_setup)
    ])