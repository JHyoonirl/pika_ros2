import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # 1. JSON 파일 로드
    pkg_share_dir = get_package_share_directory('pika_driver')
    config_path = os.path.join(pkg_share_dir, 'info', 'pika_info.json')

    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
    except Exception as e:
        print(f"[ERROR] pika_info.json 로드 실패: {e}")
        return LaunchDescription([])

    nodes_to_launch = []

    # 2. 'left', 'right' 양쪽 그리퍼 설정 순회
    sides = ['left', 'right']
    
    for side in sides:
        key = f"gripper_{side}"
        if key not in config:
            print(f"[WARN] JSON에 {key} 설정이 없어 건너뜁니다.")
            continue
            
        cfg = config[key]
        ns = cfg['namespace']
        
        # Fisheye Node
        nodes_to_launch.append(Node(
            package='pika_driver',
            executable='fisheye_driver',
            name=f'fisheye_{side}_driver',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'camera_device': cfg['fisheye_device'],
                'camera_fps': 30,
                'camera_width': 640,
                'camera_height': 480,
                'camera_frame_id': 'fisheye',
                'topic_ns': f'/{ns}',
            }]
        ))

        # Motor Gripper Node
        nodes_to_launch.append(Node(
            package='pika_driver',
            executable='motor_driver',
            name=f'motor_{side}_gripper_driver',
            output='screen',
            parameters=[{'serial_port': cfg['motor_port']}],
            remappings=[
                ('gripper/joint_state', f'{ns}/gripper/joint_state'),
                ('gripper/joint_state_ctrl', f'{ns}/gripper/joint_state_ctrl'),
            ]
        ))

        # RealSense Node (안정성을 위해 left: 5초, right: 8초 지연)
        rs_delay = 5.0 if side == 'left' else 8.0
        rs_node = Node(
            package='pika_driver',
            executable='realsense_driver',
            name=f'realsense_{side}_gripper',
            namespace=ns,
            output='screen',
            emulate_tty=True,
            parameters=[{
                'serial_no': f"_{cfg['realsense_serial']}",
                'enable_color': True,
                'enable_depth': True,
                'width': 640,
                'height': 480,
                'fps': 30,
                'color_frame_id': 'color_optical_frame',
                'depth_frame_id': 'depth_optical_frame',
                'startup_retry_count': 20,
            }]
        )
        nodes_to_launch.append(TimerAction(period=rs_delay, actions=[rs_node]))

    return LaunchDescription(nodes_to_launch)