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
        print(f"[ERROR] Failed to load pika_info.json: {e}")
        return LaunchDescription([])

    # 런치 리스트 및 트래커 데이터 수집용 리스트
    nodes_to_launch = []
    tracker_serials = []
    tracker_ids = []
    tracker_namespaces = []

    # 2. 루프를 돌며 각 사이드(left, right) 노드 생성
    sides = ['left', 'right']
    
    for side in sides:
        key = f"sense_{side}"
        if key not in config:
            continue
            
        cfg = config[key]
        ns = cfg['namespace']
        
        # Fisheye Node
        nodes_to_launch.append(Node(
            package='pika_driver',
            executable='fisheye_driver',
            name=f'fisheye_{side}',
            output='screen',
            parameters=[{
                'camera_device': cfg['fisheye_device'],
                'camera_fps': 30,
                'namespace': f'/{ns}',
            }]
        ))

        # Motor Sense Node
        nodes_to_launch.append(Node(
            package='pika_driver',
            executable='motor_driver',
            name=f'motor_sense_{side}_driver',
            output='screen',
            parameters=[{'serial_port': cfg['motor_port']}],
            remappings=[('gripper/joint_state', f'{ns}/gripper/joint_state')]
        ))

        # RealSense Node (사이드별로 지연 시간 차등 부여)
        rs_delay = 5.0 if side == 'left' else 8.0
        rs_node = Node(
            package='pika_driver',
            executable='realsense_driver',
            name=f'realsense_{side}',
            namespace=ns,
            parameters=[{
                'serial_no': f"_{cfg['realsense_serial']}",
                'enable_color': True,
                'enable_depth': True,
                'width': 640, 'height': 480, 'fps': 30
            }]
        )
        nodes_to_launch.append(TimerAction(period=rs_delay, actions=[rs_node]))

        # 트래커 정보 수집
        tracker_serials.append(cfg['tracker_serial'])
        tracker_ids.append(side)
        tracker_namespaces.append(ns)

    # 3. 통합 Vive Tracker Node (한 번에 양쪽 처리)
    nodes_to_launch.append(Node(
        package='pika_driver',
        executable='vive_tracker_driver',
        name='vive_tracker_driver',
        output='screen',
        parameters=[{
            'serial_numbers': tracker_serials,
            'tracker_ids': tracker_ids,
            'namespaces': tracker_namespaces,
            'publish_rate': 100.0,
            'frame_id': 'world',
        }]
    ))

    return LaunchDescription(nodes_to_launch)