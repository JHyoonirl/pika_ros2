import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # 1. JSON 파일 경로 획득 및 로드
    pkg_share_dir = get_package_share_directory('pika_driver')
    config_path = os.path.join(pkg_share_dir, 'info', 'pika_info.json')

    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
    except Exception as e:
        print(f"[ERROR] JSON 로드 실패: {e}")
        return LaunchDescription([])

    nodes_to_launch = []
    tracker_serials = []
    tracker_ids = []
    tracker_namespaces = []

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

    # 2. 양쪽 사이드(left, right)에 대해 루프 실행
    sides = ['left', 'right']
    
    for side in sides:
        s_key = f"sense_{side}"
        g_key = f"gripper_{side}"
        
        # 각 섹션 데이터 가져오기
        s_cfg = config.get(s_key)
        g_cfg = config.get(g_key)

        if not s_cfg or not g_cfg:
            continue

        # --- Fisheye Node (Gripper side 장치 사용) ---
        nodes_to_launch.append(Node(
            package='pika_driver',
            executable='fisheye_driver',
            name=f'fisheye_{side}_driver',
            parameters=[{
                'camera_device': g_cfg['fisheye_device'],
                'topic_ns': f"/{g_cfg['namespace']}",
            }]
        ))

        # --- Motor Sense Node ---
        nodes_to_launch.append(Node(
            package='pika_driver',
            executable='motor_driver',
            name=f'motor_sense_{side}_driver',
            parameters=[{'serial_port': s_cfg['motor_port']}],
            remappings=[
                ('gripper/joint_state', f"{s_cfg['namespace']}/gripper/joint_state"),
            ]
        ))

        # --- Motor Gripper Node (Sense의 상태를 제어값으로 연결) ---
        nodes_to_launch.append(Node(
            package='pika_driver',
            executable='motor_driver',
            name=f'motor_gripper_{side}_driver',
            parameters=[{'serial_port': g_cfg['motor_port']}],
            remappings=[
                ('gripper/joint_state_ctrl', f"{s_cfg['namespace']}/gripper/joint_state"),
                ('gripper/joint_state', f"{g_cfg['namespace']}/gripper/joint_state"),
            ]
        ))

        # --- RealSense Nodes (차등 지연 적용) ---
        rs_delay = 8.0 if side == 'right' else 10.0
        rs_node = rs_camera(g_cfg['namespace'], f'realsense_{side}_gripper', g_cfg['realsense_serial'])
        nodes_to_launch.append(TimerAction(period=rs_delay, actions=[rs_node]))

        # --- 트래커 데이터 수집 ---
        tracker_serials.append(s_cfg['tracker_serial'])
        tracker_ids.append(side)
        tracker_namespaces.append(s_cfg['namespace'])

    # 3. 통합 Vive Tracker Node
    nodes_to_launch.append(Node(
        package='pika_driver',
        executable='vive_tracker_driver',
        name='vive_tracker_driver',
        parameters=[{
            'serial_numbers': tracker_serials,
            'tracker_ids': tracker_ids,
            'namespaces': tracker_namespaces,
            'publish_rate': 100.0,
            'frame_id': 'world',
        }]
    ))

    return LaunchDescription(nodes_to_launch)