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

    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
    except Exception as e:
        print(f"[ERROR] Failed to load pika_info.json: {e}")
        return []

    # 3. JSON 구조에 따른 키 조합 (sense_side, gripper_side)
    s_key = f"sense_{side}"
    g_key = f"gripper_{side}"
    
    if s_key not in config or g_key not in config:
        print(f"[ERROR] Config keys {s_key} or {g_key} not found in JSON.")
        return []

    s_cfg = config[s_key]
    g_cfg = config[g_key]

    # RealSense 생성용 내부 헬퍼 함수
    def create_rs_node(cfg, node_name):
        return Node(
            package='pika_driver',
            executable='realsense_driver',
            name=node_name,
            namespace=cfg['namespace'],
            output='screen',
            emulate_tty=True,
            parameters=[{
                'serial_no': f"_{cfg['realsense_serial']}",
                'enable_color': True,
                'enable_depth': True,
                'width': 640, 'height': 480, 'fps': 30,
                'color_frame_id': 'color_optical_frame',
                'depth_frame_id': 'depth_optical_frame',
                'startup_retry_count': 20,
            }]
        )

    # --- 노드 리스트 구성 ---
    nodes = []

    # 1. Fisheye Node (Gripper side 장치 사용)
    nodes.append(Node(
        package='pika_driver',
        executable='fisheye_driver',
        name='fisheye_driver',
        output='screen',
        parameters=[{
            'camera_device': g_cfg['fisheye_device'],
            'camera_fps': 30,
            'namespace': f"/{g_cfg['namespace']}",
        }]
    ))

    # 2. Motor Sense Node
    nodes.append(Node(
        package='pika_driver',
        executable='motor_driver',
        name='motor_sense_driver',
        output='screen',
        parameters=[{'serial_port': s_cfg['motor_port']}],
        remappings=[
            ('gripper/joint_state', f"{s_cfg['namespace']}/gripper/joint_state"),
        ]
    ))

    # 3. Motor Gripper Node
    nodes.append(Node(
        package='pika_driver',
        executable='motor_driver',
        name='motor_gripper_driver',
        output='screen',
        parameters=[{'serial_port': g_cfg['motor_port']}],
        remappings=[
            # Sense의 상태를 제어 입력으로 사용하는 리매핑
            ('gripper/joint_state_ctrl', f"{s_cfg['namespace']}/gripper/joint_state"),
            ('gripper/joint_state', f"{g_cfg['namespace']}/gripper/joint_state"),
        ]
    ))

    # 4. Vive Tracker Node (Sense 쪽 serial 사용)
    nodes.append(Node(
        package='pika_driver',
        executable='vive_tracker_driver',
        name='vive_tracker_driver',
        output='screen',
        parameters=[{
            'serial_numbers': [s_cfg['tracker_serial']],
            'tracker_ids': [side],
            'namespaces': [s_cfg['namespace']],
            'publish_rate': 100.0,
            'frame_id': 'world',
        }]
    ))

    # 5. RealSense Nodes (안정성을 위해 시차를 두고 실행)
    rs_gripper = create_rs_node(g_cfg, 'realsense_gripper')
    
    nodes.append(TimerAction(period=8.0, actions=[rs_gripper]))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'side',
            default_value='left',
            description='Target side for sense and gripper (left/right)'
        ),
        OpaqueFunction(function=launch_setup)
    ])