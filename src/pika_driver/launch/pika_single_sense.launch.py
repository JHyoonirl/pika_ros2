import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # 1. 런타임에 Argument 값 평가 (left 또는 right)
    target_side = LaunchConfiguration('side').perform(context)
    
    # 2. JSON 파일 로드
    pkg_share_dir = get_package_share_directory('pika_driver')
    config_path = os.path.join(pkg_share_dir, 'info', 'pika_info.json')

    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
    except Exception as e:
        print(f"[ERROR] JSON 로드 실패: {e}")
        return []

    # 3. 동적 키 매핑 (sense_left 또는 sense_right)
    side_key = f"sense_{target_side}"
    if side_key not in config:
        print(f"[ERROR] JSON에 {side_key} 설정이 없습니다.")
        return []
        
    cfg = config[side_key]
    ns = cfg['namespace']

    # 4. 노드 정의
    fisheye_node = Node(
        package='pika_driver',
        executable='fisheye_driver',
        name='fisheye_driver',
        parameters=[{
            'camera_device': cfg['fisheye_device'],
            'namespace': f'/{ns}',
        }]
    )

    motor_sense_node = Node(
        package='pika_driver',
        executable='motor_driver',
        name='motor_sense_driver',
        parameters=[{'serial_port': cfg['motor_port']}],
        remappings=[('gripper/joint_state', f'{ns}/gripper/joint_state')]
    )

    vive_tracker_node = Node(
        package='pika_driver',
        executable='vive_tracker_driver',
        name='vive_tracker_driver',
        parameters=[{
            'serial_numbers': [cfg['tracker_serial']],
            'tracker_ids': [target_side],
            'namespaces': [ns],
            'publish_rate': 100.0,
            'frame_id': 'world',
        }]
    )

    realsense_node = Node(
        package='pika_driver',
        executable='realsense_driver',
        name='realsense',
        namespace=ns,
        parameters=[{
            'serial_no': f"_{cfg['realsense_serial']}",
            'enable_color': True,
            'enable_depth': True,
        }]
    )

    return [
        fisheye_node,
        motor_sense_node,
        vive_tracker_node,
        TimerAction(period=5.0, actions=[realsense_node]),
    ]

def generate_launch_description():
    return LaunchDescription([
        # 외부에서 입력받을 인자 선언 (기본값: left)
        DeclareLaunchArgument(
            'side',
            default_value='left',
            description='Side of the sensor group to launch (left/right)'
        ),
        # 컨텍스트 기반 실행을 위한 OpaqueFunction 등록
        OpaqueFunction(function=launch_setup)
    ])