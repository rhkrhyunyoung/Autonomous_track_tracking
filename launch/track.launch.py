import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 리얼센스 패키지의 공유 디렉토리 경로 가져오기
    realsense_share_dir = get_package_share_directory('realsense2_camera')
    
    # 2. 리얼센스 런치 파일 포함 (사용자님이 사용하던 옵션 그대로 적용)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'rgb_camera.profile': '1280x720x15',
            'depth_module.profile': '1280x720x15',
            'align_depth.enable': 'true'
        }.items()
    )

    # 3. 사용자님의 라인 트래킹 노드 설정
    line_tracking_node = Node(
        package='linetracing_cpp',
        executable='main_node', # 혹은 main1_node (CMakeLists.txt에 정의된 이름)
        name='lane_follower_node',
        output='screen'
    )

    # 4. 실행 리스트 반환
    return LaunchDescription([
        realsense_launch,
        line_tracking_node
    ])
