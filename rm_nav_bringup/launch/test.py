import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    # 1. 获取 astra_camera 功能包路径
    # 如果你的功能包名不是 astra_camera，请在此修改
    camera_pkg_dir = get_package_share_directory('astra_camera')
    
    # 2. 定义 XML 启动文件的路径 (假设文件名是 astra.launch)
    camera_launch_path = os.path.join(camera_pkg_dir, 'launch', 'gemini.launch.xml')

    # 3. 配置前置摄像头 (使用你查到的 SN 码: AY2755200S5)
    front_camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(camera_launch_path),
        launch_arguments={
            'camera_name': 'camera_front',
            'serial_number': 'AY27552006X',
            'enable_point_cloud': 'true'
        }.items()
    )

    # 4. 配置后置摄像头 (请将 'YOUR_REAR_SN' 替换为第二个相机的真实 SN 码)
    rear_camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(camera_launch_path),
        launch_arguments={
            'camera_name': 'camera_rear',
            'serial_number': 'AY2755200S5',
            'enable_point_cloud': 'true'
        }.items()
    )

    # 5. 返回启动描述，仅包含这两个摄像头节点
    return LaunchDescription([
        front_camera,
        rear_camera
    ])

