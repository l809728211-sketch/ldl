import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('rm_nav_bringup')
    camera_pkg_dir = get_package_share_directory('astra_camera')
    
    # 确认使用的是 gemini.launch.xml
    camera_launch_file = os.path.join(camera_pkg_dir, 'launch', 'gemini.launch.xml')

    # ================= 1. 机器人状态发布者 (TF) =================
    xacro_path = os.path.join(bringup_dir, 'urdf', 'sentry_robot_real.xacro')
    robot_description = Command(['xacro ', xacro_path])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # ================= 2. 建立水平投影坐标系 (解决地面误报的核心) =================
    # 逻辑：创建一个位置与相机一致，但没有俯仰角的坐标系。
    # 高度计算：base_link离地0.19m + 相机安装位0.07m = 0.26m
    
    # 前置水平坐标系
    tf_front_level = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_front_level_publisher',
        arguments=['--x', '0.44', '--y', '0.0', '--z', '0.07', 
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_front_level']
    )

    # 后置水平坐标系 (Yaw设为 3.1415926 保证其朝向后方)
    tf_rear_level = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_rear_level_publisher',
        arguments=['--x', '-0.44', '--y', '0.0', '--z', '0.07', 
                   '--yaw', '3.1415926', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_rear_level']
    )

    # ================= 3. Livox Mid-360 雷达 =================
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        parameters=[{
            'user_config_path': os.path.join(bringup_dir, 'config', 'reality', 'MID360_config.json'),
            'frame_id': 'livox_frame'
        }]
    )

    # ================= 4. 前置深度相机及转换节点 =================
    front_camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(camera_launch_file),
        launch_arguments={
            'camera_name': 'camera_front',
            'serial_number': 'AY27552006X',
            'device_num': '2',
            'publish_tf': 'false',
            'enable_color': 'false',
            'use_uvc_camera': 'false',
        }.items()
    )

    pct_to_scan_front_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pct_to_scan_front',
        remappings=[
            ('cloud_in', '/camera_front/depth/points'),
            ('scan', '/scan_front')
        ],
        parameters=[{
            'target_frame': 'camera_front_level', # 🌟 使用水平坐标系
            'transform_tolerance': 0.05,
            'min_height': -0.15,            # 🌟 离地6cm (0.06 - 0.26)
            'max_height': 0.10,             # 🌟 离地50cm (0.50 - 0.26)
            'angle_min': -0.8,
            'angle_max': 0.8, 
            'angle_increment': 0.0087,
            'scan_time': 0.033,
            'range_min': 0.20,
            'range_max': 1.0,
            'use_inf': True, 
            'inf_epsilon': 1.0,
            'use_sim_time': False
        }],
        output='screen'
    )

    # ================= 5. 后置深度相机及转换节点 (同步延迟) =================
    # 将相机和转换节点一起延迟，确保 TF 树建立后再开始处理数据
    rear_camera_group_delayed = TimerAction(
        period=2.0, 
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(camera_launch_file),
                launch_arguments={
                    'camera_name': 'camera_rear',
                    'serial_number': 'AY2755200S5',
                    'device_num': '2',
                    'publish_tf': 'false',
                    'enable_color': 'false',
                    'use_uvc_camera': 'false',
                }.items()
            ),
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pct_to_scan_rear',
                remappings=[
                    ('cloud_in', '/camera_rear/depth/points'),
                    ('scan', '/scan_rear')
                ],
                parameters=[{
                    'target_frame': 'camera_rear_level', # 🌟 使用水平坐标系
                    'transform_tolerance': 0.05,
                    'min_height': -0.15,
                    'max_height': 0.10,
                    'angle_min': -0.8,
                    'angle_max': 0.8, 
                    'angle_increment': 0.0087, 
                    'scan_time': 0.033, 
                    'range_min': 0.20, 
                    'range_max': 1.0, 
                    'use_inf': True, 
                    'inf_epsilon': 1.0,
                    'use_sim_time': False
                }],
                output='screen'
            )
        ]
    )

    return [
        robot_state_publisher,
        tf_front_level,
        tf_rear_level,
        livox_driver,
        front_camera,
        pct_to_scan_front_node,
        rear_camera_group_delayed
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])