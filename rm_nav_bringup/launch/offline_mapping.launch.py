import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    # 1. 获取功能包路径
    rm_nav_pkg = get_package_share_directory('rm_nav_bringup')
    
    # 获取外部传入的 rosbag 路径
    bag_path_arg = DeclareLaunchArgument('bag_path', description='Path to the rosbag file')
    bag_path = LaunchConfiguration('bag_path')

    # === 极其关键：全局强制使用仿真时间 ===
    set_sim_time = SetParameter(name='use_sim_time', value=True)

    # === 播放数据包 ===
    play_rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock', 
             '--remap', '/tf:=/tf_old', '/tf_static:=/tf_static_old'],
        output='screen'
    )

    # 2. 配置文件路径统一定义
    measurement_params_path = os.path.join(rm_nav_pkg, 'config', 'reality', 'measurement_params_real.yaml')
    segmentation_params = os.path.join(rm_nav_pkg, 'config', 'reality', 'segmentation_real.yaml')
    fastlio_params = os.path.join(rm_nav_pkg, 'config', 'reality', 'fastlio_mid360_real.yaml')
    slam_toolbox_params = os.path.join(rm_nav_pkg, 'config', 'reality', 'mapper_params_online_async_real.yaml')
    rviz_config_dir = os.path.join(rm_nav_pkg, 'rviz', 'mapping.rviz')

    # 3. 解析 Xacro 与外参
    with open(measurement_params_path, 'r') as f:
        launch_params = yaml.safe_load(f)
    
    robot_description_content = Command([
        'xacro ', os.path.join(rm_nav_pkg, 'urdf', 'sentry_robot_real.xacro'),
        ' xyz:=', launch_params['base_link2livox_frame']['xyz'], 
        ' rpy:=', launch_params['base_link2livox_frame']['rpy']
    ])

    # 4. 定义所有 Node 节点 (已剔除 Livox 真机驱动)
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )

    # [修复点 1] 补全了 IMU 滤波的重映射，解决 FAST-LIO 饿死问题
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        parameters=[{'do_bias_estimation': True}, {'do_adaptive_gain': True}, 
                    {'use_mag': False}, {'gain_acc': 0.01}, {'gain_mag': 0.01}],
        remappings=[('/imu/data_raw', '/livox/imu')],
        output='screen'
    )

    ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        parameters=[segmentation_params],
        remappings=[
            ('/livox/lidar/pointcloud', '/cloud_registered_body')
        ],
        output='screen'
    )

    filter_node = Node(
        package='filter',
        executable='filter_node',
        name='filter_node',
        output='screen'
    )

    # [修复点 2] 恢复 2D 点云处理流水线，使用你的 filter_node 输出
    pct_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/processed_pointcloud'), 
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link', 
            'transform_tolerance': 0.01,
            'min_height': -0.06, 
            'max_height': 0.70, 
            'angle_min': -3.14159, 
            'angle_max': 3.14159, 
            'angle_increment': 0.0043, 
            'scan_time': 0.3333, 
            'range_min': 0.15, 
            'range_max': 25.0, 
            'use_inf': True, 
            'inf_epsilon': 1.0
        }],
        output='screen'
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom_to_lidar_odom',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'lidar_odom'],
        output='screen'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            fastlio_params,
            {'publish_tf': True} 

        ],
        output='screen'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[slam_toolbox_params],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
        bag_path_arg,
        set_sim_time,
        play_rosbag,
        rsp_node,
        imu_filter_node,
        ground_segmentation_node,
        filter_node,
        pct_to_scan_node,
        static_tf_node,
        fast_lio_node,
        slam_toolbox_node,
        rviz_node
    ])