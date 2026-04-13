import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取功能包路径
    rm_nav_pkg = get_package_share_directory('rm_nav_bringup')
    nav2_launch_dir = os.path.join(get_package_share_directory('rm_navigation'), 'launch')

    # 2. 声明 Launch 参数 (统一管理传参)
    world_arg = DeclareLaunchArgument(
        'world', 
        description='MUST PROVIDE: Map name without extension (e.g., world:=forest_map)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='False', 
        description='Use simulation time'
    )
    show_rviz_arg = DeclareLaunchArgument(
        'show_rviz', 
        default_value='True', 
        description='Set to true to launch RViz automatically for navigation'
    )
    
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    show_rviz = LaunchConfiguration('show_rviz')

    # 3. 配置文件路径统一定义
    measurement_params_path = os.path.join(rm_nav_pkg, 'config', 'reality', 'measurement_params_real.yaml')
    segmentation_params = os.path.join(rm_nav_pkg, 'config', 'reality', 'segmentation_real.yaml')
    fastlio_params = os.path.join(rm_nav_pkg, 'config', 'reality', 'fastlio_mid360_real.yaml')
    slam_toolbox_params = os.path.join(rm_nav_pkg, 'config', 'reality', 'mapper_params_localization_real.yaml')
    nav2_params = os.path.join(rm_nav_pkg, 'config', 'reality', 'nav2_params_real.yaml')
    mid360_config_path = os.path.join(rm_nav_pkg, 'config', 'reality', 'MID360_config.json')

    # 动态拼接地图文件路径
    slam_toolbox_map_path = PathJoinSubstitution([rm_nav_pkg, 'map', world])
    nav2_map_path = [PathJoinSubstitution([rm_nav_pkg, 'map', world]), '.yaml']

    # 4. 解析 Xacro 与外参
    with open(measurement_params_path, 'r') as f:
        launch_params = yaml.safe_load(f)
    
    # 使用 Command 动态生成 robot_description
    robot_description_content = Command([
        'xacro ', os.path.join(rm_nav_pkg, 'urdf', 'sentry_robot_real.xacro'),
        ' xyz:=', launch_params['base_link2livox_frame']['xyz'], 
        ' rpy:=', launch_params['base_link2livox_frame']['rpy']
    ])

    # 5. 定义所有 Node 节点
    
    # 5.1 机器人状态发布 (骨架 TF)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}],
        output='screen'
    )

    # 5.2 Livox 驱动
    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        parameters=[
            {"xfer_format": 4}, {"multi_topic": 0}, {"data_src": 0},
            {"publish_freq": 10.0}, {"output_data_type": 0}, {"frame_id": 'livox_frame'},
            {"lvx_file_path": '/home/livox/livox_test.lvx'}, 
            {"user_config_path": mid360_config_path},
            {"cmdline_input_bd_code": 'livox0000000001'}
        ],
        output='screen'
    )

    # 5.3 IMU 互补滤波
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        parameters=[{'do_bias_estimation': True}, {'do_adaptive_gain': True}, 
                    {'use_mag': False}, {'gain_acc': 0.01}, {'gain_mag': 0.01}],
        remappings=[('/imu/data_raw', '/livox/imu')],
        output='screen'
    )

    # 5.4 激光地面分割
    ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        parameters=[segmentation_params],
        output='screen'
    )

    # 5.5 车体自过滤
    filter_node = Node(
        package='filter',
        executable='filter_node',
        name='filter_node',
        output='screen'
    )

    # 5.6 3D 转 2D 激光雷达 (保留了树林防挂底/防树枝的绝佳切片)
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
            'min_height': -0.06,     # 防挂底：只看4cm以上的石头和树根
            'max_height': 0.70,      # 防钻林：无视车顶以上的垂枝和树叶
            'angle_min': -3.14159, 
            'angle_max': 3.14159, 
            'angle_increment': 0.0043, 
            'scan_time': 0.3333, 
            'range_min': 0.15, 
            'range_max': 20.0,       # 匹配室外大范围视野
            'use_inf': True, 
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # === [新增] 前置相机点云转 2D 节点 ===
    # pct_to_scan_front_node = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pct_to_scan_front',
    #     remappings=[
    #         ('cloud_in', '/camera_front/depth/color/points'), # 订阅前置相机3D点云
    #         ('scan', '/scan_front')                           # 输出新的前置2D雷达话题
    #     ],
    #     parameters=[{
    #         'target_frame': 'base_link',    # 统一投影到车体坐标系
    #         'transform_tolerance': 0.05,
    #         'min_height': 0.06,             # 【防误报】：过滤离地 6cm 以下的地面坑洼
    #         'max_height': 0.50,             # 【防钻林】：无视 50cm 以上悬空的树枝
    #         'angle_min': -1.0,              # 相机水平视场角（约左右各57度左右，视相机而定）
    #         'angle_max': 1.0, 
    #         'angle_increment': 0.0087,      # 角分辨率约 0.5 度
    #         'scan_time': 0.033,             # 相机帧率通常30Hz (1/30s)
    #         'range_min': 0.2,               # 最近识别距离
    #         'range_max': 3.0,               # 最远信任距离 3米
    #         'use_inf': True, 
    #         'inf_epsilon': 1.0,
    #         'use_sim_time': use_sim_time
    #     }],
    #     output='screen'
    # )

    # === [新增] 后置相机点云转 2D 节点 ===
    # pct_to_scan_rear_node = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pct_to_scan_rear',
    #     remappings=[
    #         ('cloud_in', '/camera_rear/depth/color/points'), # 订阅后置相机3D点云
    #         ('scan', '/scan_rear')                           # 输出新的后置2D雷达话题
    #     ],
    #     parameters=[{
    #         'target_frame': 'base_link',
    #         'transform_tolerance': 0.05,
    #         'min_height': 0.06,             # 离地 6cm
    #         'max_height': 0.50,             # 离地 50cm
    #         'angle_min': -1.0,
    #         'angle_max': 1.0, 
    #         'angle_increment': 0.0087, 
    #         'scan_time': 0.033, 
    #         'range_min': 0.2, 
    #         'range_max': 3.0, 
    #         'use_inf': True, 
    #         'inf_epsilon': 1.0,
    #         'use_sim_time': use_sim_time
    #     }],
    #     output='screen'
    # )

    # 5.7 静态 TF 发布器 (odom -> lidar_odom)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom_to_lidar_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'lidar_odom'],
        output='screen'
    )

    # 5.8 FAST-LIO 高频里程计
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[fastlio_params, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 5.9 SLAM Toolbox (纯定位模式，加载地图)
    slam_toolbox_localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',   # 注意这里是 localization 节点
        parameters=[
            slam_toolbox_params, 
            {
                'use_sim_time': use_sim_time,
                'map_file_name': slam_toolbox_map_path, # 将外部传入的地图传给定位算法
                'map_start_pose': [0.0, 0.0, 0.0]       # 默认从原点启动匹配
            }
        ],
        output='screen'
    )

    # 5.10 Nav2 导航系统与 TEB 局部规划器
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_rm_navigation.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav2_map_path,            # 导航所需的代价地图底图 (.yaml)
            'params_file': nav2_params,      # 包含了 TEB 和代价地图参数
            'nav_rviz': show_rviz            # 是否打开 RViz
        }.items()
    )

    # 5.11 底层串口通信桥
    serial_bridge_node = Node(
        package='serial_bridge',
        executable='serial_bridge',
        name='stm32_driver',
        output='screen',
        parameters=[{'port': '/dev/ttyUSB0', 'baudrate': 115200}]
    )

    # 5.12 前置 Gemini Pro 节点
    orbbec_pkg_dir = get_package_share_directory('astra_camera')
    
    front_camera_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(orbbec_pkg_dir, 'launch', 'gemini.launch.xml')),
        launch_arguments={
            'camera_name': 'camera_front',
            'serial_number': 'AY27552006X', # 强烈建议绑定SN码，防止双相机USB号反转
            'enable_point_cloud': 'true',
            'enable_depth': 'true',
            'depth_registration': 'false', 
            'enable_colored_point_cloud': 'false', # 关闭彩色点云以节省带宽
            'point_cloud_qos': 'sensor_data',
            # 开启降采样滤波器，降低处理负荷
            'enable_decimation_filter': 'true'     
        }.items()
    )

    # 5.13 后置 Gemini Pro 节点
    rear_camera_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(orbbec_pkg_dir, 'launch', 'gemini.launch.xml')),
        launch_arguments={
            'camera_name': 'camera_rear',
            'serial_number': 'AY2755200S5', 
            'enable_point_cloud': 'true',
            'enable_depth': 'true',
            'depth_registration': 'false',
            'enable_colored_point_cloud': 'false',
            'point_cloud_qos': 'sensor_data',
            'enable_decimation_filter': 'true'
        }.items()
    )

    # 6. 返回启动描述
    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        show_rviz_arg,
        rsp_node,
        front_camera_node,
        rear_camera_node,
        livox_node,
        imu_filter_node,
        ground_segmentation_node,
        filter_node,
        pct_to_scan_node,
        static_tf_node,
        fast_lio_node,
        slam_toolbox_localization_node,
        nav2_launch,
        # pct_to_scan_front_node,   
        # pct_to_scan_rear_node,    
        serial_bridge_node
    ])