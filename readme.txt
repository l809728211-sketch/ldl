安装依赖 rosdep install --from-paths src --ignore-src -r -y

雷达ip 192.168.1.181

建图算法 FAST_LIO 定位 slam_toolbox 需要地图文件格式 .data .posegraph

参数文件 fastlio_mid360_real.yaml 雷达
        mapper_params_localization_real.yaml slam_toolbox导航时定位参数文件
        mapper_params_online_async_real.yaml slam_toolbox建图时定位参数文件
        measurement_params_real.yaml 雷达安装位置
        MID360_config.json 雷达ip设置
        nav2_params_real.yaml 导航参数文件
        segmentation_real.yaml 地面点云分割参数文件

filter 雷达点云过滤功能包 livox_ros_driver2 雷达驱动功能包 fast_lio 建图算法功能包 imu_complementary_filter imu互补滤波功能包 
linefit_ground_segmentation 地面点云分割功能包 pointcloud_to_laserscan 点云转换功能包 rm_nav_bringup 算法启动功能包
rm_navigation 导航算法功能包 

建图命令 source install/setup.bash
        ros2 launch rm_nav_bringup mapping.launch.py 

离线建图 终端1 ros2 launch rm_nav_bringup mapping.launch.py
        终端2 ros2 bag record -o forest_mapping_bag_02 /livox/lidar /livox/imu
        forest_mapping_bag_02 rosbag包名
        注： “热身”静止（前10秒）： 录制开始后，千万不要马上推摇杆！ 让小车在原地静止 5 到 10 秒钟。这是为了让 FAST-LIO 算法能够收集足够的静止 IMU 数据，计算出正确的重力方向和零偏。
             原地微动（第10-20秒）： 极其缓慢地原地左转一下、右转一下，让雷达扫清四周的初始环境。
             龟速行驶： 树林里坑洼多，剧烈颠簸加上快速旋转会让 FAST-LIO 算法“飞掉”。请以**极低的速度（比如 0.3 m/s）**匀速行驶，特别是转弯时，要像老奶奶推车一样慢。
             主动制造“闭环”： 如果您要在树林里绕一圈，一定要确保最终把小车开回起点的同一个位置（正对着同一棵树）。这会让离线建图时的 SLAM Toolbox 能够成功识别出起点，消除这一圈累积的误差。

回放 终端1 ros2 launch rm_nav_bringup mapping.launch.py use_sim_time:=True
     终端2 ros2 bag play forest_mapping_bag_01 --clock
     注： 如果您觉得车速太快 FAST-LIO 处理不过来，可以在播放时加上慢速参数，例如 ros2 bag play forest_mapping_bag_01 --clock -r 0.5 (以 0.5 倍速播放)。

地图保存 rviz->Panels->Add New Panels->SlamToolboxPlugin->save Map（.pgm .yaml） serialize Map（.data .posegraph）
        PCD ros2 service call /map_save std_srvs/srv/Trigger

导航命令 ros2 launch rm_nav_bringup navigation.launch.py world:=<你的地图名>
        注： 地图要保存到 install/rm_nav_bringup/map/ 目录下




    
