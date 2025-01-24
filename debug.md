
# tare调试问题

1、explore.launch文件如果嵌套启动，需手动给个启动waypoint，主要时中间层先要有个路点，不能在默认位置，哪怕此时explore节点还没启动
2、仿真环境参数已验证有效，待同步到真实环境，但目前参数不稳定


可能导致探索不完全原因
1、看 terrain_map 输出，会不会因为噪点比较多，当成障碍物了
2、看源码，导致Exploration completed, returning home的原因
    1、selected_viewpoint_indices_itr为空
        在local_coverage_planner源码中增加调试信息，观测什么原因导致为空，尚未编译
    2、exploring_cell_indices为空
        grid_world源码中增加调试信息，观测什么原因导致exploring_cell_indices为空，尚未编译
3、跟use_boundary有关？要用explore_matterport.launch启动
4、直接用cmu给好的indoor参数文件试下
5、


3、真机验证有效，可同步开启建图开关



colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select tare_planner


./system_simulation_with_tare_planner.sh
ros2 launch tare_planner explore.launch


## 真机
./system_real_robot_with_tare_planner.sh
ros2 launch tare_planner explore.launch

# far planner调试问题
1、如果确实没有路，还是会不停的规划，甚至撞门
2、仅从rviz上看，把机器人模型简化为一个圆形，没考虑实际尺寸，实际是多边形
3、建图保存在哪里了，要验证下——需要开启一个参数


# tare planner问题

# local planner问题
在室内环境中，为了避免低矮障碍物，用户可以将 src/base_autonomy/local_planner/launch/local_planner.launch 文件中的 obstacleHeightThre 从 0.15 降低到 0.015 或 0.02。这样，车辆将避开距离地面2-2.5厘米的障碍物。

在室外环境中，请将阈值设置得更高（0.1-0.15）。


## AI模块

在启动AI模型时，请在终端中设置 export ROS_DOMAIN_ID=1。

请注意，如果添加了摄像头，数据传输需要将压缩图像发送到基站计算机。请在NUC i7计算机上的 /camera/image/compressed 主题上准备压缩图像。图像在传输后会在 base_station.sh 脚本中解压。
在基站计算机上，请确保订阅 /camera/image/transmitted 主题上的未压缩图像。