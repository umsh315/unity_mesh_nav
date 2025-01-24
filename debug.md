
# tare调试问题

1、explore.launch文件如果嵌套启动，需手动给个启动waypoint，主要时中间层先要有个路点，不能在默认位置，哪怕此时explore节点还没启动
2、仿真环境参数已验证有效，待同步到真实环境，但目前参数不稳定


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