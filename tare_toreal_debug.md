# 真机部署方法

## 一、原理

## 二、中间层参数调整
1、将机器人上的车辆模拟器替换为具体的LOAM模块，并将 ROS 话题 /cmd_vel 上的 geometry_msgs::TwistStamped 类型的命令速度消息转发到运动控制器。

2、调整 src/local_planner/launch/local_planner.launch 中的 minRelZ 和 maxRelZ 参数，以裁剪注册扫描中的地面和天花板。在车辆模拟器中，默认的传感器高度设置为距离地面0.75米，并且注册扫描在相对于传感器的高度-0.5米和0.25米处进行裁剪。

3、sensorscanGeneration节点：
    1）订阅话题："/state_estimation"，"/registered_scan"
    2）发布话题：/state_estimation_at_scan——改了里程计的子坐标系，重命名了一个新的里程计话题；/sensor_scan——作了坐标系转换

4、使用unitree自带的point-lio，需要
    注意：在 src/loam_interface/launch/loam_interface.launch 中，设置 stateEstimationTopic = /aft_mapped_to_init，registeredScanTopic = /cloud_registered，flipStateEstimation = false，以及 flipRegisteredScan = false。

5、处理负障碍：虽然处理负障碍的最佳方法是在机器人高处安装一个传感器，向下观察负障碍，但一个快速的解决方案是打开地形分析并在“src/terrain_analysis/launch/terrain_analysis.launch”中设置“noDataObstacle = true”。负障碍通常会导致某些区域没有数据。系统会将这些区域视为不可穿越。

6、调整路径跟随：要更改行驶速度，请调整“src/local_planner/launch/local_planner.launch”中的“maxSpeed”、“autonomySpeed”和“maxAccel”。其他路径跟随参数（如“maxYawRate”、“yawRateGain”和“lookAheadDis”）位于同一文件中。如果设置“twoWayDrive = false”，机器人将仅向前行驶。


7、其他待确认
    1、cmakefile文件中，是否需要补充tf2_sensor_msgs
    2、cmakefile文件含义

5、其他参数调整方法
    参考https://github.com/jizhang-cmu/ground_based_autonomy_basic/tree/noetic

## 启动
    ros2 launch vehicle_simulator system_real_robot.launch