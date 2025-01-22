# 在GO2实物平台上调试FAR planner算法

## 一、系统包含模块
- SLAM模块
- 路径规划器
- 基础自主系统
  其中基础自主系统进一步包括用于：
  - 地形可通行性分析
  - 避障
  - 航点跟踪
的基础导航模块。


## 二、仿真环境构建
### 仿真模型加载
   1、下载Go2的Unity环境模型，并解压到'src/base_autonomy/vehicle_simulator/mesh/unity'文件夹。环境模型文件结构应如下所示：
   mesh/
      unity/
         environment/
               Model_Data/ (文件夹中包含多个文件)
               Model.x86_64
               UnityPlayer.so
               AssetList.csv (运行时生成)
               Dimensions.csv
               Categories.csv
         map.ply                # 地图文件
         object_list.txt        # 对象列表
         traversable_area.ply   # 可通行区域
         map.jpg               # 地图图片
         render.jpg            # 渲染图片

### 启动仿真环境
#### 构建
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

#### 无可视图模式
   ./system_simulation.sh

#### 有可视图模式
   ./system_simulation_with_route_planner.sh
   ./system_simulation_with_tare_planner.sh

   1、可视图控制：
   - 点击'Reset Visibility Graph'按钮：重新初始化可视图
   - 取消勾选'Update Visibility Graph'复选框：停止更新可视图

   2、规划模式控制：
   通过'Planning Attemptable'复选框：
   - 勾选时：规划器会先尝试在自由空间中找路径（路径显示为绿色）
   - 取消勾选时：如果在自由空间中找不到路径，规划器会同时考虑未知空间（路径显示为蓝色）



## 三、实物测试
### 在拓展坞安装
#### 安装依赖

   sudo apt update
   sudo apt install libusb-dev ros-$ROS_DISTRO-perception-pcl ros-$ROS_DISTRO-sensor-msgs-py ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-rosidl-generator-dds-idl
   pip install transforms3d pyyaml


### 在上位机安装
#### 安装依赖

   sudo apt update
   sudo apt install -y libusb-dev ros-$ROS_DISTRO-perception-pcl ros-$ROS_DISTRO-sensor-msgs-py ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-rmw-cyclonedds-cpp ros-$ROS_DISTRO-rosidl-generator-dds-idl ros-$ROS_DISTRO-tf2-sensor-msgs python3-colcon-common-extensions python-is-python3 gstreamer1.0-plugins-bad gstreamer1.0-libav
   pip install transforms3d pyyaml

   GStreamer LibAV插件
   - 提供音视频编解码支持
   - 基于FFmpeg/LibAV
   - 支持常见多媒体格式

#### 编译功能包
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

#### IMU校准
   source install/setup.bash
   ros2 run calibrate_imu calibrate_imu

#### 启动节点

   这将启动：
   - SLAM模块
   - 基础自主系统
   ./system_real_robot.sh

   要使用路线规划器启动系统，请使用下面的命令行。
   ./system_real_robot_with_route_planner.sh
   ./system_real_robot_with_tare_planner.sh


ros2 launch far_planner far_planner.launch

ros2 launch tare_planner explore.launch

1、试一下用套件仿真时，话题订阅情况，关掉overmap,——要缩小了才能看到已探索区域，且在不同rviz上展示不同
1、探索两步就停了

#### 启动相机驱动：
   1. 在新的终端中：
      - 进入代码库文件夹
      - 使用下面的命令行
   source install/setup.bash
   ros2 run go2_h264_repub go2_h264_repub

   2. 相机驱动功能：
      - 接收H.264视频流
      - 在'/camera/image/raw'话题上发布图像


# 备注
1、激光雷达限制

L1激光雷达特点：
- 低成本
- 噪声较高
- 地形可通行性分析无法识别低障碍物
- 环境中障碍物需要高于地面0.3米以上



2、角点
   1. 尖角（Sharp Corner）
   曲率变化最大的点
   通常是墙角、物体边缘等

   2. 钝角（Less Sharp）
   曲率变化次之的点
   通常是较平缓的转角

3、V-graph



# 单独测试point-lio slam

## 改参数文件
   修改utlidar.yaml文件中pcd_save_en参数
## 编译
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 
## 建图
   1、与go2建立通信
   2、使用独立的launch文件，以下2个都可以
      ros2 launch vehicle_simulator system_real_robot_only_slam.launch
      ros2 launch point_lio_unilidar mapping_utlidar.launch 
## 保存地图
   地图保存在PCD文件夹中，使用以下命令可以查看
   pcl_viewer scans.pcd 

