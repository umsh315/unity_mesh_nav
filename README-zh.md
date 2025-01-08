系统包含以下模块：
- SLAM模块
- 路径规划器
- 基础自主系统
  其中基础自主系统进一步包括用于：
  - 地形可通行性分析
  - 避障
  - 航点跟踪
的基础导航模块。


用户可以选择在以下环境运行系统：
- Go2的板载计算机
- 通过以太网电缆连接到Go2的外部计算机

请确保使用具有SDK支持的Go2 EDU版本。



下载Go2的Unity环境模型，并解压到'src/base_autonomy/vehicle_simulator/mesh/unity'文件夹。环境模型文件结构应如下所示：
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



总结三种操作模式：
1. 航点导航模式：设置目标点自主导航
    车辆会尝试跟随航点并同时避免碰撞。可以通过以下方式操作：
    1. 使用RVIZ设置航点：
    - 首先点击'Waypoint'按钮
    - 然后在车辆周围点击要设置航点的位置

    2. 模式切换：
    - 如果系统在其他模式下，点击RVIZ中的'Resume Navigation to Goal'按钮可切换到航点模式
    - 或者，用户可以按住控制器上的'waypoint-mode'按钮，并使用右摇杆设置速度
    - 如果只是按住'waypoint-mode'按钮，系统将使用ROS消息中发送的速度


2. 智能摇杆模式：手动控制+自动避障(在这种模式下，车辆会尝试跟随摇杆命令的同时避免碰撞)
    车辆会尝试跟随摇杆命令的同时避免碰撞。可以通过以下方式控制：
    - 使用RVIZ中的控制面板
    - 使用控制器上的右摇杆来设置速度和偏航角速度
    如果系统当前在其他模式下，执行上述操作会将系统切换到智能摇杆模式。

3. 完全手动模式：纯手动控制，无避障功能
车辆会直接跟随摇杆命令，没有任何避障功能。操作方法：

    1. 模式切换：
    - 按下控制器上的'manual-mode'按钮切换到手动模式

    2. 控制方式（采用Mode 2约定）：
    - 使用右摇杆设置前进和横向速度
    - 使用左摇杆设置偏航角速度



另外，用户也可以运行一个ROS节点来发送一系列航点。具体操作如下：

1. 在另一个终端中：
   - 进入指定文件夹
   - 加载ROS工作空间
   - 使用下面的命令行运行ROS节点
source install/setup.sh
ros2 launch waypoint_example waypoint_example.launch

2. 该ROS节点同时会发送：
   - 导航边界
   - 速度信息

3. 操作步骤：
   点击RVIZ中的'Resume Navigation to Goal'按钮，车辆将在指定边界内按照航点进行导航。


# 路径规划器
## 基本操作
用户可以使用RVIZ中的'Goalpoint'按钮设置目标点。车辆会导航到目标点，并在途中构建可视图（显示为青色）。被可视图覆盖的区域将被标记为自由空间。

## 控制选项
1、可视图控制：
- 点击'Reset Visibility Graph'按钮：重新初始化可视图
- 取消勾选'Update Visibility Graph'复选框：停止更新可视图

2、规划模式控制：
通过'Planning Attemptable'复选框：
- 勾选时：规划器会先尝试在自由空间中找路径（路径显示为绿色）
- 取消勾选时：如果在自由空间中找不到路径，规划器会同时考虑未知空间（路径显示为蓝色）

## 操作模式切换
使用路径规划器导航时，基础系统运行在航点模式下。用户可以：

1. 切换到智能摇杆模式：
   - 点击控制面板中的黑色框
   - 或使用摇杆控制器的按钮

2. 切换到手动模式：
   - 使用摇杆控制器的按钮

3. 恢复路径规划导航：
   - 点击RVIZ中的'Resume Navigation to Goal'按钮
   - 或使用'Goalpoint'按钮设置新的目标点





# 实物测试
## 在拓展坞安装
1、安装依赖

sudo apt update
sudo apt install libusb-dev ros-$ROS_DISTRO-perception-pcl ros-$ROS_DISTRO-sensor-msgs-py ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-rosidl-generator-dds-idl
pip install transforms3d pyyaml


### 在上位机安装
1、安装依赖

sudo apt update
sudo apt install -y libusb-dev ros-$ROS_DISTRO-perception-pcl ros-$ROS_DISTRO-sensor-msgs-py ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-rmw-cyclonedds-cpp ros-$ROS_DISTRO-rosidl-generator-dds-idl ros-$ROS_DISTRO-tf2-sensor-msgs python3-colcon-common-extensions python-is-python3 gstreamer1.0-plugins-bad gstreamer1.0-libav
pip install transforms3d pyyaml

GStreamer LibAV插件
- 提供音视频编解码支持
- 基于FFmpeg/LibAV
- 支持常见多媒体格式

2、IMU校准
source install/setup.bash
ros2 run calibrate_imu calibrate_imu

3、启动节点

这将启动：
- SLAM模块
- 基础自主系统
./system_real_robot.sh

要使用路线规划器启动系统，请使用下面的命令行。
./system_real_robot_with_route_planner.sh


4、操控和启动相机

#### 关于以下模式的操作方法，请参考"仿真设置"部分：
- 智能摇杆模式
- 航点模式
- 手动模式

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





