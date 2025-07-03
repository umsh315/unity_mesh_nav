# Go2 path planner algorithm

## 1, システム内のモジュール
- SLAMモジュール
- パスプランナー
- 基礎自立システム
  更に以下を含む：
  - 地形通過正分析
  - 障害物回避
  - ウェイポイント追跡
ナビゲーションモジュール


## 2, シミュレーション環境の構築
### シミュレーションモデルのロード
   1、Go2用のunity環境をダウンロードし、'src/base_autonomy/vehicle_simulator/mesh/unity'フォルダに解凍する。
   環境モデルのファイル構造は以下のようにする：
   mesh/
      unity/
         environment/
               Model_Data/ (複数ファイルを含む)
               Model.x86_64
               UnityPlayer.so
               AssetList.csv (実行時に生成)
               Dimensions.csv
               Categories.csv
         map.ply                # 地図ファイル
         object_list.txt        # オブジェクトリスト
         traversable_area.ply   # 通過可否エリア
         map.jpg               # 地図画像
         render.jpg            # レンダリングイメージ

### シミュレーションの起動
#### ビルド
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


#### No view mode
   ./system_simulation.sh

#### GUIを用いた可視化モード
   ./system_simulation_with_route_planner.sh
   ./system_simulation_with_exploration_planner.sh


   1、可視化モードコントロール：
   - 'Reset Visibility Graph'ボタン：可視化グラフの更新
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
   colcon build --packages-select far_planner
   colcon build --packages-select go2_h264_repub
   colcon build --packages-select tare_planner

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
   ./system_real_robot_with_exploration_planner.sh

#### 启动AI运控

ros2 launch go2_cmd go2_ai_cmd_test.launch.py



#### 启动相机驱动：
   1. 在新的终端中：
      - 进入代码库文件夹
      - 使用下面的命令行
      2) source install/setup.bash
      3) ros2 run go2_h264_repub go2_h264_repub --ros-args -p multicast_iface:=wlp0s20f3
      (网口根据自己实际情况调整)
      

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



# 保存slam的建图结果地图

## 改参数文件
   修改utlidar.yaml文件中pcd_save_en参数（目前是设为false）
## 编译
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 
## 建图
   1、与go2建立通信

   2、单独启动point-lio建图以下2个都可以
      ros2 launch vehicle_simulator system_real_robot_only_slam.launch
      ros2 launch point_lio_unilidar mapping_utlidar.launch 

   3、结合自探索建图
      ./system_real_robot_with_tare_planner.sh
      ros2 launch tare_planner explore.launch

## 保存地图
   地图保存在PCD文件夹中，使用以下命令可以查看
   pcl_viewer scans.pcd 

   下次建图时，将上次的文件删除

