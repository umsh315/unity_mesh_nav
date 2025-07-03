# Go2 path planner algorithm

## 1, システム内のモジュール
- SLAMモジュール
- パスプランナー
- 基礎自律システム
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
   - チェックボックス'Update Visibility Graph'のチェックを外す：更新を停止

   2、planning mode control：
   チェックボックス'Planning Attemptable'を有効化：
   - 有効時：自由空間で経路を探索（経路は緑色で表示される）
   - 無効時：自由空間で経路が見つからなかった場合，未知の空間も考慮して経路を探索する（経路は青色で表示される）



## 3, 実機テスト
### Docking stationへのインストール
#### 依存関係のインストール

   sudo apt update
   sudo apt install libusb-dev ros-$ROS_DISTRO-perception-pcl ros-$ROS_DISTRO-sensor-msgs-py ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-rosidl-generator-dds-idl
   pip install transforms3d pyyaml


### 外部ホストPCへのインストール
#### 依存関係のインストール

   sudo apt update
   sudo apt install -y libusb-dev ros-$ROS_DISTRO-perception-pcl ros-$ROS_DISTRO-sensor-msgs-py ros-$ROS_DISTRO-tf-transformations ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-rmw-cyclonedds-cpp ros-$ROS_DISTRO-rosidl-generator-dds-idl ros-$ROS_DISTRO-tf2-sensor-msgs python3-colcon-common-extensions python-is-python3 gstreamer1.0-plugins-bad gstreamer1.0-libav
   pip install transforms3d pyyaml

   GStreamer LibAVプラグイン
   - オーディオ及び, ビデオコーデックのサポート
   - FFmpeg/LibAV base
   - マルチメディアフォーマットのサポート

#### パッケージのコンパイル
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   colcon build --packages-select far_planner
   colcon build --packages-select go2_h264_repub
   colcon build --packages-select tare_planner

#### IMU calibration
   source install/setup.bash
   ros2 run calibrate_imu calibrate_imu

#### nodeの起動

   基本的な起動：
   - SLAMモジュール
   - 基本的な自律システム
   ./system_real_robot.sh

   ルートプランナーを含む起動command
   ./system_real_robot_with_route_planner.sh
   ./system_real_robot_with_exploration_planner.sh

#### AI制御の起動

ros2 launch go2_cmd go2_ai_cmd_test.launch.py



#### カメラドライバの起動：
   1. 新規ターミナルの作成：
      - プロジェクトフォルダへ移動
      - 下記コマンドの起動
      2) source install/setup.bash
      3) ros2 run go2_h264_repub go2_h264_repub --ros-args -p multicast_iface:=wlp0s20f3
      (ネットワークインターフェース名は別途変更)
      

   2. カメラドライブ機能：
      - H.264ビデオストリームを受信
      - '/camera/image/raw'に画像を配信


# 備考
1、ライダーの限界

Unitree L1 Lidar：
- 低コスト
- ノイズが大きい
- 低い障害物の認識が困難
- 環境内の障害物は地上0.3m以上である必要がある



2、
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

