#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

// For real robot
#include "unitree_api/msg/request.hpp"                                // 引入宇树机器人API请求消息头文件
#include "common/ros2_sport_client.h"                                // 引入运动客户端头文件

using namespace std;                                                 // 使用标准命名空间

const double PI = 3.1415926;                                        // 定义圆周率常量

double sensorOffsetX = 0;                                           // 传感器X轴偏移量
double sensorOffsetY = 0;                                           // 传感器Y轴偏移量
int pubSkipNum = 1;                                                // 发布跳过数量
int pubSkipCount = 0;                                              // 发布跳过计数器
bool twoWayDrive = true;                                           // 双向驱动开关
double lookAheadDis = 0.5;                                         // 前视距离
double yawRateGain = 7.5;                                          // 偏航角速度增益
double stopYawRateGain = 7.5;                                      // 停止时偏航角速度增益
double maxYawRate = 45.0;                                          // 最大偏航角速度
double maxSpeed = 1.0;                                             // 最大速度
double maxAccel = 1.0;                                             // 最大加速度
double switchTimeThre = 1.0;                                       // 切换时间阈值
double dirDiffThre = 0.1;                                          // 方向差异阈值
double omniDirDiffThre = 1.5;                                      // 全向方向差异阈值
double noRotSpeed = 10.0;                                          // 无旋转速度
double stopDisThre = 0.2;                                          // 停止距离阈值
double slowDwnDisThre = 1.0;                                       // 减速距离阈值
bool useInclRateToSlow = false;                                    // 使用倾角速率减速开关
double inclRateThre = 120.0;                                       // 倾角速率阈值
double slowRate1 = 0.25;                                           // 减速率1
double slowRate2 = 0.5;                                            // 减速率2
double slowTime1 = 2.0;                                            // 减速时间1
double slowTime2 = 2.0;                                            // 减速时间2
bool useInclToStop = false;                                        // 使用倾角停止开关
double inclThre = 45.0;                                            // 倾角阈值
double stopTime = 5.0;                                             // 停止时间
bool noRotAtStop = false;                                          // 停止时不旋转开关
bool noRotAtGoal = true;                                           // 到达目标时不旋转开关
bool manualMode = false;                                           // 手动模式开关
bool autonomyMode = false;                                         // 自主模式开关
double autonomySpeed = 1.0;                                        // 自主模式速度
double joyToSpeedDelay = 2.0;                                      // 手柄到速度延迟
double goalCloseDis = 1.0;                                         // 接近目标距离
bool is_real_robot = false;                                        // 是否为真实机器人

float joySpeed = 0;                                                // 手柄速度
float joySpeedRaw = 0;                                            // 原始手柄速度
float joyYaw = 0;                                                 // 手柄偏航角
float joyManualFwd = 0;                                           // 手动前进速度
float joyManualLeft = 0;                                          // 手动左转速度
float joyManualYaw = 0;                                           // 手动偏航速度
int safetyStop = 0;                                               // 安全停止标志

float vehicleX = 0;                                               // 车辆X坐标
float vehicleY = 0;                                               // 车辆Y坐标
float vehicleZ = 0;                                               // 车辆Z坐标
float vehicleRoll = 0;                                            // 车辆横滚角
float vehiclePitch = 0;                                           // 车辆俯仰角
float vehicleYaw = 0;                                             // 车辆偏航角

float vehicleXRec = 0;                                            // 记录的车辆X坐标
float vehicleYRec = 0;                                            // 记录的车辆Y坐标
float vehicleZRec = 0;                                            // 记录的车辆Z坐标
float vehicleRollRec = 0;                                         // 记录的车辆横滚角
float vehiclePitchRec = 0;                                        // 记录的车辆俯仰角
float vehicleYawRec = 0;                                          // 记录的车辆偏航角

float vehicleYawRate = 0;                                         // 车辆偏航角速度
float vehicleSpeed = 0;                                           // 车辆速度

double odomTime = 0;                                              // 里程计时间
double joyTime = 0;                                               // 手柄时间
double slowInitTime = 0;                                          // 减速初始时间
double stopInitTime = false;                                      // 停止初始时间
int pathPointID = 0;                                              // 路径点ID
bool pathInit = false;                                            // 路径初始化标志
bool navFwd = true;                                               // 前向导航标志
double switchTime = 0;                                            // 切换时间

nav_msgs::msg::Path path;                                         // 路径消息
rclcpp::Node::SharedPtr nh;                                       // ROS2节点句柄

unitree_api::msg::Request req;                                    // 宇树API请求消息
SportClient sport_req;                                            // 运动客户端对象

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)  // 里程计回调函数
{
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();            // 获取里程计时间戳
  double roll, pitch, yaw;                                            // 声明姿态角变量
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;  // 获取四元数
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);  // 四元数转欧拉角

  vehicleRoll = roll;                                                 // 更新车辆横滚角
  vehiclePitch = pitch;                                              // 更新车辆俯仰角
  vehicleYaw = yaw;                                                  // 更新车辆偏航角
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;  // 计算车辆X坐标
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;  // 计算车辆Y坐标
  vehicleZ = odomIn->pose.pose.position.z;                           // 更新车辆Z坐标

  //roll和pitch值来自里程计消息的四元数转换
  //inclThre(默认45度)与机器人当前的横滚角(roll)和俯仰角(pitch)进行比较
  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {  // 检查倾角是否超过阈值
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();     // 更新停止初始时间
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {  // 检查倾角速率是否超过阈值
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();     // 更新减速初始时间
  }
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
{
  int pathSize = pathIn->poses.size();                                      // 获取路径点数量
  path.poses.resize(pathSize);                                             // 调整路径消息大小
  for (int i = 0; i < pathSize; i++) {                                    // 遍历所有路径点
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;     // 复制X坐标
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;     // 复制Y坐标
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;     // 复制Z坐标
  }

  vehicleXRec = vehicleX;                                                 // 记录当前车辆X坐标
  vehicleYRec = vehicleY;                                                 // 记录当前车辆Y坐标
  vehicleZRec = vehicleZ;                                                 // 记录当前车辆Z坐标
  vehicleRollRec = vehicleRoll;                                          // 记录当前车辆横滚角
  vehiclePitchRec = vehiclePitch;                                        // 记录当前车辆俯仰角
  vehicleYawRec = vehicleYaw;                                            // 记录当前车辆偏航角

  pathPointID = 0;                                                       // 重置路径点索引
  pathInit = true;                                                       // 设置路径初始化标志
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joyTime = nh->now().seconds();                                        // 获取当前时间
  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);  // 计算手柄原始速度
  joySpeed = joySpeedRaw;                                              // 设置手柄速度
  if (joySpeed > 1.0) joySpeed = 1.0;                                  // 限制最大速度为1
  if (joy->axes[4] == 0) joySpeed = 0;                                 // 如果前进轴为0则速度为0
  joyYaw = joy->axes[3];                                               // 设置偏航控制量
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;                       // 停止时不允许旋转

  if (joy->axes[4] < 0 && !twoWayDrive) {                             // 如果不允许双向驾驶且前进轴为负
    joySpeed = 0;                                                      // 速度置零
    joyYaw = 0;                                                        // 偏航置零
  }

  joyManualFwd = joy->axes[4];                                         // 设置手动前进控制量
  joyManualLeft = joy->axes[3];                                        // 设置手动左转控制量
  joyManualYaw = joy->axes[0];                                         // 设置手动偏航控制量

  if (joy->axes[2] > -0.1) {                                          // 根据手柄轴2判断自动模式
    autonomyMode = false;                                              // 关闭自动模式
  } else {
    autonomyMode = true;                                               // 开启自动模式
  }

  if (joy->axes[5] > -0.1) {                                          // 根据手柄轴5判断手动模式
    manualMode = false;                                               // 关闭手动模式
  } else {
    manualMode = true;                                                // 开启手动模式
  }
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
{
  double speedTime = nh->now().seconds();                              // 获取当前时间
  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {  // 自动模式且手柄未操作
    joySpeed = speed->data / maxSpeed;                                 // 根据速度消息设置速度

    if (joySpeed < 0) joySpeed = 0;                                    // 限制最小速度为0
    else if (joySpeed > 1.0) joySpeed = 1.0;                          // 限制最大速度为1
  }
}

void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
{
  safetyStop = stop->data;                                            // 设置安全停止标志
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("pathFollower");

  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<int>("pubSkipNum", pubSkipNum);
  nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  nh->declare_parameter<double>("lookAheadDis", lookAheadDis);
  nh->declare_parameter<double>("yawRateGain", yawRateGain);
  nh->declare_parameter<double>("stopYawRateGain", stopYawRateGain);
  nh->declare_parameter<double>("maxYawRate", maxYawRate);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("maxAccel", maxAccel);
  nh->declare_parameter<double>("switchTimeThre", switchTimeThre);
  nh->declare_parameter<double>("dirDiffThre", dirDiffThre);
  nh->declare_parameter<double>("omniDirDiffThre", omniDirDiffThre);
  nh->declare_parameter<double>("noRotSpeed", noRotSpeed);
  nh->declare_parameter<double>("stopDisThre", stopDisThre);
  nh->declare_parameter<double>("slowDwnDisThre", slowDwnDisThre);
  nh->declare_parameter<bool>("useInclRateToSlow", useInclRateToSlow);
  nh->declare_parameter<double>("inclRateThre", inclRateThre);
  nh->declare_parameter<double>("slowRate1", slowRate1);
  nh->declare_parameter<double>("slowRate2", slowRate2);
  nh->declare_parameter<double>("slowTime1", slowTime1);
  nh->declare_parameter<double>("slowTime2", slowTime2);
  nh->declare_parameter<bool>("useInclToStop", useInclToStop);
  nh->declare_parameter<double>("inclThre", inclThre);
  nh->declare_parameter<double>("stopTime", stopTime);
  nh->declare_parameter<bool>("noRotAtStop", noRotAtStop);
  nh->declare_parameter<bool>("noRotAtGoal", noRotAtGoal);
  nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
  nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);
  nh->declare_parameter<double>("goalCloseDis", goalCloseDis);
  nh->declare_parameter<bool>("is_real_robot", is_real_robot);

  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("pubSkipNum", pubSkipNum);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("yawRateGain", yawRateGain);
  nh->get_parameter("stopYawRateGain", stopYawRateGain);
  nh->get_parameter("maxYawRate", maxYawRate);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("maxAccel", maxAccel);
  nh->get_parameter("switchTimeThre", switchTimeThre);
  nh->get_parameter("dirDiffThre", dirDiffThre);
  nh->get_parameter("omniDirDiffThre", omniDirDiffThre);
  nh->get_parameter("noRotSpeed", noRotSpeed);
  nh->get_parameter("stopDisThre", stopDisThre);
  nh->get_parameter("slowDwnDisThre", slowDwnDisThre);
  nh->get_parameter("useInclRateToSlow", useInclRateToSlow);
  nh->get_parameter("inclRateThre", inclRateThre);
  nh->get_parameter("slowRate1", slowRate1);
  nh->get_parameter("slowRate2", slowRate2);
  nh->get_parameter("slowTime1", slowTime1);
  nh->get_parameter("slowTime2", slowTime2);
  nh->get_parameter("useInclToStop", useInclToStop);
  nh->get_parameter("inclThre", inclThre);
  nh->get_parameter("stopTime", stopTime);
  nh->get_parameter("noRotAtStop", noRotAtStop);
  nh->get_parameter("noRotAtGoal", noRotAtGoal);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);
  nh->get_parameter("goalCloseDis", goalCloseDis);
  nh->get_parameter("is_real_robot", is_real_robot);

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);

  auto subPath = nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>("/speed", 5, speedHandler);

  auto subStop = nh->create_subscription<std_msgs::msg::Int8>("/stop", 5, stopHandler);

  auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);

  auto pubGo2Request = nh->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

  geometry_msgs::msg::TwistStamped cmd_vel;                                   // 创建速度消息对象
  cmd_vel.header.frame_id = "vehicle";                                        // 设置消息帧ID为vehicle

  if (autonomyMode) {                                                         // 如果是自主模式
    joySpeed = autonomySpeed / maxSpeed;                                      // 计算标准化的速度值

    if (joySpeed < 0) joySpeed = 0;                                          // 速度下限为0
    else if (joySpeed > 1.0) joySpeed = 1.0;                                 // 速度上限为1
  }

  rclcpp::Rate rate(100);                                                    // 设置循环频率为100Hz
  bool status = rclcpp::ok();                                                // 获取ROS系统状态
  while (status) {                                                           // 主循环
    rclcpp::spin_some(nh);                                                  // 处理回调

    if (pathInit) {                                                          // 如果路径已初始化
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);     // 计算相对X坐标
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);     // 计算相对Y坐标

      int pathSize = path.poses.size();                                      // 获取路径点数量
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;  // 计算终点X方向距离
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;  // 计算终点Y方向距离
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);           // 计算到终点的距离

      float disX, disY, dis;                                                 // 声明距离变量
      while (pathPointID < pathSize - 1) {                                   // 寻找前视路径点
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;        // 计算当前点X方向距离
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;        // 计算当前点Y方向距离
        dis = sqrt(disX * disX + disY * disY);                              // 计算到当前点的距离
        if (dis < lookAheadDis) {                                           // 如果距离小于前视距离
          pathPointID++;                                                     // 继续查找下一点
        } else {
          break;                                                            // 找到合适的前视点后退出
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;          // 计算目标点X方向距离
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;          // 计算目标点Y方向距离
      dis = sqrt(disX * disX + disY * disY);                                // 计算到目标点的距离
      float pathDir = atan2(disY, disX);                                    // 计算路径方向角

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;                 // 计算方向差
      if (dirDiff > PI) dirDiff -= 2 * PI;                                  // 角度归一化
      else if (dirDiff < -PI) dirDiff += 2 * PI;                            // 角度归一化
      if (dirDiff > PI) dirDiff -= 2 * PI;                                  // 二次角度归一化
      else if (dirDiff < -PI) dirDiff += 2 * PI;                            // 二次角度归一化

      if (twoWayDrive) {                                                     // 如果允许双向驾驶
        double time = nh->now().seconds();                                   // 获取当前时间
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {  // 判断是否需要切换到后退
          navFwd = false;                                                    // 切换为后退模式
          switchTime = time;                                                 // 记录切换时间
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {  // 判断是否需要切换到前进
          navFwd = true;                                                     // 切换为前进模式
          switchTime = time;                                                 // 记录切换时间
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;                                 // 计算实际速度值
      if (!navFwd) {                                                         // 如果是后退模式
        dirDiff += PI;                                                       // 调整方向差
        if (dirDiff > PI) dirDiff -= 2 * PI;                                // 角度归一化
        joySpeed2 *= -1;                                                     // 速度取反
      }

      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;  // 低速时使用停止偏航增益
      else vehicleYawRate = -yawRateGain * dirDiff;                         // 正常速度时使用正常偏航增益

      if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;  // 限制最大偏航速率
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;  // 限制最小偏航速率

      if (joySpeed2 == 0 && !autonomyMode) {                                // 如果速度为零且非自主模式
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;                  // 使用手柄控制偏航
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {     // 如果到达目标点
        vehicleYawRate = 0;                                                  // 停止旋转
      }

      if (pathSize <= 1) {                                                   // 如果路径点不足
        joySpeed2 = 0;                                                       // 停止移动
      } else if (endDis / slowDwnDisThre < joySpeed) {                      // 如果接近终点
        joySpeed2 *= endDis / slowDwnDisThre;                               // 降低速度
      }

      float joySpeed3 = joySpeed2;                                          // 复制速度值
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;  // 第一阶段减速
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) joySpeed3 *= slowRate2;  // 第二阶段减速

      if ((fabs(dirDiff) < dirDiffThre || (dis < goalCloseDis && fabs(dirDiff) < omniDirDiffThre))  && dis > stopDisThre) {  // 如果方向对准且未到达停止距离
        if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;     // 加速
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;  // 减速
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;             // 正向减速
        else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;        // 反向减速
      }

      if (fabs(vehicleSpeed) > noRotSpeed) vehicleYawRate = 0;              // 高速时禁止旋转

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {         // 如果在停止时间内
        vehicleSpeed = 0;                                                    // 速度置零
        vehicleYawRate = 0;                                                  // 偏航速率置零
      }

      if ((safetyStop & 1) > 0 && vehicleSpeed > 0) vehicleSpeed = 0;       // 前向安全停止
      if ((safetyStop & 2) > 0 && vehicleSpeed < 0) vehicleSpeed = 0;       // 后向安全停止
      if ((safetyStop & 4) > 0 && vehicleYawRate > 0) vehicleYawRate = 0;   // 顺时针安全停止
      if ((safetyStop & 8) > 0 && vehicleYawRate < 0) vehicleYawRate = 0;   // 逆时针安全停止

      pubSkipCount--;                                                        // 发布计数器递减
      if (pubSkipCount < 0) {                                               // 如果到达发布时间
        cmd_vel.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));  // 设置时间戳
        if (fabs(vehicleSpeed) <= maxAccel / 100.0) {                       // 如果速度接近零
          cmd_vel.twist.linear.x = 0;                                        // X方向速度置零
          cmd_vel.twist.linear.y = 0;                                        // Y方向速度置零
        } else {
          cmd_vel.twist.linear.x = cos(dirDiff) * vehicleSpeed;             // 计算X方向速度
          cmd_vel.twist.linear.y = -sin(dirDiff) * vehicleSpeed;            // 计算Y方向速度
        }
        cmd_vel.twist.angular.z = vehicleYawRate;                           // 设置角速度
        
        if (manualMode) {                                                    // 如果是手动模式
          cmd_vel.twist.linear.x = maxSpeed * joyManualFwd;                 // 使用手柄前进速度
          cmd_vel.twist.linear.y = maxSpeed / 2.0 * joyManualLeft;          // 使用手柄横向速度
          cmd_vel.twist.angular.z = maxYawRate * PI / 180.0 * joyManualYaw;  // 使用手柄旋转速度
        }

        pubSpeed->publish(cmd_vel);                                          // 发布速度命令

        pubSkipCount = pubSkipNum;                                          // 重置发布计数器

        if (is_real_robot)                                                   // 如果是真实机器人
        {
          if (cmd_vel.twist.linear.x == 0 && cmd_vel.twist.linear.y == 0 && cmd_vel.twist.angular.z == 0){  // 如果所有速度为零
          	sport_req.StopMove(req);                                         // 发送停止命令
          }
          else{
               sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);  // 发送移动命令
          }
          pubGo2Request->publish(req);                                       // 发布请求消息
        }
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
