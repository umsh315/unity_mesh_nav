#include <waypoint_tool_3d.hpp>

#include <string>
#include <nav_msgs/msg/odometry.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>

namespace waypoint_rviz_plugin
{
// 构造函数实现
WaypointTool3D::WaypointTool3D()
: dddmr_rviz_default_plugins::tools::PoseTool(), qos_profile_(5)
{
  // 设置快捷键为'w'
  shortcut_key_ = 'q';

  // 创建话题属性,用于设置发布3D导航路径点的ROS话题
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "waypoint_3d", 
    "The topic on which to publish 3D navigation waypoints.",
    getPropertyContainer(), SLOT(updateTopic()), this);
  
  // 创建QoS配置文件属性
  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

// 析构函数实现
WaypointTool3D::~WaypointTool3D() = default;

// 初始化函数
void WaypointTool3D::onInitialize()
{
  // 调用父类初始化
  dddmr_rviz_default_plugins::tools::PoseTool::onInitialize();
  // 初始化QoS配置
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  // 设置工具名称为"3D Waypoint"
  setName("3D Waypoint");
  // 更新话题
  updateTopic();
  vehicle_z = 0;  // 初始化机器人高度
}

// 更新话题函数
void WaypointTool3D::updateTopic()
{
  // 获取ROS节点
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  
  // 订阅机器人状态
  sub_ = raw_node->create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5,
    std::bind(&WaypointTool3D::odomHandler, this, std::placeholders::_1));
  
  // 创建路径点发布者
  pub_ = raw_node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/way_point", qos_profile_);
  // 创建Joy消息发布者
  pub_joy_ = raw_node->create_publisher<sensor_msgs::msg::Joy>(
    "/joy", qos_profile_);
  // 获取时钟
  clock_ = raw_node->get_clock();
}

void WaypointTool3D::odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

// 设置位姿的回调函数
void WaypointTool3D::onPoseSet(double x, double y, double z, double /*theta*/)
{
  // 添加彩色日志输出
  RCLCPP_INFO_STREAM(
    context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger(),
    "\033[1;34m[WaypointTool-3D]\033[0m 设置3D导航点 - 坐标: (" << x << ", " << y << ", " << z << "), 机器人当前高度: " << vehicle_z);

  // 创建Joy消息
  sensor_msgs::msg::Joy joy;

  // 设置Joy消息的axes数组
  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  // 设置Joy消息的buttons数组
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  // 设置Joy消息的时间戳和frame_id
  joy.header.stamp = clock_->now();
  joy.header.frame_id = "waypoint_tool";
  // 发布Joy消息
  pub_joy_->publish(joy);

  // 创建路径点消息
  geometry_msgs::msg::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = joy.header.stamp;
  waypoint.point.x = x;
  waypoint.point.y = y;
  waypoint.point.z = vehicle_z;  // 使用机器人当前高度

  // 发布路径点消息两次,确保消息被接收
  pub_->publish(waypoint);
  usleep(10000);
  pub_->publish(waypoint);
}
}

// 导出插件类
#include <pluginlib/class_list_macros.hpp> 
PLUGINLIB_EXPORT_CLASS(waypoint_rviz_plugin::WaypointTool3D, rviz_common::Tool)
