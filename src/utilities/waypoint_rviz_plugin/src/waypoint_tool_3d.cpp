#include <waypoint_tool_3d.hpp>

#include <string>

#include <rviz_common/display_context.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>

namespace waypoint_rviz_plugin
{
WaypointTool3D::WaypointTool3D()
: dddmr_rviz_default_plugins::tools::PoseTool(), qos_profile_(5)
{
  shortcut_key_ = 'w';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "waypoint_3d", 
    "The topic on which to publish 3D navigation waypoints.",
    getPropertyContainer(), SLOT(updateTopic()), this);
  
  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

WaypointTool3D::~WaypointTool3D() = default;

void WaypointTool3D::onInitialize()
{
  dddmr_rviz_default_plugins::tools::PoseTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  setName("3D Waypoint");
  updateTopic();
}

void WaypointTool3D::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  
  pub_ = raw_node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/way_point", qos_profile_);
  pub_joy_ = raw_node->create_publisher<sensor_msgs::msg::Joy>(
    "/joy", qos_profile_);
  clock_ = raw_node->get_clock();
}

void WaypointTool3D::onPoseSet(double x, double y, double z, double /*theta*/)
{
  sensor_msgs::msg::Joy joy;

  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

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

  joy.header.stamp = clock_->now();
  joy.header.frame_id = "waypoint_tool";
  pub_joy_->publish(joy);

  geometry_msgs::msg::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = joy.header.stamp;
  waypoint.point.x = x;
  waypoint.point.y = y;
  waypoint.point.z = z;

  pub_->publish(waypoint);
  usleep(10000);
  pub_->publish(waypoint);
}
}

#include <pluginlib/class_list_macros.hpp> 
PLUGINLIB_EXPORT_CLASS(waypoint_rviz_plugin::WaypointTool3D, rviz_common::Tool)
