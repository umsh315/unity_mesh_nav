/**
 * @file planning_env.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the world representation using point clouds
 * @version 0.1
 * @date 2020-06-04
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <cmath>
#include <vector>
#include <memory>
#include <Eigen/Core>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
// PCL
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>

// Third parties
#include <utils/pointcloud_utils.h>
// Components
#include <pointcloud_manager/pointcloud_manager.h>
#include <lidar_model/lidar_model.h>
#include "rolling_occupancy_grid/rolling_occupancy_grid.h"

namespace viewpoint_manager_ns
{
class ViewPointManager;
}

namespace planning_env_ns
{
typedef pcl::PointXYZRGBNormal PlannerCloudPointType;
typedef pcl::PointCloud<PlannerCloudPointType> PlannerCloudType;
struct PlanningEnvParameters;
class PlanningEnv;
}  // namespace planning_env_ns

struct planning_env_ns::PlanningEnvParameters
{
  // Collision check
  double kSurfaceCloudDwzLeafSize;
  double kCollisionCloudDwzLeafSize;
  double kKeyposeGraphCollisionCheckRadius;
  int kKeyposeGraphCollisionCheckPointNumThr;

  int kKeyposeCloudStackNum;

  int kPointCloudRowNum;
  int kPointCloudColNum;
  int kPointCloudLevelNum;
  int kMaxCellPointNum;
  double kPointCloudCellSize;
  double kPointCloudCellHeight;
  int kPointCloudManagerNeighborCellNum;
  double kCoverCloudZSqueezeRatio;

  // Occupancy Grid
  bool kUseFrontier;
  double kFrontierClusterTolerance;
  int kFrontierClusterMinSize;
  Eigen::Vector3d kExtractFrontierRange;

  // Boundary
  bool kUseCoverageBoundaryOnFrontier;
  bool kUseCoverageBoundaryOnObjectSurface;

  void ReadParameters(rclcpp::Node::SharedPtr nh);
};

class planning_env_ns::PlanningEnv
{
public:
  PlanningEnv(rclcpp::Node::SharedPtr nh, std::string world_frame_id = "map");
  ~PlanningEnv() = default;
  double GetPlannerCloudResolution()
  {
    return parameters_.kSurfaceCloudDwzLeafSize;
  }
  void SetUseFrontier(bool use_frontier)
  {
    parameters_.kUseFrontier = use_frontier;
  }
  void UpdateRobotPosition(geometry_msgs::msg::Point robot_position)
  {
    bool pointcloud_manager_rolling = pointcloud_manager_->UpdateRobotPosition(robot_position);
    Eigen::Vector3d pointcloud_manager_neighbor_cells_origin = pointcloud_manager_->GetNeighborCellsOrigin();
    rolling_occupancy_grid_->InitializeOrigin(pointcloud_manager_neighbor_cells_origin);
    bool occupancy_grid_rolling = rolling_occupancy_grid_->UpdateRobotPosition(
        Eigen::Vector3d(robot_position.x, robot_position.y, robot_position.z));
    if (pointcloud_manager_rolling)
    {
      // Update rolling occupancy grid
      rolled_in_occupancy_cloud_->cloud_ = pointcloud_manager_->GetRolledInOccupancyCloud();
      pointcloud_manager_->ClearNeighborCellOccupancyCloud();
      // rolled_in_occupancy_cloud_->Publish();
      rolling_occupancy_grid_->UpdateOccupancyStatus(rolled_in_occupancy_cloud_->cloud_);
    }
    if (occupancy_grid_rolling)
    {
      // Store and retrieve occupancy cloud
      rolled_out_occupancy_cloud_->cloud_ = rolling_occupancy_grid_->GetRolledOutOccupancyCloud();
      // rolled_out_occupancy_cloud_->Publish();
      pointcloud_manager_->StoreOccupancyCloud(rolled_out_occupancy_cloud_->cloud_);

      pointcloud_manager_->GetOccupancyCloud(pointcloud_manager_occupancy_cloud_->cloud_);
      // pointcloud_manager_occupancy_cloud_->Publish();
    }

    robot_position_.x() = robot_position.x;
    robot_position_.y() = robot_position.y;
    robot_position_.z() = robot_position.z;
    if (!robot_position_update_)
    {
      prev_robot_position_ = robot_position_;
    }
    robot_position_update_ = true;
  }
  template <class PCLPointType>
  void UpdateRegisteredCloud(typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
  {
    if (cloud->points.empty())
    {
      RCLCPP_WARN(rclcpp::get_logger("standalone_logger"),
                  "PlanningEnv::UpdateRegisteredCloud(): registered cloud empty");
      return;
    }
    else
    {
      if (parameters_.kUseFrontier)
      {
        rolling_occupancy_grid_->UpdateOccupancy<PCLPointType>(cloud);
        rolling_occupancy_grid_->RayTrace(robot_position_);
        rolling_occupancy_grid_->GetVisualizationCloud(rolling_occupancy_grid_cloud_->cloud_);
        // rolling_occupancy_grid_cloud_->Publish();
      }
    }
  }

  template <class PCLPointType>
  // 更新关键姿态点云
  void UpdateKeyposeCloud(typename pcl::PointCloud<PCLPointType>::Ptr& keypose_cloud)
  {
    // 检查关键姿态点云是否为空
    if (keypose_cloud->points.empty())
    {
      // 记录警告信息：关键姿态点云为空
      RCLCPP_WARN(rclcpp::get_logger("standalone_logger"), "PlanningEnv::UpdateKeyposeCloud(): keypose cloud empty");
      return; // 退出函数
    }
    else
    {
      // 复制关键姿态点云到内部云
      pcl::copyPointCloud<PCLPointType, PlannerCloudPointType>(*keypose_cloud, *(keypose_cloud_->cloud_));

      // 如果使用覆盖边界在物体表面
      if (parameters_.kUseCoverageBoundaryOnObjectSurface)
      {
        // 获取边界内的覆盖云
        GetCoverageCloudWithinBoundary<PlannerCloudPointType>(keypose_cloud_->cloud_);
      }

      // 提取感兴趣的表面
      misc_utils_ns::Timer get_surface_timer("get coverage and diff cloud");
      get_surface_timer.Start(); // 启动计时器
      vertical_surface_cloud_->cloud_->clear(); // 清空垂直表面云

      // 提取垂直表面
      vertical_surface_extractor_.ExtractVerticalSurface<PlannerCloudPointType, PlannerCloudPointType>(
          keypose_cloud_->cloud_, vertical_surface_cloud_->cloud_);
      // vertical_surface_cloud_->Publish(); // 发布垂直表面云

      // 更新旧的点云
      pointcloud_manager_->UpdateOldCloudPoints();
      // 更新点云
      pointcloud_manager_->UpdatePointCloud<PlannerCloudPointType>(*(vertical_surface_cloud_->cloud_));
      // 更新覆盖的点云
      pointcloud_manager_->UpdateCoveredCloudPoints();

      // 清空规划云
      planner_cloud_->cloud_->clear();
      // 获取点云
      pointcloud_manager_->GetPointCloud(*(planner_cloud_->cloud_));
      planner_cloud_->Publish(); // 发布规划云

      // 获取差异云
      diff_cloud_->cloud_->clear(); // 清空差异云
      // 将关键姿态点云的颜色设置为黑色
      for (auto& point : keypose_cloud_->cloud_->points)
      {
        point.r = 0;
        point.g = 0;
        point.b = 0;
      }
      // 将堆叠云的颜色设置为白色
      for (auto& point : stacked_cloud_->cloud_->points)
      {
        point.r = 255;
      }
      // 将关键姿态点云添加到堆叠云中
      *(stacked_cloud_->cloud_) += *(keypose_cloud_->cloud_);
      // 对堆叠云进行下采样
      stacked_cloud_downsizer_.Downsize(stacked_cloud_->cloud_, parameters_.kSurfaceCloudDwzLeafSize,
                                        parameters_.kSurfaceCloudDwzLeafSize, parameters_.kSurfaceCloudDwzLeafSize);
      // 检查堆叠云中的点
      for (const auto& point : stacked_cloud_->cloud_->points)
      {
        // 如果点的红色分量小于40
        if (point.r < 40)  // TODO: 根据关键姿态云分辨率和堆叠云分辨率计算
        {
          // 将点添加到差异云中
          diff_cloud_->cloud_->points.push_back(point);
        }
      }
      diff_cloud_->Publish(); // 发布差异云
      get_surface_timer.Stop(false); // 停止计时器

      // 堆叠在一起
      keypose_cloud_stack_[keypose_cloud_count_]->clear(); // 清空当前关键姿态云堆栈
      *keypose_cloud_stack_[keypose_cloud_count_] = *keypose_cloud_->cloud_; // 将当前关键姿态云添加到堆栈
      keypose_cloud_count_ = (keypose_cloud_count_ + 1) % parameters_.kKeyposeCloudStackNum; // 更新计数
      stacked_cloud_->cloud_->clear(); // 清空堆叠云
      // 将所有关键姿态云堆叠在一起
      for (int i = 0; i < parameters_.kKeyposeCloudStackNum; i++)
      {
        *(stacked_cloud_->cloud_) += *keypose_cloud_stack_[i];
      }
      // 对堆叠云进行下采样
      stacked_cloud_downsizer_.Downsize(stacked_cloud_->cloud_, parameters_.kSurfaceCloudDwzLeafSize,
                                        parameters_.kSurfaceCloudDwzLeafSize, parameters_.kSurfaceCloudDwzLeafSize);

      // 清空垂直表面云堆栈
      vertical_surface_cloud_stack_[keypose_cloud_count_]->clear();
      *vertical_surface_cloud_stack_[keypose_cloud_count_] = *(vertical_surface_cloud_->cloud_); // 将当前垂直表面云添加到堆栈
      keypose_cloud_count_ = (keypose_cloud_count_ + 1) % parameters_.kKeyposeCloudStackNum; // 更新计数
      stacked_vertical_surface_cloud_->cloud_->clear(); // 清空堆叠垂直表面云
      // 将所有垂直表面云堆叠在一起
      for (int i = 0; i < parameters_.kKeyposeCloudStackNum; i++)
      {
        *(stacked_vertical_surface_cloud_->cloud_) += *vertical_surface_cloud_stack_[i];
      }

      // 对堆叠垂直表面云进行下采样
      stacked_cloud_downsizer_.Downsize(stacked_vertical_surface_cloud_->cloud_, parameters_.kSurfaceCloudDwzLeafSize,
                                        parameters_.kSurfaceCloudDwzLeafSize, parameters_.kSurfaceCloudDwzLeafSize);

      // 如果堆叠垂直表面云不为空
      if (!stacked_vertical_surface_cloud_->cloud_->points.empty())
      {
        // 设置KD树输入云
        stacked_vertical_surface_cloud_kdtree_->setInputCloud(stacked_vertical_surface_cloud_->cloud_);
      }

      UpdateCollisionCloud(); // 更新碰撞云

      UpdateFrontiers(); // 更新前沿
    }
  }


  
  inline void UpdateCoverageBoundary(const geometry_msgs::msg::Polygon& polygon)
  {
    coverage_boundary_ = polygon;
  }

  template <class PCLPointType>
  void GetCoverageCloudWithinBoundary(typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
  {
    if (cloud->points.empty())
    {
      return;
    }
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (int i = 0; i < cloud->points.size(); i++)
    {
      geometry_msgs::msg::Point geo_point;
      geo_point.x = cloud->points[i].x;
      geo_point.y = cloud->points[i].y;
      geo_point.z = cloud->points[i].z;
      if (misc_utils_ns::PointInPolygon(geo_point, coverage_boundary_))
      {
        inliers->indices.push_back(i);
      }
    }
    pcl::ExtractIndices<PCLPointType> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetCollisionCloud()
  {
    return collision_cloud_;
  }
  pcl::PointCloud<PlannerCloudPointType>::Ptr GetStackedCloud()
  {
    return stacked_cloud_->cloud_;
  }

  void UpdateTerrainCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void UpdateCollisionCostGrid();
  bool InCollision(double x, double y, double z) const;

  inline pcl::PointCloud<PlannerCloudPointType>::Ptr GetDiffCloud()
  {
    return diff_cloud_->cloud_;
  }
  inline pcl::PointCloud<PlannerCloudPointType>::Ptr GetPlannerCloud()
  {
    return planner_cloud_->cloud_;
  }
  void UpdateCoveredArea(const lidar_model_ns::LiDARModel& robot_viewpoint,
                         const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);

  void GetUncoveredArea(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                        int& uncovered_point_num, int& uncovered_frontier_point_num);

  Eigen::Vector3d GetPointCloudManagerNeighborCellsOrigin()
  {
    return pointcloud_manager_->GetNeighborCellsOrigin();
  }
  void GetVisualizationPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud);
  void PublishStackedCloud();
  void PublishUncoveredCloud();
  void PublishUncoveredFrontierCloud();

private:
  PlanningEnvParameters parameters_;

  std::vector<typename PlannerCloudType::Ptr> keypose_cloud_stack_;
  std::vector<typename PlannerCloudType::Ptr> vertical_surface_cloud_stack_;

  int keypose_cloud_count_;
  Eigen::Vector3d robot_position_;
  Eigen::Vector3d prev_robot_position_;
  bool robot_position_update_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> keypose_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> stacked_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> stacked_vertical_surface_cloud_;
  pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr stacked_vertical_surface_cloud_kdtree_;
  pointcloud_utils_ns::PointCloudDownsizer<PlannerCloudPointType> stacked_cloud_downsizer_;
  pointcloud_utils_ns::PointCloudDownsizer<pcl::PointXYZI> collision_cloud_downsizer_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> vertical_surface_cloud_;
  pointcloud_utils_ns::VerticalSurfaceExtractor vertical_surface_extractor_;
  pointcloud_utils_ns::VerticalSurfaceExtractor vertical_frontier_extractor_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr collision_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> diff_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> terrain_cloud_;

  geometry_msgs::msg::Polygon coverage_boundary_;

  std::shared_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> planner_cloud_;
  std::shared_ptr<pointcloud_manager_ns::PointCloudManager> pointcloud_manager_;
  std::shared_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> rolling_occupancy_grid_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> rolling_occupancy_grid_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> rolling_frontier_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> rolling_filtered_frontier_cloud_;

  // For debug
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> rolled_in_occupancy_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> rolled_out_occupancy_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> pointcloud_manager_occupancy_cloud_;

  std::shared_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> squeezed_planner_cloud_;
  pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr squeezed_planner_cloud_kdtree_;

  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> uncovered_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> uncovered_frontier_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> frontier_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> filtered_frontier_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> occupied_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> free_cloud_;
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> unknown_cloud_;

  // std::shared_ptr<occupancy_grid_ns::OccupancyGrid> occupancy_grid_;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree_frontier_cloud_;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree_rolling_frontier_cloud_;

  void UpdateCollisionCloud();
  void UpdateFrontiers();
};
