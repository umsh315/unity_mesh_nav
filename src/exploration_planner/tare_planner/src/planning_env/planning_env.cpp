/**
 * @file planning_env.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the world representation using point clouds
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <planning_env/planning_env.h>
#include <viewpoint_manager/viewpoint_manager.h>

namespace planning_env_ns
{
void PlanningEnvParameters::ReadParameters(rclcpp::Node::SharedPtr nh)
{
  nh->get_parameter("kSurfaceCloudDwzLeafSize", kSurfaceCloudDwzLeafSize);
  nh->get_parameter("kCollisionCloudDwzLeafSize", kCollisionCloudDwzLeafSize);
  nh->get_parameter("keypose_graph/kAddEdgeCollisionCheckRadius", kKeyposeGraphCollisionCheckRadius);
  nh->get_parameter("keypose_graph/kAddEdgeCollisionCheckPointNumThr", kKeyposeGraphCollisionCheckPointNumThr);
  nh->get_parameter("kKeyposeCloudStackNum", kKeyposeCloudStackNum);
  nh->get_parameter("kPointCloudRowNum", kPointCloudRowNum);
  nh->get_parameter("kPointCloudColNum", kPointCloudColNum);
  nh->get_parameter("kPointCloudLevelNum", kPointCloudLevelNum);
  nh->get_parameter("kMaxCellPointNum", kMaxCellPointNum);
  nh->get_parameter("kPointCloudCellSize", kPointCloudCellSize);
  nh->get_parameter("kPointCloudCellHeight", kPointCloudCellHeight);
  nh->get_parameter("kPointCloudManagerNeighborCellNum", kPointCloudManagerNeighborCellNum);
  nh->get_parameter("kCoverCloudZSqueezeRatio", kCoverCloudZSqueezeRatio);
  nh->get_parameter("kUseFrontier", kUseFrontier);
  nh->get_parameter("kFrontierClusterTolerance", kFrontierClusterTolerance);
  nh->get_parameter("kFrontierClusterMinSize", kFrontierClusterMinSize);
  nh->get_parameter("kUseCoverageBoundaryOnFrontier", kUseCoverageBoundaryOnFrontier);
  nh->get_parameter("kUseCoverageBoundaryOnObjectSurface", kUseCoverageBoundaryOnObjectSurface);
  int viewpoint_number = nh->get_parameter("viewpoint_manager/number_x").as_int();
  double viewpoint_resolution = nh->get_parameter("viewpoint_manager/resolution_x").as_double();
  double sensor_range = nh->get_parameter("kSensorRange").as_double();

  double local_planning_horizon_half_size = viewpoint_number * viewpoint_resolution / 2;
  kExtractFrontierRange.x() = local_planning_horizon_half_size + sensor_range * 2;
  kExtractFrontierRange.y() = local_planning_horizon_half_size + sensor_range * 2;
  kExtractFrontierRange.z() = 2;
}

PlanningEnv::PlanningEnv(rclcpp::Node::SharedPtr nh, std::string world_frame_id)
  : keypose_cloud_count_(0)
  , vertical_surface_extractor_()
  , vertical_frontier_extractor_()
  , robot_position_update_(false)
{
  parameters_.ReadParameters(nh);

  keypose_cloud_stack_.resize(parameters_.kKeyposeCloudStackNum);
  for (int i = 0; i < keypose_cloud_stack_.size(); i++)
  {
    keypose_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
  }

  vertical_surface_cloud_stack_.resize(parameters_.kKeyposeCloudStackNum);
  for (int i = 0; i < vertical_surface_cloud_stack_.size(); i++)
  {
    vertical_surface_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
  }
  keypose_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "keypose_cloud", world_frame_id);
  stacked_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "stacked_cloud", world_frame_id);
  stacked_vertical_surface_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
      nh, "stacked_vertical_surface_cloud", world_frame_id);

  stacked_vertical_surface_cloud_kdtree_ =
      pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>());
  vertical_surface_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "coverage_cloud", world_frame_id);

  diff_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "diff_cloud", world_frame_id);

  collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  terrain_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_cloud", world_frame_id);

  // 创建一个共享指针，指向一个新的点云对象，用于存储规划点云
  planner_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "planner_cloud", world_frame_id);

  pointcloud_manager_ = std::make_shared<pointcloud_manager_ns::PointCloudManager>(
      parameters_.kPointCloudRowNum, parameters_.kPointCloudColNum, parameters_.kPointCloudLevelNum,
      parameters_.kMaxCellPointNum, parameters_.kPointCloudCellSize, parameters_.kPointCloudCellHeight,
      parameters_.kPointCloudManagerNeighborCellNum);
  pointcloud_manager_->SetCloudDwzFilterLeafSize() = parameters_.kSurfaceCloudDwzLeafSize;

  rolling_occupancy_grid_ = std::make_shared<rolling_occupancy_grid_ns::RollingOccupancyGrid>(nh);

  squeezed_planner_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
      nh, "squeezed_planner_cloud", world_frame_id);

  squeezed_planner_cloud_kdtree_ =
      pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>());

  uncovered_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_cloud", world_frame_id);
  uncovered_frontier_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_frontier_cloud", world_frame_id);
  frontier_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "frontier_cloud", world_frame_id);
  filtered_frontier_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "filtered_frontier_cloud", world_frame_id);
  occupied_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "occupied_cloud", world_frame_id);
  free_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "free_cloud", world_frame_id);
  unknown_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "unknown_cloud", world_frame_id);

  rolling_occupancy_grid_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "rolling_occupancy_grid_cloud", world_frame_id);

  rolling_frontier_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolling_frontier_cloud", world_frame_id);

  rolling_filtered_frontier_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "rolling_filtered_frontier_cloud", world_frame_id);

  rolled_in_occupancy_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolled_in_occupancy_cloud", world_frame_id);
  rolled_out_occupancy_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolled_out_occupancy_cloud", world_frame_id);

  pointcloud_manager_occupancy_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "pointcloud_manager_occupancy_cloud_", world_frame_id);

  kdtree_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);
  kdtree_rolling_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);

  // Todo: parameterize
  vertical_surface_extractor_.SetRadiusThreshold(0.2);
  vertical_surface_extractor_.SetZDiffMax(2.0);
  vertical_surface_extractor_.SetZDiffMin(parameters_.kSurfaceCloudDwzLeafSize);
  vertical_frontier_extractor_.SetNeighborThreshold(2);

  Eigen::Vector3d rolling_occupancy_grid_resolution = rolling_occupancy_grid_->GetResolution();
  double vertical_frontier_neighbor_search_radius =
      std::max(rolling_occupancy_grid_resolution.x(), rolling_occupancy_grid_resolution.y());
  vertical_frontier_neighbor_search_radius =
      std::max(vertical_frontier_neighbor_search_radius, rolling_occupancy_grid_resolution.z());
  vertical_frontier_extractor_.SetRadiusThreshold(vertical_frontier_neighbor_search_radius);
  double z_diff_max = vertical_frontier_neighbor_search_radius * 5;
  double z_diff_min = vertical_frontier_neighbor_search_radius;
  vertical_frontier_extractor_.SetZDiffMax(z_diff_max);
  vertical_frontier_extractor_.SetZDiffMin(z_diff_min);
  vertical_frontier_extractor_.SetNeighborThreshold(2);
}

void PlanningEnv::UpdateCollisionCloud()
{
  collision_cloud_->clear();
  for (int i = 0; i < parameters_.kKeyposeCloudStackNum; i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud<PlannerCloudPointType, pcl::PointXYZI>(*vertical_surface_cloud_stack_[i], *cloud_tmp);
    *(collision_cloud_) += *cloud_tmp;
  }
  collision_cloud_downsizer_.Downsize(collision_cloud_, parameters_.kCollisionCloudDwzLeafSize,
                                      parameters_.kCollisionCloudDwzLeafSize, parameters_.kCollisionCloudDwzLeafSize);
}

void PlanningEnv::UpdateFrontiers()
{
  if (parameters_.kUseFrontier)
  {
    prev_robot_position_ = robot_position_;
    rolling_occupancy_grid_->GetFrontier(frontier_cloud_->cloud_, robot_position_, parameters_.kExtractFrontierRange);

    if (!frontier_cloud_->cloud_->points.empty())
    {
      if (parameters_.kUseCoverageBoundaryOnFrontier)
      {
        GetCoverageCloudWithinBoundary<pcl::PointXYZI>(frontier_cloud_->cloud_);
      }
      vertical_frontier_extractor_.ExtractVerticalSurface<pcl::PointXYZI, pcl::PointXYZI>(
          frontier_cloud_->cloud_, filtered_frontier_cloud_->cloud_);
    }

    // Cluster frontiers
    if (!filtered_frontier_cloud_->cloud_->points.empty())
    {
      kdtree_frontier_cloud_->setInputCloud(filtered_frontier_cloud_->cloud_);
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(parameters_.kFrontierClusterTolerance);
      ec.setMinClusterSize(1);
      ec.setMaxClusterSize(10000);
      ec.setSearchMethod(kdtree_frontier_cloud_);
      ec.setInputCloud(filtered_frontier_cloud_->cloud_);
      ec.extract(cluster_indices);

      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      int cluster_count = 0;
      for (int i = 0; i < cluster_indices.size(); i++)
      {
        if (cluster_indices[i].indices.size() < parameters_.kFrontierClusterMinSize)
        {
          continue;
        }
        for (int j = 0; j < cluster_indices[i].indices.size(); j++)
        {
          int point_ind = cluster_indices[i].indices[j];
          filtered_frontier_cloud_->cloud_->points[point_ind].intensity = cluster_count;
          inliers->indices.push_back(point_ind);
        }
        cluster_count++;
      }
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud(filtered_frontier_cloud_->cloud_);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*(filtered_frontier_cloud_->cloud_));
      filtered_frontier_cloud_->Publish();
    }
  }
}

void PlanningEnv::UpdateTerrainCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  if (cloud->points.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("standalone_logger"), "Terrain cloud empty");
  }
  else
  {
    terrain_cloud_->cloud_ = cloud;
  }
}

bool PlanningEnv::InCollision(double x, double y, double z) const
{
  if (stacked_cloud_->cloud_->points.empty())
  {
    RCLCPP_WARN(rclcpp::get_logger("standalone_logger"),
                "PlanningEnv::InCollision(): collision cloud empty, not checking collision");
    return false;
  }
  PlannerCloudPointType check_point;
  check_point.x = x;
  check_point.y = y;
  check_point.z = z;
  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_sqdist;
  stacked_vertical_surface_cloud_kdtree_->radiusSearch(check_point, parameters_.kKeyposeGraphCollisionCheckRadius,
                                                       neighbor_indices, neighbor_sqdist);
  if (neighbor_indices.size() > parameters_.kKeyposeGraphCollisionCheckPointNumThr)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void PlanningEnv::UpdateCoveredArea(const lidar_model_ns::LiDARModel& robot_viewpoint,
                                    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  if (planner_cloud_->cloud_->points.empty())
  {
    std::cout << "Planning cloud empty, cannot update covered area" << std::endl;
    return;
  }

  geometry_msgs::msg::Point robot_position = robot_viewpoint.getPosition();
  double sensor_range = viewpoint_manager->GetSensorRange();
  double coverage_occlusion_thr = viewpoint_manager->GetCoverageOcclusionThr();
  double coverage_dilation_radius = viewpoint_manager->GetCoverageDilationRadius();
  std::vector<int> covered_point_indices;
  double vertical_fov_ratio = 0.3;  // bigger fov than viewpoints
  double diff_z_max = sensor_range * vertical_fov_ratio;
  double xy_dist_threshold = 3 * (parameters_.kSurfaceCloudDwzLeafSize / 2) / 0.3;
  double z_diff_threshold = 3 * parameters_.kSurfaceCloudDwzLeafSize;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      planner_cloud_->cloud_->points[i].g = 255;
      continue;
    }
    if (std::abs(point.z - robot_position.z) < diff_z_max)
    {
      if (misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                     Eigen::Vector3d(robot_position.x, robot_position.y, robot_position.z),
                                     vertical_fov_ratio, sensor_range, xy_dist_threshold, z_diff_threshold))
      {
        if (robot_viewpoint.CheckVisibility<PlannerCloudPointType>(point, coverage_occlusion_thr))
        {
          planner_cloud_->cloud_->points[i].g = 255;
          covered_point_indices.push_back(i);
          continue;
        }
      }
    }
    // mark covered by visited viewpoints
    for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
    {
      if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
        {
          planner_cloud_->cloud_->points[i].g = 255;
          covered_point_indices.push_back(i);
          break;
        }
      }
    }
  }

  // Dilate the covered area
  squeezed_planner_cloud_->cloud_->clear();
  for (const auto& point : planner_cloud_->cloud_->points)
  {
    PlannerCloudPointType squeezed_point = point;
    squeezed_point.z = point.z / parameters_.kCoverCloudZSqueezeRatio;
    squeezed_planner_cloud_->cloud_->points.push_back(squeezed_point);
  }
  squeezed_planner_cloud_kdtree_->setInputCloud(squeezed_planner_cloud_->cloud_);

  for (const auto& ind : covered_point_indices)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[ind];
    std::vector<int> nearby_indices;
    std::vector<float> nearby_sqdist;
    squeezed_planner_cloud_kdtree_->radiusSearch(point, coverage_dilation_radius, nearby_indices, nearby_sqdist);
    if (!nearby_indices.empty())
    {
      for (const auto& idx : nearby_indices)
      {
        MY_ASSERT(idx >= 0 && idx < planner_cloud_->cloud_->points.size());
        planner_cloud_->cloud_->points[idx].g = 255;
      }
    }
  }

  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      int cloud_idx = 0;
      int cloud_point_idx = 0;
      pointcloud_manager_->GetCloudPointIndex(i, cloud_idx, cloud_point_idx);
      pointcloud_manager_->UpdateCoveredCloudPoints(cloud_idx, cloud_point_idx);
    }
  }
}

void PlanningEnv::GetUncoveredArea(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                   int& uncovered_point_num, int& uncovered_frontier_point_num)
{
  // Clear viewpoint covered point list
  for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
  {
    viewpoint_manager->ResetViewPointCoveredPointList(viewpoint_ind);
  }


  uncovered_cloud_->cloud_->clear(); // 清空未覆盖点云
  uncovered_frontier_cloud_->cloud_->clear(); // 清空未覆盖前沿点云
  uncovered_point_num = 0; // 初始化未覆盖点数量为0
  uncovered_frontier_point_num = 0; // 初始化未覆盖前沿点数量为0

  // 检查未覆盖的点
  int g_greater_than_zero_count = 0; // 记录g值大于0的点的数量

  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++) // 遍历规划点云中的每个点
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i]; // 获取当前点
    
    if (point.g > 0) // 如果点的g值大于0
    {
      g_greater_than_zero_count++; // 计数加1
      continue; // 跳过该点
    }
    bool observed = false; // 标记当前点是否被观察到
    for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_) // 遍历视点管理器中的候选视点索引
    {
      if (!viewpoint_manager->ViewPointVisited(viewpoint_ind)) // 检查视点是否未被访问
      {
        if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind)) // 检查当前点是否被该视点可见
        {
          viewpoint_manager->AddUncoveredPoint(viewpoint_ind, uncovered_point_num); // 将未覆盖的点添加到视点管理器中
          observed = true; // 标记为已观察
        }
      }
    }
    if (observed) // 如果当前点被观察到
    {
      pcl::PointXYZI uncovered_point; // 创建未覆盖点
      uncovered_point.x = point.x; // 设置x坐标
      uncovered_point.y = point.y; // 设置y坐标
      uncovered_point.z = point.z; // 设置z坐标
      uncovered_point.intensity = i; // 设置强度
      uncovered_cloud_->cloud_->points.push_back(uncovered_point); // 将未覆盖点添加到点云中
      uncovered_point_num++; // 增加未覆盖点的数量
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("planning_env"), "点云中g值大于0的点数量: %d，添加到视点管理器点的普通点数量: %d，总共点云数量: %d", g_greater_than_zero_count, uncovered_point_num, planner_cloud_->cloud_->points.size());


  // 检查未覆盖的前沿点
  if (parameters_.kUseFrontier)
  {
    // 遍历过滤后的前沿点云中的每个点
    for (int i = 0; i < filtered_frontier_cloud_->cloud_->points.size(); i++)
    {
      pcl::PointXYZI point = filtered_frontier_cloud_->cloud_->points[i]; // 获取当前点
      bool observed = false; // 标记当前点是否被观察到
      // 遍历视点管理器中的候选视点索引
      for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
      {
        // 检查视点是否未被访问
        if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
        {
          // 检查当前点是否被该视点可见
          if (viewpoint_manager->VisibleByViewPoint<pcl::PointXYZI>(point, viewpoint_ind))
          {
            // 将未覆盖的前沿点添加到视点管理器中
            viewpoint_manager->AddUncoveredFrontierPoint(viewpoint_ind, uncovered_frontier_point_num);
            observed = true; // 标记为已观察
          }
        }
      }
      // 如果当前点被观察到
      if (observed)
      {
        pcl::PointXYZI uncovered_frontier_point; // 创建未覆盖的前沿点
        uncovered_frontier_point.x = point.x; // 设置x坐标
        uncovered_frontier_point.y = point.y; // 设置y坐标
        uncovered_frontier_point.z = point.z; // 设置z坐标
        uncovered_frontier_point.intensity = i; // 设置强度
        uncovered_frontier_cloud_->cloud_->points.push_back(uncovered_frontier_point); // 将未覆盖的前沿点添加到点云中
        uncovered_frontier_point_num++; // 增加未覆盖前沿点的数量
      }
    }
  }
}

void PlanningEnv::GetVisualizationPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud)
{
  pointcloud_manager_->GetVisualizationPointCloud(vis_cloud);
}

void PlanningEnv::PublishStackedCloud()
{
  stacked_cloud_->Publish();
}

void PlanningEnv::PublishUncoveredCloud()
{
  uncovered_cloud_->Publish();
}

void PlanningEnv::PublishUncoveredFrontierCloud()
{
  uncovered_frontier_cloud_->Publish();
}

}  // namespace planning_env_ns