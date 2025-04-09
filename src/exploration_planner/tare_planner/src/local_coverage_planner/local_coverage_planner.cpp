/**
 * @file local_coverage_planner.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that ensures coverage in the surroundings of the robot
 * @version 0.1
 * @date 2021-05-31
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "local_coverage_planner/local_coverage_planner.h"

namespace local_coverage_planner_ns {
const std::string LocalCoveragePlanner::kRuntimeUnit = "us";

bool LocalCoveragePlannerParameter::ReadParameters(rclcpp::Node::SharedPtr nh) {
  nh->get_parameter("kMinAddPointNumSmall", kMinAddPointNum);
  nh->get_parameter("kMinAddFrontierPointNum", kMinAddFrontierPointNum);
  nh->get_parameter("kGreedyViewPointSampleRange", kGreedyViewPointSampleRange);
  nh->get_parameter("kLocalPathOptimizationItrMax",
                    kLocalPathOptimizationItrMax);

  return true;
}
LocalCoveragePlanner::LocalCoveragePlanner(rclcpp::Node::SharedPtr nh)
    : lookahead_point_update_(false), use_frontier_(true),
      local_coverage_complete_(false) {
  parameters_.ReadParameters(nh);
}

// 获取边界视点索引
int LocalCoveragePlanner::GetBoundaryViewpointIndex(
    const exploration_path_ns::ExplorationPath &global_path) {
  int boundary_viewpoint_index = robot_viewpoint_ind_; // 初始化边界视点索引为机器人视点索引
  if (!global_path.nodes_.empty()) { // 检查全局路径节点是否为空
    if (viewpoint_manager_->InLocalPlanningHorizon(
            global_path.nodes_.front().position_)) { // 检查全局路径的第一个节点是否在局部规划范围内
      for (int i = 0; i < global_path.nodes_.size(); i++) { // 遍历全局路径的节点
        // 检查节点类型是否为全局视点或家，或者节点不在局部规划范围内
        if (global_path.nodes_[i].type_ ==
                exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
            global_path.nodes_[i].type_ ==
                exploration_path_ns::NodeType::HOME ||
            !viewpoint_manager_->InLocalPlanningHorizon(
                global_path.nodes_[i].position_)) {
          break; // 如果条件满足，跳出循环
        }
        // 获取离当前节点最近的候选视点索引
        boundary_viewpoint_index =
            viewpoint_manager_->GetNearestCandidateViewPointInd(
                global_path.nodes_[i].position_);
      }
    }
  }
  return boundary_viewpoint_index; // 返回边界视点索引
}

void LocalCoveragePlanner::GetBoundaryViewpointIndices(
    exploration_path_ns::ExplorationPath global_path) {
  start_viewpoint_ind_ = GetBoundaryViewpointIndex(global_path);
  global_path.Reverse();
  end_viewpoint_ind_ = GetBoundaryViewpointIndex(global_path);
}

// 获取导航视点索引
void LocalCoveragePlanner::GetNavigationViewPointIndices(
    exploration_path_ns::ExplorationPath global_path,
    std::vector<int> &navigation_viewpoint_indices) {
  // 获取起始点和结束点
  robot_viewpoint_ind_ =
      viewpoint_manager_->GetNearestCandidateViewPointInd(robot_position_);
  lookahead_viewpoint_ind_ =
      viewpoint_manager_->GetNearestCandidateViewPointInd(lookahead_point_);
  // 如果未更新前瞻点或前瞻点不在范围内，则将前瞻点设置为机器人视点
  if (!lookahead_point_update_ ||
      !viewpoint_manager_->InRange(lookahead_viewpoint_ind_)) {
    lookahead_viewpoint_ind_ = robot_viewpoint_ind_;
  }
  // 获取连接到全局路径的视点
  GetBoundaryViewpointIndices(global_path);

  // 更新必须访问的视点覆盖
  navigation_viewpoint_indices.push_back(start_viewpoint_ind_);
  navigation_viewpoint_indices.push_back(end_viewpoint_ind_);
  navigation_viewpoint_indices.push_back(robot_viewpoint_ind_);
  navigation_viewpoint_indices.push_back(lookahead_viewpoint_ind_);
}

void LocalCoveragePlanner::UpdateViewPointCoveredPoint(
    std::vector<bool> &point_list, int viewpoint_index, bool use_array_ind) {
  for (const auto &point_ind : viewpoint_manager_->GetViewPointCoveredPointList(
           viewpoint_index, use_array_ind)) {
    MY_ASSERT(misc_utils_ns::InRange<bool>(point_list, point_ind));
    point_list[point_ind] = true;
  }
}
void LocalCoveragePlanner::UpdateViewPointCoveredFrontierPoint(
    std::vector<bool> &frontier_point_list, int viewpoint_index,
    bool use_array_ind) {
  for (const auto &point_ind :
       viewpoint_manager_->GetViewPointCoveredFrontierPointList(
           viewpoint_index, use_array_ind)) {
    MY_ASSERT(misc_utils_ns::InRange<bool>(frontier_point_list, point_ind));
    frontier_point_list[point_ind] = true;
  }
}

void LocalCoveragePlanner::EnqueueViewpointCandidates(
    std::vector<std::pair<int, int>> &cover_point_queue,
    std::vector<std::pair<int, int>> &frontier_queue,
    const std::vector<bool> &covered_point_list,
    const std::vector<bool> &covered_frontier_point_list,
    const std::vector<int> &selected_viewpoint_array_indices) {


  bool has_candidates = false; // 用于跟踪是否有候选视点
  bool all_visited = true; // 用于跟踪是否所有候选视点都被访问过
  bool all_not_in_cell = true; // 用于跟踪是否所有候选视点都不在可探索单元格中
  bool insufficient_coverage = true; // 用于跟踪是否所有候选视点的覆盖点数量不足

  for (const auto &viewpoint_index :
       viewpoint_manager_->GetViewPointCandidateIndices()) {

    
    has_candidates = true; // 找到候选视点
    all_visited = all_visited && viewpoint_manager_->ViewPointVisited(viewpoint_index);
    all_not_in_cell = all_not_in_cell && !viewpoint_manager_->ViewPointInExploringCell(viewpoint_index);

    if (viewpoint_manager_->ViewPointVisited(viewpoint_index)) {
      continue;
    }
    
    if (!viewpoint_manager_->ViewPointInExploringCell(viewpoint_index)) {
      // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "Viewpoint %d is not in exploring cell.", viewpoint_index);
      continue;
    }

    int viewpoint_array_index =
        viewpoint_manager_->GetViewPointArrayInd(viewpoint_index);

    if (std::find(selected_viewpoint_array_indices.begin(),
                  selected_viewpoint_array_indices.end(),
                  viewpoint_array_index) !=
        selected_viewpoint_array_indices.end()) {
      continue;
    }
    // 返回视点覆盖普通点队列的数量
    int covered_point_num = viewpoint_manager_->GetViewPointCoveredPointNum(
        covered_point_list, viewpoint_array_index, true);

    // if (covered_point_num >= 1) {
    //     // 打印当前视点覆盖的普通点数量
    //     RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "当前视点 %d 覆盖的普通点数量: %d", viewpoint_array_index, covered_point_num);
    // }


    int covered_frontier_point_num = viewpoint_manager_->GetViewPointCoveredFrontierPointNum(
        covered_frontier_point_list, viewpoint_array_index, true);

    
    if (covered_point_num >= parameters_.kMinAddPointNum) {
      cover_point_queue.emplace_back(covered_point_num, viewpoint_index);

      // 打印covered_point_queue的大小以及每个视点对应的covered_point_num
      // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "存在视点覆盖普通点的数量covered_point_num大于阈值的视点队列数量为: %zu", cover_point_queue.size());
      insufficient_coverage = false; // 找到一个覆盖点数量足够的视点
    } 
    else if (use_frontier_) {
      if (covered_frontier_point_num >= parameters_.kMinAddFrontierPointNum) {
        frontier_queue.emplace_back(covered_frontier_point_num,
                                    viewpoint_index);
        insufficient_coverage = false; // 找到一个覆盖前沿点数量足够的视点
      }
    }
  }

  
  // for (const auto& pair : cover_point_queue) {
  //   RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "Viewpoint %d has covered points: %d", pair.second, pair.first);
  // }


  // // 日志打印以检查候选视点的状态
  // if (!has_candidates) {
  //   RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "No candidate viewpoints available.");
  // } else if (all_visited) {
  //   RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "All candidate viewpoints have been visited.");
  // } else if (all_not_in_cell) {
  //   RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "All candidate viewpoints are not in exploring cells.");
  // } else if (insufficient_coverage) {
  //   RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "All candidate viewpoints have insufficient coverage.");
  // } else if (frontier_queue.empty()) {
  //   RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "frontier_queue is empty after processing candidates.");
  // }


  // Sort the queue
  std::sort(cover_point_queue.begin(), cover_point_queue.end(), SortPairInRev);
  if (use_frontier_) {
    std::sort(frontier_queue.begin(), frontier_queue.end(), SortPairInRev);
  }
}

void LocalCoveragePlanner::SelectViewPoint(
    const std::vector<std::pair<int, int>> &queue,  // 输入的视点队列,pair中first是覆盖点数,second是视点索引
    const std::vector<bool> &covered,  // 已覆盖点的标记数组
    std::vector<int> &selected_viewpoint_indices,  // 输出的已选择视点索引数组
    bool use_frontier) {  // 是否使用前沿点模式
  if (use_frontier) {  // 如果使用前沿点模式
    if (queue.empty() || queue[0].first < parameters_.kMinAddFrontierPointNum) {  // 如果队列为空或首个视点覆盖的前沿点数小于阈值（视点最小覆盖的前沿点数量,原值10）
      return;  // 直接返回
    }
  } else {  // 如果不使用前沿点模式
    if (queue.empty() || queue[0].first < parameters_.kMinAddPointNum) {  // 如果队列为空或首个视点覆盖的点数小于阈值（视点最小覆盖的点数量 ,原值30）
      return;  // 直接返回
    }
  }

  std::vector<bool> covered_copy;  // 创建已覆盖点标记数组的副本
  for (int i = 0; i < covered.size(); i++) {  // 遍历原数组
    covered_copy.push_back(covered[i]);  // 复制每个元素
  }
  std::vector<std::pair<int, int>> queue_copy;  // 创建视点队列的副本
  for (int i = 0; i < queue.size(); i++) {  // 遍历原队列
    queue_copy.push_back(queue[i]);  // 复制每个元素
  }

  int sample_range = 0;  // 初始化采样范围
  for (int i = 0; i < queue_copy.size(); i++) {  // 遍历队列副本
    if (use_frontier) {  // 如果使用前沿点模式
      if (queue_copy[i].first >= parameters_.kMinAddFrontierPointNum) {  // 如果视点覆盖的前沿点数大于等于阈值
        sample_range++;  // 增加采样范围
      }
    } else {  // 如果不使用前沿点模式
      if (queue_copy[i].first >= parameters_.kMinAddPointNum) {  // 如果视点覆盖的点数大于等于阈值
        sample_range++;  // 增加采样范围
      }
    }
  }

  sample_range =  // 限制采样范围
      std::min(parameters_.kGreedyViewPointSampleRange, sample_range);  // 取贪心采样范围和计算得到的范围的较小值
  std::random_device rd;  // 创建随机数生成器
  std::mt19937 gen(rd());  // 使用Mersenne Twister算法初始化随机数生成器
  std::uniform_int_distribution<int> gen_next_queue_idx(0, sample_range - 1);  // 创建均匀分布的整数随机数生成器
  int queue_idx = gen_next_queue_idx(gen);  // 生成随机索引
  int cur_ind = queue_copy[queue_idx].second;  // 获取对应的视点索引

  while (true) {  // 循环处理视点
    int cur_array_ind = viewpoint_manager_->GetViewPointArrayInd(cur_ind);  // 获取当前视点在数组中的索引
    if (use_frontier) {  // 如果使用前沿点模式
      for (const auto &point_ind :  // 遍历当前视点覆盖的前沿点
           viewpoint_manager_->GetViewPointCoveredFrontierPointList(
               cur_array_ind, true))  // 获取视点覆盖的前沿点列表
      {
        MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));  // 断言检查点索引是否在范围内
        if (!covered_copy[point_ind]) {  // 如果该点尚未被覆盖
          covered_copy[point_ind] = true;  // 标记为已覆盖
        }
      }
    } else {  // 如果不使用前沿点模式
      for (const auto &point_ind :  // 遍历当前视点覆盖的点
           viewpoint_manager_->GetViewPointCoveredPointList(cur_array_ind,
                                                            true)) {  // 获取视点覆盖的点列表
        MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));  // 断言检查点索引是否在范围内
        if (!covered_copy[point_ind]) {  // 如果该点尚未被覆盖
          covered_copy[point_ind] = true;  // 标记为已覆盖
        }
      }
    }
    selected_viewpoint_indices.push_back(cur_ind);  // 将当前视点添加到已选择视点列表
    queue_copy.erase(queue_copy.begin() + queue_idx);  // 从队列中删除已选择的视点

    // 更新队列中剩余视点的覆盖点数
    for (int i = 0; i < queue_copy.size(); i++) {  // 遍历剩余视点
      int add_point_num = 0;  // 初始化新增覆盖点数
      int ind = queue_copy[i].second;  // 获取视点索引
      int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);  // 获取视点在数组中的索引
      if (use_frontier) {  // 如果使用前沿点模式
        for (const auto &point_ind :  // 遍历视点覆盖的前沿点
             viewpoint_manager_->GetViewPointCoveredFrontierPointList(array_ind,
                                                                      true)) {  // 获取视点覆盖的前沿点列表
          MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));  // 断言检查点索引是否在范围内
          if (!covered_copy[point_ind]) {  // 如果该点尚未被覆盖
            add_point_num++;  // 增加新增覆盖点数
          }
        }
      } else {  // 如果不使用前沿点模式
        for (const auto &point_ind :  // 遍历视点覆盖的点
             viewpoint_manager_->GetViewPointCoveredPointList(array_ind,
                                                              true)) {  // 获取视点覆盖的点列表
          MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));  // 断言检查点索引是否在范围内
          if (!covered_copy[point_ind]) {  // 如果该点尚未被覆盖
            add_point_num++;  // 增加新增覆盖点数
          }
        }
      }

      queue_copy[i].first = add_point_num;  // 更新视点的覆盖点数
    }

    std::sort(queue_copy.begin(), queue_copy.end(), SortPairInRev);  // 根据覆盖点数对队列重新排序

    if (queue_copy.empty() ||  // 如果队列为空
        queue_copy[0].first < parameters_.kMinAddPointNum) {  // 或首个视点的覆盖点数小于阈值
      break;  // 退出循环
    }
    if (use_frontier) {  // 如果使用前沿点模式
      if (queue_copy.empty() ||  // 如果队列为空
          queue_copy[0].first < parameters_.kMinAddFrontierPointNum) {  // 或首个视点的覆盖前沿点数小于阈值
        break;  // 退出循环
      }
    }

    // 随机选择下一个视点
    int sample_range = 0;  // 初始化采样范围
    for (int i = 0; i < queue.size(); i++) {  // 遍历原队列
      if (use_frontier) {  // 如果使用前沿点模式
        if (queue[i].first >= parameters_.kMinAddFrontierPointNum) {  // 如果视点覆盖的前沿点数大于等于阈值
          sample_range++;  // 增加采样范围
        }
      } else {  // 如果不使用前沿点模式
        if (queue[i].first >= parameters_.kMinAddPointNum) {  // 如果视点覆盖的点数大于等于阈值
          sample_range++;  // 增加采样范围
        }
      }
    }
    sample_range =  // 限制采样范围
        std::min(parameters_.kGreedyViewPointSampleRange, sample_range);  // 取贪心采样范围和计算得到的范围的较小值
    std::uniform_int_distribution<int> gen_next_queue_idx(0, sample_range - 1);  // 创建均匀分布的整数随机数生成器
    queue_idx = gen_next_queue_idx(gen);  // 生成随机索引
    cur_ind = queue_copy[queue_idx].second;  // 获取对应的视点索引
  }
}

void LocalCoveragePlanner::SelectViewPointFromFrontierQueue(
    std::vector<std::pair<int, int>> &frontier_queue,
    std::vector<bool> &frontier_covered,
    std::vector<int> &selected_viewpoint_indices) {

  // 添加日志以检查frontier_queue是否为空
  // if (frontier_queue.empty()) {
  //   RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "frontier_queue is empty.");
  // } else {
  //   RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "frontier_queue has %zu viewpoints.", frontier_queue.size());
  // }

  // 打印use_frontier_和frontier_queue[0].first
  
  // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "frontier_queue[0].first: %d", frontier_queue.empty() ? 0 : frontier_queue[0].first);

  // kMinAddFrontierPointNum=10
  if (use_frontier_ && !frontier_queue.empty() &&
      frontier_queue[0].first > parameters_.kMinAddFrontierPointNum) {
    // Update the frontier queue
    for (const auto &ind : selected_viewpoint_indices) {
      UpdateViewPointCoveredFrontierPoint(frontier_covered, ind);
    }
    for (int i = 0; i < frontier_queue.size(); i++) {
      int ind = frontier_queue[i].second;
      int covered_frontier_point_num =
          viewpoint_manager_->GetViewPointCoveredFrontierPointNum(
              frontier_covered, ind);
      frontier_queue[i].first = covered_frontier_point_num;
    }
    std::sort(frontier_queue.begin(), frontier_queue.end(), SortPairInRev);
    SelectViewPoint(frontier_queue, frontier_covered,
                    selected_viewpoint_indices, true);
  }
}

exploration_path_ns::ExplorationPath LocalCoveragePlanner::SolveTSP(
    const std::vector<int> &selected_viewpoint_indices,
    std::vector<int> &ordered_viewpoint_indices)

{
  // nav_msgs::msg::Path tsp_path;
  exploration_path_ns::ExplorationPath tsp_path;

  // // 打印选定视点索引的大小
  // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "开始执行SolveTSP函数是，视点队列大小: %zu", selected_viewpoint_indices.size());


  if (selected_viewpoint_indices.empty()) {
    return tsp_path;
  }

  // Get start and end index
  // 初始化起始索引为选定视点索引的最后一个元素
  int start_ind = selected_viewpoint_indices.size() - 1;
  // 初始化结束索引为选定视点索引的最后一个元素
  int end_ind = selected_viewpoint_indices.size() - 1;
  // 初始化机器人索引为0
  int robot_ind = 0;
  // 初始化前瞻索引为0
  int lookahead_ind = 0;

  // 遍历选定的视点索引
  for (int i = 0; i < selected_viewpoint_indices.size(); i++) {
    // 如果当前视点是起始视点，则更新起始索引
    if (selected_viewpoint_indices[i] == start_viewpoint_ind_) {
      start_ind = i;
    }
    // 如果当前视点是结束视点，则更新结束索引
    if (selected_viewpoint_indices[i] == end_viewpoint_ind_) {
      end_ind = i;
    }
    // 如果当前视点是机器人视点，则更新机器人索引
    if (selected_viewpoint_indices[i] == robot_viewpoint_ind_) {
      robot_ind = i;
    }
    // 如果当前视点是前瞻视点，则更新前瞻索引
    if (selected_viewpoint_indices[i] == lookahead_viewpoint_ind_) {
      lookahead_ind = i;
    }
  }

  // 检查起点和终点是否不同,如果不同则需要添加虚拟节点来连接它们，值为true
  bool has_start_end_dummy = start_ind != end_ind;
  bool has_robot_lookahead_dummy = robot_ind != lookahead_ind;

  // Get distance matrix
  int node_size;
  if (has_start_end_dummy && has_robot_lookahead_dummy) {
    node_size = selected_viewpoint_indices.size() + 2;
  } else if (has_start_end_dummy || has_robot_lookahead_dummy) {
    node_size = selected_viewpoint_indices.size() + 1;
  } else {
    node_size = selected_viewpoint_indices.size();
  }
  misc_utils_ns::Timer find_path_timer("find path");
  find_path_timer.Start();
  std::vector<std::vector<int>> distance_matrix(node_size,
                                                std::vector<int>(node_size, 0));
  std::vector<int> tmp;
  for (int i = 0; i < selected_viewpoint_indices.size(); i++) {
    int from_ind = selected_viewpoint_indices[i];
    // int from_graph_idx = graph_index_map_[from_ind];
    for (int j = 0; j < i; j++) {
      int to_ind = selected_viewpoint_indices[j];
      nav_msgs::msg::Path path =
          viewpoint_manager_->GetViewPointShortestPath(from_ind, to_ind);
      double path_length = misc_utils_ns::GetPathLength(path);
      //   int to_graph_idx = graph_index_map_[to_ind];
      //   double path_length =
      //       misc_utils_ns::AStarSearch(candidate_viewpoint_graph_,
      //       candidate_viewpoint_dist_,
      //                                  candidate_viewpoint_position_,
      //                                  from_graph_idx, to_graph_idx, false,
      //                                  tmp);
      distance_matrix[i][j] = static_cast<int>(10 * path_length);
    }
  }

  for (int i = 0; i < selected_viewpoint_indices.size(); i++) {
    for (int j = i + 1; j < selected_viewpoint_indices.size(); j++) {
      distance_matrix[i][j] = distance_matrix[j][i];
    }
  }

  // Add a dummy node to connect the start and end nodes
  if (has_start_end_dummy && has_robot_lookahead_dummy) {
    int start_end_dummy_node_ind = node_size - 1;
    int robot_lookahead_dummy_node_ind = node_size - 2;
    for (int i = 0; i < selected_viewpoint_indices.size(); i++) {
      if (i == start_ind || i == end_ind) {
        distance_matrix[i][start_end_dummy_node_ind] = 0;
        distance_matrix[start_end_dummy_node_ind][i] = 0;
      } else {
        distance_matrix[i][start_end_dummy_node_ind] = 9999;
        distance_matrix[start_end_dummy_node_ind][i] = 9999;
      }
      if (i == robot_ind || i == lookahead_ind) {
        distance_matrix[i][robot_lookahead_dummy_node_ind] = 0;
        distance_matrix[robot_lookahead_dummy_node_ind][i] = 0;
      } else {
        distance_matrix[i][robot_lookahead_dummy_node_ind] = 9999;
        distance_matrix[robot_lookahead_dummy_node_ind][i] = 9999;
      }
    }

    distance_matrix[start_end_dummy_node_ind][robot_lookahead_dummy_node_ind] =
        9999;
    distance_matrix[robot_lookahead_dummy_node_ind][start_end_dummy_node_ind] =
        9999;
  } else if (has_start_end_dummy) {
    int end_node_ind = node_size - 1;
    for (int i = 0; i < selected_viewpoint_indices.size(); i++) {
      if (i == start_ind || i == end_ind) {
        distance_matrix[i][end_node_ind] = 0;
        distance_matrix[end_node_ind][i] = 0;
      } else {
        distance_matrix[i][end_node_ind] = 9999;
        distance_matrix[end_node_ind][i] = 9999;
      }
    }
  } else if (has_robot_lookahead_dummy) {
    int end_node_ind = node_size - 1;
    for (int i = 0; i < selected_viewpoint_indices.size(); i++) {
      if (i == robot_ind || i == lookahead_ind) {
        distance_matrix[i][end_node_ind] = 0;
        distance_matrix[end_node_ind][i] = 0;
      } else {
        distance_matrix[i][end_node_ind] = 9999;
        distance_matrix[end_node_ind][i] = 9999;
      }
    }
  }

  find_path_timer.Stop(false);
  find_path_runtime_ += find_path_timer.GetDuration(kRuntimeUnit);

  misc_utils_ns::Timer tsp_timer("tsp");
  tsp_timer.Start();

  tsp_solver_ns::DataModel data;
  data.distance_matrix = distance_matrix;
  data.depot = start_ind;

  tsp_solver_ns::TSPSolver tsp_solver(data);
  tsp_solver.Solve();

  // 存储TSP求解得到的路径节点索引
  std::vector<int> path_index;
  if (has_start_end_dummy) {
    // 如果有起点和终点的虚拟节点,获取包含虚拟节点的路径索引
    tsp_solver.getSolutionNodeIndex(path_index, true);
  } else {
    // 否则获取不包含虚拟节点的路径索引
    tsp_solver.getSolutionNodeIndex(path_index, false);
  }

  // 移除连接机器人和前瞻点的虚拟节点
  for (int i = 0; i < path_index.size(); i++) {
    // 如果路径索引超出选定视点索引范围或为负数
    if (path_index[i] >= selected_viewpoint_indices.size() ||
        path_index[i] < 0) {
      // 从路径中删除该虚拟节点索引
      path_index.erase(path_index.begin() + i);
      i--; // 由于删除了一个元素,索引需要回退
    }
  }

  // 清空已排序的视点索引列表
  ordered_viewpoint_indices.clear();
  // 根据TSP求解得到的路径顺序重新排列视点索引
  for (int i = 0; i < path_index.size(); i++) {
    ordered_viewpoint_indices.push_back(
        selected_viewpoint_indices[path_index[i]]);
  }

  // Add the end node index
  if (start_ind == end_ind && !path_index.empty()) {
    path_index.push_back(path_index[0]);
  }

  tsp_timer.Stop(false);
  tsp_runtime_ += tsp_timer.GetDuration(kRuntimeUnit);

  if (path_index.size() > 1) {
    int cur_ind;
    int next_ind;
    int from_graph_idx;
    int to_graph_idx;

    for (int i = 0; i < path_index.size() - 1; i++) {
      cur_ind = selected_viewpoint_indices[path_index[i]];
      next_ind = selected_viewpoint_indices[path_index[i + 1]];

      //   from_graph_idx = graph_index_map_[cur_ind];
      //   to_graph_idx = graph_index_map_[next_ind];

      // Add viewpoint node
      // int cur_array_ind = grid_->GetArrayInd(cur_ind);
      // geometry_msgs::msg::Point cur_node_position =
      // viewpoints_[cur_array_ind].GetPosition();
      geometry_msgs::msg::Point cur_node_position =
          viewpoint_manager_->GetViewPointPosition(cur_ind);
      exploration_path_ns::Node cur_node(
          cur_node_position, exploration_path_ns::NodeType::LOCAL_VIEWPOINT);
      cur_node.local_viewpoint_ind_ = cur_ind;
      if (cur_ind == robot_viewpoint_ind_) {
        cur_node.type_ = exploration_path_ns::NodeType::ROBOT;
      } else if (cur_ind == lookahead_viewpoint_ind_) {
        int covered_point_num =
            viewpoint_manager_->GetViewPointCoveredPointNum(cur_ind);
        int covered_frontier_num =
            viewpoint_manager_->GetViewPointCoveredFrontierPointNum(cur_ind);
        if (covered_point_num > parameters_.kMinAddPointNum ||
            covered_frontier_num > parameters_.kMinAddFrontierPointNum) {
          cur_node.type_ = exploration_path_ns::NodeType::LOCAL_VIEWPOINT;
        } else {
          cur_node.type_ = exploration_path_ns::NodeType::LOOKAHEAD_POINT;
        }
      } else if (cur_ind == start_viewpoint_ind_) {
        cur_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
      } else if (cur_ind == end_viewpoint_ind_) {
        cur_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
      }
      tsp_path.Append(cur_node);

      nav_msgs::msg::Path path_between_viewpoints =
          viewpoint_manager_->GetViewPointShortestPath(cur_ind, next_ind);

      //   std::vector<int> path_graph_indices;
      //   misc_utils_ns::AStarSearch(candidate_viewpoint_graph_,
      //   candidate_viewpoint_dist_, candidate_viewpoint_position_,
      //                              from_graph_idx, to_graph_idx, true,
      //                              path_graph_indices);
      // Add viapoint nodes;
      //   if (path_graph_indices.size() > 2)
      if (path_between_viewpoints.poses.size() > 2) {
        // for (int j = 1; j < path_graph_indices.size() - 1; j++)
        for (int j = 1; j < path_between_viewpoints.poses.size() - 1; j++) {
          //   int graph_idx = path_graph_indices[j];
          //   int ind = candidate_indices_[graph_idx];
          exploration_path_ns::Node node;
          node.type_ = exploration_path_ns::NodeType::LOCAL_VIA_POINT;
          node.local_viewpoint_ind_ = -1;
          //   geometry_msgs::msg::Point node_position =
          //   viewpoint_manager_->GetViewPointPosition(ind); node.position_.x()
          //   = node_position.x; node.position_.y() = node_position.y;
          //   node.position_.z() = node_position.z;
          node.position_.x() = path_between_viewpoints.poses[j].pose.position.x;
          node.position_.y() = path_between_viewpoints.poses[j].pose.position.y;
          node.position_.z() = path_between_viewpoints.poses[j].pose.position.z;
          tsp_path.Append(node);
        }
      }

      geometry_msgs::msg::Point next_node_position =
          viewpoint_manager_->GetViewPointPosition(next_ind);
      exploration_path_ns::Node next_node(
          next_node_position, exploration_path_ns::NodeType::LOCAL_VIEWPOINT);
      next_node.local_viewpoint_ind_ = next_ind;
      if (next_ind == robot_viewpoint_ind_) {
        next_node.type_ = exploration_path_ns::NodeType::ROBOT;
      } else if (next_ind == lookahead_viewpoint_ind_) {
        next_node.type_ = exploration_path_ns::NodeType::LOOKAHEAD_POINT;
      } else if (next_ind == start_viewpoint_ind_) {
        next_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
      } else if (next_ind == end_viewpoint_ind_) {
        next_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
      }
      tsp_path.Append(next_node);
    }
  }

  return tsp_path;
}

exploration_path_ns::ExplorationPath
LocalCoveragePlanner::SolveLocalCoverageProblem(
    const exploration_path_ns::ExplorationPath &global_path,
    int uncovered_point_num, int uncovered_frontier_point_num) {
  exploration_path_ns::ExplorationPath local_path;

  find_path_runtime_ = 0;
  viewpoint_sampling_runtime_ = 0;
  tsp_runtime_ = 0;

  local_coverage_complete_ = false;

  misc_utils_ns::Timer find_path_timer("find path");
  find_path_timer.Start();

  std::vector<int> navigation_viewpoint_indices;
  GetNavigationViewPointIndices(global_path, navigation_viewpoint_indices);

  find_path_timer.Stop(false);
  find_path_runtime_ += find_path_timer.GetDuration(kRuntimeUnit);

  // Sampling viewpoints
  misc_utils_ns::Timer viewpoint_sampling_timer("viewpoint sampling");
  viewpoint_sampling_timer.Start();

  std::vector<bool> covered(uncovered_point_num, false);
  std::vector<bool> frontier_covered(uncovered_frontier_point_num, false);

  std::vector<int> pre_selected_viewpoint_array_indices;
  std::vector<int> reused_viewpoint_indices;
  for (auto &viewpoint_array_ind : last_selected_viewpoint_array_indices_) {
    if (viewpoint_manager_->ViewPointVisited(viewpoint_array_ind, true) ||
        !viewpoint_manager_->IsViewPointCandidate(viewpoint_array_ind, true)) {
      continue;
    }
    int covered_point_num = viewpoint_manager_->GetViewPointCoveredPointNum(
        covered, viewpoint_array_ind, true);
    if (covered_point_num >= parameters_.kMinAddPointNum) {
      reused_viewpoint_indices.push_back(
          viewpoint_manager_->GetViewPointInd(viewpoint_array_ind));
    } else if (use_frontier_) {
      int covered_frontier_point_num =
          viewpoint_manager_->GetViewPointCoveredFrontierPointNum(
              frontier_covered, viewpoint_array_ind, true);
      if (covered_frontier_point_num >= parameters_.kMinAddFrontierPointNum) {
        reused_viewpoint_indices.push_back(
            viewpoint_manager_->GetViewPointInd(viewpoint_array_ind));
      }
    }
  }

  for (const auto &ind : reused_viewpoint_indices) {
    int viewpoint_array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
    pre_selected_viewpoint_array_indices.push_back(viewpoint_array_ind);
  }
  for (const auto &ind : navigation_viewpoint_indices) {
    int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
    pre_selected_viewpoint_array_indices.push_back(array_ind);
  }

  // Update coverage
  for (auto &viewpoint_array_ind : pre_selected_viewpoint_array_indices) {
    // Update covered points and frontiers
    UpdateViewPointCoveredPoint(covered, viewpoint_array_ind, true);
    if (use_frontier_) {
      UpdateViewPointCoveredFrontierPoint(frontier_covered, viewpoint_array_ind,
                                          true);
    }
  }

  // Enqueue candidate viewpoints
  std::vector<std::pair<int, int>> queue;
  std::vector<std::pair<int, int>> frontier_queue;

  // queue赋值，frontier_queue赋值函数
  EnqueueViewpointCandidates(queue, frontier_queue, covered, frontier_covered,
                             pre_selected_viewpoint_array_indices);

  viewpoint_sampling_timer.Stop(false, kRuntimeUnit);
  viewpoint_sampling_runtime_ +=
      viewpoint_sampling_timer.GetDuration(kRuntimeUnit);

  std::vector<int> ordered_viewpoint_indices;
  // queue队列就是covered_point_list_队列，
  if (!queue.empty() && queue[0].first >= parameters_.kMinAddPointNum) {
    double min_path_length = DBL_MAX;
    for (int itr = 0; itr < parameters_.kLocalPathOptimizationItrMax; itr++) {
      std::vector<int> selected_viewpoint_indices_itr;

      // Select from the queue
      // 创建计时器以选择视点
      misc_utils_ns::Timer select_viewpoint_timer("select viewpoints");
      select_viewpoint_timer.Start();
      // 打印queue和frontier_queue的大小
      // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "queue队列满足条件时普通视点队列queue size: %zu", queue.size());
      // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "queue队列满足条件时前沿视点队列frontier_queue size: %zu", frontier_queue.size());

      // 从队列中选择视点
      SelectViewPoint(queue, covered, selected_viewpoint_indices_itr, false);
      // 打印selected_viewpoint_indices_itr的大小
      // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "queue队列满足条件下调用TSP，普通模式获取selected_viewpoint_indices_itr队列视点大小为: %zu", selected_viewpoint_indices_itr.size());

      // 从前沿队列中选择视点
      SelectViewPointFromFrontierQueue(frontier_queue, frontier_covered, selected_viewpoint_indices_itr);

      // 打印selected_viewpoint_indices_itr的大小
      // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "queue队列满足条件下调用TSP，前沿模式获取selected_viewpoint_indices_itr队列视点大小为: %zu", selected_viewpoint_indices_itr.size());




      // 添加上一个规划周期的视点
      for (const auto &ind : reused_viewpoint_indices) {

        
        
        selected_viewpoint_indices_itr.push_back(ind);
      }


      // 添加用于导航的视点
      for (const auto &ind : navigation_viewpoint_indices) {

       
        selected_viewpoint_indices_itr.push_back(ind);
      }

      // 视点索引去重
      misc_utils_ns::UniquifyIntVector(selected_viewpoint_indices_itr);

      // 停止计时器并更新运行时间
      select_viewpoint_timer.Stop(false, kRuntimeUnit);
      viewpoint_sampling_runtime_ +=
          select_viewpoint_timer.GetDuration(kRuntimeUnit);

      // Solve the TSP problem
      exploration_path_ns::ExplorationPath local_path_itr;

      // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "正在第1次调用SolveTSP函数");
      local_path_itr =
          SolveTSP(selected_viewpoint_indices_itr, ordered_viewpoint_indices);

      double path_length = local_path_itr.GetLength();
      
      // // 打印路径节点数量和路径长度
      // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "local_path_itr.nodes_ size: %zu", local_path_itr.nodes_.size());
      // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "path_length: %f, min_path_length: %f", path_length, min_path_length);

      if (!local_path_itr.nodes_.empty() && path_length < min_path_length)

      {
        min_path_length = path_length;
        local_path = local_path_itr;
        last_selected_viewpoint_indices_ = ordered_viewpoint_indices;
      }
    }
  } else {
    misc_utils_ns::Timer select_viewpoint_timer("viewpoint sampling"); // 创建一个计时器，用于视点采样
    select_viewpoint_timer.Start(); // 启动计时器

    // std::cout << "entering tsp routine" << std::endl; // 输出调试信息，表示进入TSP例程
    std::vector<int> selected_viewpoint_indices_itr; // 创建一个整数向量，用于存储选定的视点索引

    // Add viewpoints from last planning cycle
    for (const auto &ind : reused_viewpoint_indices) { // 遍历上一个规划周期的视点索引

      
      selected_viewpoint_indices_itr.push_back(ind); // 将视点索引添加到选定的视点索引向量中
    }

    // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "添加reused_viewpoint_indices后，视点数量: %zu", selected_viewpoint_indices_itr.size());

    // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "queue队列不满足，前沿视点队列frontier_queue size: %zu", frontier_queue.size());

    SelectViewPointFromFrontierQueue(frontier_queue, frontier_covered, // 从前沿队列中选择视点
                                     selected_viewpoint_indices_itr); // 传入选定的视点索引向量

    // // 打印selected_viewpoint_indices_itr的大小
    // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "queue队列不满足条件，从前沿队列中选择视点，前沿模式获取selected_viewpoint_indices_itr队列视点大小为: %zu", selected_viewpoint_indices_itr.size());



    if (selected_viewpoint_indices_itr.empty()) { // 如果选定的视点索引向量为空
      local_coverage_complete_ = true; // 设置局部覆盖完成标志为真
    }

    // Add viewpoints for navigation
    for (const auto &ind : navigation_viewpoint_indices) { // 遍历导航视点索引

      selected_viewpoint_indices_itr.push_back(ind); // 将导航视点索引添加到选定的视点索引向量中
    }

    misc_utils_ns::UniquifyIntVector(selected_viewpoint_indices_itr); // 去重选定的视点索引向量

    select_viewpoint_timer.Stop(false, kRuntimeUnit); // 停止计时器
    viewpoint_sampling_runtime_ += // 更新视点采样运行时间
        select_viewpoint_timer.GetDuration(kRuntimeUnit); // 获取计时器的持续时间并累加

    // 打印正在第几次调用SolveTSP函数
    // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "正在第2次调用SolveTSP函数");
    
    local_path =
        SolveTSP(selected_viewpoint_indices_itr, ordered_viewpoint_indices);

    last_selected_viewpoint_indices_ = ordered_viewpoint_indices;
  }

  last_selected_viewpoint_array_indices_.clear();
  for (const auto &ind : last_selected_viewpoint_indices_) {
    int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
    last_selected_viewpoint_array_indices_.push_back(array_ind);
  }

  int viewpoint_num = viewpoint_manager_->GetViewPointNum();
  for (int i = 0; i < viewpoint_num; i++) {
    viewpoint_manager_->SetViewPointSelected(i, false, true);
  }
  for (const auto &viewpoint_index : last_selected_viewpoint_indices_) {

    // // 打印索引值比较情况
    // RCLCPP_INFO(rclcpp::get_logger("local_coverage_planner"), "viewpoint_index: %d, robot_viewpoint_ind_: %d, start_viewpoint_ind_: %d, end_viewpoint_ind_: %d, lookahead_viewpoint_ind_: %d", 
    //              viewpoint_index, robot_viewpoint_ind_, start_viewpoint_ind_, end_viewpoint_ind_, lookahead_viewpoint_ind_);


    if (viewpoint_index != robot_viewpoint_ind_ &&
        viewpoint_index != start_viewpoint_ind_ &&
        viewpoint_index != end_viewpoint_ind_ &&
        viewpoint_index != lookahead_viewpoint_ind_) {
      viewpoint_manager_->SetViewPointSelected(viewpoint_index, true);
    }
  }
  return local_path;
}

void LocalCoveragePlanner::GetSelectedViewPointVisCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
  cloud->clear();
  for (const auto &viewpoint_index : last_selected_viewpoint_indices_) {
    geometry_msgs::msg::Point position =
        viewpoint_manager_->GetViewPointPosition(viewpoint_index);
    pcl::PointXYZI point;
    point.x = position.x;
    point.y = position.y;
    point.z = position.z;
    if (viewpoint_index == robot_viewpoint_ind_) {
      point.intensity = 0.0;
    } else if (viewpoint_index == start_viewpoint_ind_) {
      point.intensity = 1.0;
    } else if (viewpoint_index == end_viewpoint_ind_) {
      point.intensity = 2.0;
    } else {
      point.intensity = 3.0;
    }
    cloud->points.push_back(point);
  }
}

} // namespace local_coverage_planner_ns