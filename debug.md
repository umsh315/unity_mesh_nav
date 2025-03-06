# 参数修改硬编码处
    1、(current_time - start_time_) > 5

# 触发探索结束的原因
## 1、local_coverage_complete_ = true
    selected_viewpoint_indices_itr 为空 <——
    第一次或第二次调用SolveTSP函数时，执行 SelectViewPointFromFrontierQueue 添加
    （未完待续）


## 2、return_home_ = true (打印“没有探索的单元格索引”)
    exploring_cell_indices为空 <——

    缺少Exploring状态的点 <——

    执行GetExploringCellIndices函数时，subspaces_->GetCell(i).GetStatus() == CellStatus的状态都是UNSEEN <——

    这种时候，必须selected_viewpoint_count>0 ,但实际都等于0 <——

    执行 ViewPointSelected 执行ViewPoint 对象的 Selected 方法返回布尔值 selected_ <——

    selected_ 变量的值是在 ViewPoint 类中通过 SetSelected 方法进行设置 <——

    SetViewPointSelected函数中 调用SetSelected 方法设置 <——

    SolveLocalCoverageProblem函数中，当满足viewpoint_index和robot_viewpoint_ind_、start_viewpoint_ind_、end_viewpoint_ind_、lookahead_viewpoint_ind_一系列关系后，调用SetViewPointSelected函数设置为true <——

    viewpoint_index 来自 last_selected_viewpoint_indices_ 来自 ordered_viewpoint_indices <——

    ordered_viewpoint_indices 来自SolveLocalCoverageProblem函数触发时，调用 SolveTSP函数 和 selected_viewpoint_indices_itr参数 <——

    而selected_viewpoint_indices_itr的赋值，来自SolveLocalCoverageProblem函数触发时，调用SelectViewPoint（从队列中选择视点）+调用SelectViewPointFromFrontierQueue（从前沿队列中选择视点） <——

    SelectViewPoint函数 ——> 选择非前沿模式
### queue(covered_point_list)来源
    UpdateCoveredAreas 函数 调用——> GetUncoveredArea(planning_env.cpp)函数 调用——> 
    根据VisibleByViewPoint判断条件，决定是否调用——> 
    根据point点的g值情况，决定是否参与——> 
    
        ## g值大于0就放弃，g值大于0有什么特殊意义吗？？
        ## g值在一开始转换的队列里有吗——观察/keypose_cloud话题
    AddUncoveredPoint 函数 返回给——> AddCoveredPoint  添加到covered_point_list_

    SolveLocalCoverageProblem函数中，先触发EnqueueViewpointCandidates函数 （给queue赋值，frontier_queue赋值函数）——>
    EnqueueViewpointCandidates 函数 ——> 触发 GetViewPointCoveredPointNum  ——>
    GetViewPointCoveredPointList 调用 GetCoveredPointList 返回 covered_point_list_ ——> 
    
    视点覆盖普通点队列的数量给——> covered_point_num
    covered_point_num 满足条件 ——>触发cover_point_queue.emplace_back(covered_point_num, viewpoint_index)队列更新 (queue队列）——>
    最终调用 SelectViewPoint 函数 ——> 选择非前沿模式


    SelectViewPointFromFrontierQueue函数 ——> 最终调用 SelectViewPoint 函数
### frontier_point_队列来源
    filtered_frontier_cloud_：pcl::PointXYZI类型
    VisibleByViewPoint(很多点不满足要求) 返回给——> 
    GetUncoveredArea(planning_env.cpp) 返回给——>
    AddUncoveredFrontierPoint 返回给——>
    AddCoveredFrontierPoint 返回给——> covered_frontier_point_list_.size() 
    
 

    SolveLocalCoverageProblem函数中，先触发EnqueueViewpointCandidates函数 （给queue赋值，frontier_queue赋值函数）——>
    EnqueueViewpointCandidates 函数 ——> 触发GetViewPointCoveredFrontierPointNum 获取 covered_frontier_point_num ——>
    GetCoveredFrontierPointNum 返回给 ——> GetViewPointCoveredFrontierPointNum 返回给——> covered_frontier_point_num （更新视点覆盖的前沿点数量）
    如果covered_frontier_point_num 满足条件 ——>触发frontier_queue.emplace_back(covered_frontier_point_num,viewpoint_index) 队列更新 ——>
    调用SelectViewPointFromFrontierQueue 函数 ——>  最终调用 SelectViewPoint 函数，从更新后的 frontier_queue 中选择视点，并将选中的视点添加到 selected_viewpoint_indices 中 ——> selected_viewpoint_indices的数据结构是一个动态数组，使用 std::vector<int> 来实现


### g值的来源
激光雷达数据来自：sub_registered_scan_topic_ 
——> 触发回调函数：RegisteredScanCallback
——> 将registered_scan_stack_点云（类型PCLCloud<pcl::PointXYZ>）复制给keypose_cloud_（类型PCLCloud<PlannerCloudPointType>）

——> 同步execution_timer_定时器，每1秒触发execute回调函数，触发——> UpdateGlobalRepresentation 函数触发——> UpdateKeyposeCloud函数 
——> 调用UpdateKeyposeCloud函数（关键一步）——> 调用UpdatePointCloud，将PCLPointType类型的点（pcl::PointXYZRGBNormal的别名）储存到pointcloud_grid_对应元素中
——> 调用UpdateCoveredCloudPoints，更新pointcloud_grid_里的points.g  

——> 调用GetPointCloud方法，将pointcloud_grid_的点云，储存到planner_cloud_里
    （planner_cloud_.cloud_ 是pcl::PointCloud<PlannerCloudPointType>::Ptr 类型的指针（别名PlannerCloudType），这是pcl库的标准类型，这个类型一定有一个points变量)



# 总结调试经验

    1、kViewPointCollisionMargin 的2倍要大于 viewpoint_manager/resolution_x 不然waypoint穿墙
    2、kViewPointCollisionMargin 值再稍微大一点，0.05感觉已经超出雷达的最小范围了
    3、terrain_analysis 有个参数noDataObstacle，设置为true算法会把没有激光雷达点的区域，比如向下的楼梯当成障碍物。tare会接受障碍物信息阻挡viewpoint生成。
    4、长直走廊，如果horrizon太小，uncovered point会落在绿色框外，框内没有需要探索的点，探索结束


# 功能问题

    1、走廊容易退化
    2、传感器能力，看不到足够的前沿点，遇到走廊宽度比较窄的90度转角，可能不会探索


       
  

4、 保存时后缀加日期时间

    斜坡处理
          我是通过计算点云的斜率来判断是不是斜坡，然后修改intensity来调整可通行区域的范围，但这样修改仅对local_planner有效，目前还没找到和tare相关的接口










































## AI模块

在启动AI模型时，请在终端中设置 export ROS_DOMAIN_ID=1。

请注意，如果添加了摄像头，数据传输需要将压缩图像发送到基站计算机。请在NUC i7计算机上的 /camera/image/compressed 主题上准备压缩图像。图像在传输后会在 base_station.sh 脚本中解压。
在基站计算机上，请确保订阅 /camera/image/transmitted 主题上的未压缩图像。