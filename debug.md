
# tare调试问题

1、explore.launch文件如果嵌套启动，需手动给个启动waypoint，主要时中间层先要有个路点，不能在默认位置，哪怕此时explore节点还没启动
2、仿真环境参数已验证有效，待同步到真实环境，但目前参数不稳定


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
# queue(covered_point_list)来源
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
# frontier_point_队列来源
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



# g值的来源
激光雷达数据来自：sub_registered_scan_topic_ 
——> 触发回调函数：RegisteredScanCallback
——> 将registered_scan_stack_点云（类型PCLCloud<pcl::PointXYZ>）复制给keypose_cloud_（类型PCLCloud<PlannerCloudPointType>）

——> 同步execution_timer_定时器，每1秒触发execute回调函数，触发——> UpdateGlobalRepresentation 函数触发——> UpdateKeyposeCloud函数 
——> 调用UpdateKeyposeCloud函数（关键一步）——> 调用UpdatePointCloud，将PCLPointType类型的点（pcl::PointXYZRGBNormal的别名）储存到pointcloud_grid_对应元素中
——> 调用UpdateCoveredCloudPoints，更新pointcloud_grid_里的points.g  

——> 调用GetPointCloud方法，将pointcloud_grid_的点云，储存到planner_cloud_里
    （planner_cloud_.cloud_ 是pcl::PointCloud<PlannerCloudPointType>::Ptr 类型的指针（别名PlannerCloudType），这是pcl库的标准类型，这个类型一定有一个points变量)









    
# 调试发现问题及解决

## 总结规律
    1、必须触发第一次TSP，否则不可能持续探索
    2、如果突然触发了[tare_planner_node-11] [INFO] [grid_wolrd]: 没有探索的单元格索引。，基本就死了，然后规划的路线也消失了
    3、20250228下午16点48分，这套参数目前在仿真环境看是奏效的
    4、目前参数实测似乎可以了，但是雷达的噪点太多，试下去畸变的话题
    5、室外场景可以测一下——

    6、20250304版室内走廊可行，但由于雷达不是360度 面朝死角就会死
    7、waypoint穿墙
        1）尝试把路点投射关掉
        3）调小一点绿色框是一下
        1）kSurfaceCloudDwzLeafSize 值再稍微调小一点
        2）kViewPointCollisionMargin 值再稍微大一点，0.05感觉已经超出雷达的最小范围了

        2）是不是terrain参数的问题
        3）Rviz里看terrain_map输出对不对，有没有因为state stimation不准而导致一些噪点被当成障碍物
        4）这个问题可以先检查一下/terrain_map 和state estimation输出对不对。理论上来说TARE会用/terrain_map确定可行驶区域，不会把/way_point放在不能到的地方。但如果/terrain_map 和state estimation输出有误，倒是障碍物没有被检测到则会出现这个问题。另外在小环境中可以尝试减小kExtendWayPointDistance，这样TARE就不会把目标点投影过远至不可到达的地方。
        5）在terrain_analysis里面有个参数noDataObstacle可以设成true试一下。这样terrain_analysis在做可行驶区域分析的时候会在没有激光数据的地方加上与机器人同高的虚拟障碍点
        6）terrain_analysis 有个参数noDataObstacle，设置为true算法会把没有激光雷达点的区域，比如向下的楼梯当成障碍物。tare会接受障碍物信息阻挡viewpoint生成。
        7）你可以把launch文件里的clearDyObs设置成false，不要检测动态障碍物，我们的动态障碍物检测考虑的分辨率比较粗糙，所以可能会把部分不是动态障碍物的部分当成障碍物消除掉，所以导致地面上出现一些空白
        8）另外，地面上出现的红色和黄色的点，你可以在rviz里选中红色的点，看看intensity是多少，然后看一下这些intensity与实际的是不是相符。intensity的计算方式是该点的高度相对于周围0.1m范围内最低点的高度，你可以判断一下是不是正确。
        9）你可以先把terrain_annalysis以及local planner都设置好，确保车辆在这个环境里能够正常的到达waypoint，然后再设置tare planner
        10）在Rviz里打开viewpoint的visualization,把颜色设为intensity，如果能看到比较多的viewpoint点有较大的"intensity"（颜色偏紫色），说明有足够多的viewpoint能看到surface point或者frontier point。这种情况下探索会更仔细和完整。

        11）另外车身高度 minRelZ maxRelZ这些按我自己的小车情况做了相应调整，结果就是小车在自动规划的时候转向没有那么高频且慢了很多，地形图里周围的红点也基本上消除了。
        12）需要注意的是这个参数需要和viewpoint_manager/resolution_[x,y,z]相互配合，确保kViewPointCollisionMargin的两倍要大于viewpoint的resolution，要不然计算出来的路径可能会穿墙

        
    算法不成熟
        1、动态障碍物粗糙
        2、路点强外面
        3、走廊表现很差，很多人反馈，调了很多参数才效果好一些

    斜坡处理
          我是通过计算点云的斜率来判断是不是斜坡，然后修改intensity来调整可通行区域的范围，但这样修改仅对local_planner有效，目前还没找到和tare相关的接口

 ros2 run go2_bringup motion_to_tf 
 
line 136, in cloud_callback
[transform_everything-2]     transformed_points[i][4] = int(transformed_points[i][4])
[transform_everything-2] IndexError: list index out of range
[ERROR] [transform_everything-2]: process has died [pid 67031,

1、int covered_point_num = viewpoint_manager_->GetViewPointCoveredPointNum(
        covered_point_list, viewpoint_array_index, true) 这个函数的处理原理是什么，为什么经常covered_point_num=0
2、尝试降低阈值到1
3、雷达是不是噪声明显，导致g值存在问题（打印g值）————在调用UpdateCoveredCloudPoints函数处理g值处，g值有很多=0

4、尝试用更好雷达试试————已找到对应文件，待进一步处理
5、调整utildar雷达话题
5、Viewpoint 13 is not in exploring cell. 总报这个错

1） [tare_planner_node-11] 当前41274行,2列点的g值: 0 这个节点产生的g=0的点很多
但是到了 [tare_planner_node-11] [INFO] [planning_env]: 点云中g值大于0的点数量: 1017，添加到视点管理器点的普通点数量: 2，总共点云数量: 1106
这个节点 g=0明显变少，且添加到管理器的更少








7、sim2real 有什么范式吗


[tare_planner_node-11] [INFO] [local_coverage_planner]: 开始执行SolveTSP函数是，视点队列大小: 10
[tare_planner_node-11] [INFO] [local_coverage_planner]: queue队列满足条件时普通视点队列queue size: 16
[tare_planner_node-11] [INFO] [local_coverage_planner]: queue队列满足条件时前沿视点队列frontier_queue size: 38
[tare_planner_node-11] [INFO] [local_coverage_planner]: queue队列满足条件下调用TSP，普通模式获取selected_viewpoint_indices_itr队列视点大小为: 7
[tare_planner_node-11] [INFO] [local_coverage_planner]: frontier_queue has 38 viewpoints.
[tare_planner_node-11] [INFO] [local_coverage_planner]: frontier_queue[0].first: 0
[tare_planner_node-11] [INFO] [local_coverage_planner]: queue队列满足条件下调用TSP，前沿模式获取selected_viewpoint_indices_itr队列视点大小为: 7
[tare_planner_node-11] [INFO] [local_coverage_planner]: 正在第一次调用SolveTSP函数
[tare_planner_node-11] [INFO] [local_coverage_planner]: 开始执行SolveTSP函数是，视点队列大小: 10
[tare_planner_node-11] [INFO] [planning_env]: 点云中g值大于0的点数量: 474，添加到视点管理器点的普通点数量: 0，总共点云数量: 572
[tare_planner_node-11] [INFO] [grid_wolrd]: 没有探索的单元格索引。
[tare_planner_node-11] [INFO] [local_coverage_planner]: 添加reused_viewpoint_indices后，视点数量: 2
[tare_planner_node-11] [INFO] [local_coverage_planner]: queue队列不满足，前沿视点队列frontier_queue size: 55
[tare_planner_node-11] [INFO] [local_coverage_planner]: frontier_queue has 55 viewpoints.
[tare_planner_node-11] [INFO] [local_coverage_planner]: frontier_queue[0].first: 31
[tare_planner_node-11] [INFO] [local_coverage_planner]: queue队列不满足条件，从前沿队列中选择视点，前沿模式获取selected_viewpoint_indices_itr队列视点大小为: 3
[tare_planner_node-11] [INFO] [local_coverage_planner]: 正在第2次调用SolveTSP函数
[tare_planner_node-11] [INFO] [local_coverage_planner]: 开始执行SolveTSP函数是，视点队列大小: 6
[tare_planner_node-11] [INFO] [planning_env]: 点云中g值大于0的点数量: 515，添加到视点管理器点的普通点数量: 0，总共点云数量: 618

[tare_planner_node-11] [INFO] [local_coverage_planner]: All candidate viewpoints have been visited.
[tare_planner_node-12] [INFO] [local_coverage_planner]: Viewpoint 1565 is not in exploring cell.














































## AI模块

在启动AI模型时，请在终端中设置 export ROS_DOMAIN_ID=1。

请注意，如果添加了摄像头，数据传输需要将压缩图像发送到基站计算机。请在NUC i7计算机上的 /camera/image/compressed 主题上准备压缩图像。图像在传输后会在 base_station.sh 脚本中解压。
在基站计算机上，请确保订阅 /camera/image/transmitted 主题上的未压缩图像。