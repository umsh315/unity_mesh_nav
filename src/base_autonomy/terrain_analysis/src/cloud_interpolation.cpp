#include "../include/terrain_analysis/cloud_interpolation.hpp"
#include "rclcpp/rclcpp.hpp"

namespace terrain_analysis {

CloudInterpolation::CloudInterpolation() : 
    groundCloud(new pcl::PointCloud<pcl::PointXYZI>()),         // 初始化地面点云指针
    fullCloud(new pcl::PointCloud<pcl::PointXYZI>()),          // 初始化完整点云指针
    patched_ground(new pcl::PointCloud<pcl::PointXYZI>()),     // 初始化补丁地面点云指针
    patched_ground_edge(new pcl::PointCloud<pcl::PointXYZI>()), // 初始化补丁地面边缘点云指针
    first_frame_count_(0) {                                     // 初始化首帧计数器
    
    // 默认参数
    N_SCAN = 16;                                               // 垂直扫描线数
    HORIZONTAL_SCAN = 1800;                                    // 水平扫描点数
    groundScanIndex = 7;                                       // 地面扫描线索引
    
    minimum_range = 0.3;                                       // 最小检测范围
    maximum_range = 100.0;                                     // 最大检测范围
    distance_for_patch_between_rings = 1.0;                    // 扫描线间补丁距离阈值
    
    scan_period = 0.05;                                        // 扫描周期
    segment_theta = 60.0 * DEG_TO_RAD;                        // 分割角度阈值
    segment_valid_point_num = 5;                              // 有效分割点数阈值
    segment_valid_line_num = 3;                               // 有效分割线数阈值
    
    sensorMountAngle = 0.0;                                   // 传感器安装角度
    ang_bottom = -15.0 * DEG_TO_RAD;                         // 底部扫描角度
    ang_top = 15.0 * DEG_TO_RAD;                             // 顶部扫描角度
    
    // 初始化矩阵
    rangeMatrix.resize(N_SCAN, HORIZONTAL_SCAN);              // 调整距离矩阵大小
    groundMatrix.resize(N_SCAN, HORIZONTAL_SCAN);             // 调整地面矩阵大小
}

void CloudInterpolation::setLidarParams(int num_vertical_scans,
                                      int num_horizontal_scans,
                                      int ground_scan_index,
                                      float vertical_angle_bottom,
                                      float vertical_angle_top,
                                      float sensor_mount_angle) {
    N_SCAN = num_vertical_scans;                              // 设置垂直扫描线数
    HORIZONTAL_SCAN = num_horizontal_scans;                   // 设置水平扫描点数
    groundScanIndex = ground_scan_index;                      // 设置地面扫描线索引
    ang_bottom = vertical_angle_bottom * DEG_TO_RAD;         // 设置底部扫描角度
    ang_top = vertical_angle_top * DEG_TO_RAD;               // 设置顶部扫描角度
    sensorMountAngle = sensor_mount_angle * DEG_TO_RAD;      // 设置传感器安装角度
    
    // 重新计算角度分辨率
    ang_resolution_X = (M_PI * 2) / HORIZONTAL_SCAN;         // 计算水平角分辨率
    ang_resolution_Y = (ang_top - ang_bottom) / float(N_SCAN-1); // 计算垂直角分辨率
    
    // 重新初始化矩阵
    rangeMatrix.resize(N_SCAN, HORIZONTAL_SCAN);             // 重置距离矩阵大小
    groundMatrix.resize(N_SCAN, HORIZONTAL_SCAN);            // 重置地面矩阵大小
}

void CloudInterpolation::setProjectionParams(float scan_period_,
                                           float segment_theta_,
                                           float segment_valid_point_num_,
                                           float segment_valid_line_num_,
                                           float maximum_detection_range_,
                                           float minimum_detection_range_,
                                           float distance_for_patch_between_rings_) {
    scan_period = scan_period_;                              // 设置扫描周期
    segment_theta = segment_theta_ * DEG_TO_RAD;            // 设置分割角度阈值
    segment_valid_point_num = segment_valid_point_num_;     // 设置有效分割点数阈值
    segment_valid_line_num = segment_valid_line_num_;       // 设置有效分割线数阈值
    maximum_range = maximum_detection_range_;               // 设置最大检测范围
    minimum_range = minimum_detection_range_;               // 设置最小检测范围
    distance_for_patch_between_rings = distance_for_patch_between_rings_; // 设置扫描线间补丁距离阈值
}

void CloudInterpolation::projectPointCloud() {
    // 初始化
    const size_t cloudSize = groundCloud->points.size();     // 获取点云大小
    fullCloud->clear();                                      // 清空完整点云
    fullCloud->points.resize(N_SCAN * HORIZONTAL_SCAN);     // 调整点云大小
    rangeMatrix.setConstant(FLT_MAX);                       // 初始化距离矩阵
    groundMatrix.setZero();                                 // 初始化地面矩阵
    
    // 遍历输入点云
    for (size_t i = 0; i < cloudSize; ++i) {
        // 获取当前点
        pcl::PointXYZI thisPoint = groundCloud->points[i];

        // 计算该点到原点的距离
        float range = sqrt(thisPoint.x * thisPoint.x +
                         thisPoint.y * thisPoint.y +
                         thisPoint.z * thisPoint.z);

        // 计算该点在图像中的行列索引
        float verticalAngle = std::asin(thisPoint.z / range);

        // 计算行索引
        int rowIdn = (verticalAngle - ang_bottom) / ang_resolution_Y;
        if (rowIdn < 0 || rowIdn >= N_SCAN) {
            continue;
        }

        // 计算水平角度
        float horizonAngle = std::atan2(thisPoint.x, thisPoint.y);

        // 计算列索引
        int columnIdn = -round((horizonAngle - M_PI_2) / ang_resolution_X) + HORIZONTAL_SCAN * 0.5;

        // 处理列索引超出范围的情况
        if (columnIdn >= HORIZONTAL_SCAN){
            columnIdn -= HORIZONTAL_SCAN;
        }

        // 检查列索引是否有效
        if (columnIdn < 0 || columnIdn >= HORIZONTAL_SCAN){
            continue;
        }

        // 检查点的距离是否在有效范围内
        if (range < minimum_range || range > maximum_range){
            continue;
        }

        // 保存距离信息到range矩阵
        rangeMatrix(rowIdn, columnIdn) = range;

        // 将行列信息编码到点的强度值中
        thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

        // 计算一维索引并保存点云信息
        size_t index = columnIdn + rowIdn * HORIZONTAL_SCAN;
        fullCloud->points[index] = thisPoint;
        // 将对应点的距离信息保存为强度值
        fullInfoCloud->points[index] = thisPoint;
        fullInfoCloud->points[index].intensity = range;
    }

void CloudInterpolation::findStartEndAngle() {
    if(groundCloud->empty()) return;                        // 检查点云是否为空
    
    auto& firstPoint = groundCloud->points.front();         // 获取第一个点
    startOrientation = -std::atan2(firstPoint.y, firstPoint.x); // 计算起始角度

    auto& lastPoint = groundCloud->points.back();           // 获取最后一个点
    endOrientation = -std::atan2(lastPoint.y, lastPoint.x) + 2 * M_PI; // 计算结束角度

    if (endOrientation - startOrientation > 3 * M_PI) {     // 处理角度环绕情况
        endOrientation -= 2 * M_PI;
    } else if (endOrientation - startOrientation < M_PI) {
        endOrientation += 2 * M_PI;
    }
    
    orientationDiff = endOrientation - startOrientation;    // 计算角度差
}

void CloudInterpolation::groundRemoval() {
    groundMatrix.setZero();                                 // 初始化地面矩阵
    
    patched_ground->clear();                               // 清空补丁地面点云
    patched_ground_edge->clear();                          // 清空补丁地面边缘点云

    for (size_t j = 0; j < HORIZONTAL_SCAN; ++j) {         // 遍历每一列
        size_t ring_edge = 0;                              // 初始化环边缘索引
        size_t closest_ring_edge = groundScanIndex;        // 初始化最近环边缘索引
        bool do_patch = false;                             // 初始化补丁标志

        for (size_t i = 0; i < groundScanIndex; ++i) {     // 遍历每一行直到地面扫描线
            size_t lowerInd = j + (i)*HORIZONTAL_SCAN;     // 计算下方点索引
            size_t upperInd = j + (i + 1) * HORIZONTAL_SCAN; // 计算上方点索引

            if (fullCloud->points[lowerInd].intensity == -1 ||
                fullCloud->points[upperInd].intensity == -1) { // 检查点是否有效
                groundMatrix(i, j) = -1;                    // 标记无效点
                continue;
            }

            float dX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x; // 计算X方向差值
            float dY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y; // 计算Y方向差值
            float dZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z; // 计算Z方向差值

            float vertical_angle = std::atan2(dZ, sqrt(dX * dX + dY * dY + dZ * dZ)); // 计算垂直角度

            if ((vertical_angle - sensorMountAngle) <= 10 * DEG_TO_RAD) { // 判断是否为地面点
                groundMatrix(i, j) = 1;                     // 标记为地面点
                groundMatrix(i + 1, j) = 1;                 // 标记相邻点为地面点

                if(i < closest_ring_edge && i != groundScanIndex) // 更新最近环边缘
                    closest_ring_edge = i;

                float ds = sqrt(dX*dX + dY*dY + dZ*dZ);    // 计算点间距离
                
                if(ds < distance_for_patch_between_rings) { // 判断是否需要补丁
                    ring_edge = i+1;                        // 更新环边缘
                    float dt = 1.0/(ds/0.1+1);             // 计算插值步长
                    for(float t=0; t<=1.0; t+=dt) {        // 插值生成补丁点
                        pcl::PointXYZI a_pt;               // 创建新点
                        a_pt.intensity = 0.0;              // 设置强度值
                        a_pt.x = fullCloud->points[lowerInd].x + dX*t; // 计算X坐标
                        a_pt.y = fullCloud->points[lowerInd].y + dY*t; // 计算Y坐标
                        a_pt.z = fullCloud->points[lowerInd].z + dZ*t; // 计算Z坐标
                        patched_ground->push_back(a_pt);    // 添加到补丁点云
                    }
                    do_patch = true;                       // 设置补丁标志
                }
            }
        }

        size_t ringEdgeInd = j + ring_edge*HORIZONTAL_SCAN; // 计算环边缘点索引
        pcl::PointXYZI a_pt;                               // 创建边缘点
        a_pt.x = fullCloud->points[ringEdgeInd].x;         // 设置X坐标
        a_pt.y = fullCloud->points[ringEdgeInd].y;         // 设置Y坐标
        a_pt.z = fullCloud->points[ringEdgeInd].z;         // 设置Z坐标
        a_pt.intensity = 100;                              // 设置强度值
        patched_ground_edge->push_back(a_pt);              // 添加到边缘点云

        if(do_patch && first_frame_count_ < 5 && closest_ring_edge < groundScanIndex) { // 判断是否需要额外补丁
            size_t closest_ring_edgeInd = j + (closest_ring_edge)*HORIZONTAL_SCAN; // 计算最近环边缘点索引
            float dXf = -fullCloud->points[closest_ring_edgeInd].x; // 计算X方向差值
            float dYf = -fullCloud->points[closest_ring_edgeInd].y; // 计算Y方向差值
            float dZf = -fullCloud->points[closest_ring_edgeInd].z; // 计算Z方向差值

            for(float t=0; t<=1.0; t+=0.05) {              // 生成额外补丁点
                pcl::PointXYZI a_ptf;                      // 创建新点
                a_ptf.intensity = 0.0;                     // 设置强度值
                a_ptf.x = fullCloud->points[closest_ring_edgeInd].x + dXf*t; // 计算X坐标
                a_ptf.y = fullCloud->points[closest_ring_edgeInd].y + dYf*t; // 计算Y坐标
                a_ptf.z = fullCloud->points[closest_ring_edgeInd].z;         // 计算Z坐标
                patched_ground->push_back(a_ptf);          // 添加到补丁点云
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr processedGroundCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 创建处理后的地面点云
    for (size_t i = 0; i <= groundScanIndex; ++i) {        // 遍历地面扫描线
        for (size_t j = 0; j < HORIZONTAL_SCAN; ++j) {     // 遍历每一列
            if (groundMatrix(i, j) == 1) {                 // 检查是否为地面点
                processedGroundCloud->push_back(fullCloud->points[j + i * HORIZONTAL_SCAN]); // 添加地面点
            }
        }
    }

    *processedGroundCloud += *patched_ground;              // 合并补丁点云
    *processedGroundCloud += *patched_ground_edge;         // 合并边缘点云

    groundCloud = processedGroundCloud;                    // 更新地面点云
}

void CloudInterpolation::setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    groundCloud = cloud;                                   // 设置输入点云
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CloudInterpolation::process() {
    if (!groundCloud || groundCloud->empty()) {            // 检查输入点云是否有效
        return nullptr;
    }

    findStartEndAngle();                                  // 查找起始结束角度
    projectPointCloud();                                  // 投影点云
    groundRemoval();                                      // 地面点提取
    first_frame_count_++;                                 // 更新帧计数

    return groundCloud;                                   // 返回处理后的地面点云
}

} // namespace terrain_analysis