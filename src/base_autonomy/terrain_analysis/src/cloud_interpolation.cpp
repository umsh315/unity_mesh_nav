#include "../include/terrain_analysis/cloud_interpolation.hpp"
#include "rclcpp/rclcpp.hpp"  // 添加ROS2日志支持

namespace terrain_analysis {

CloudInterpolation::CloudInterpolation() : 
    groundCloud(new pcl::PointCloud<pcl::PointXYZI>()),
    vehicleX(0), vehicleY(0), vehicleZ(0),
    first_frame_count_(0) {
}

void CloudInterpolation::setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    groundCloud = cloud;
    RCLCPP_INFO(rclcpp::get_logger("cloud_interpolation"), 
                "Input cloud size: %lu", groundCloud->points.size());
}

void CloudInterpolation::setVehiclePosition(float x, float y, float z) {
    vehicleX = x;
    vehicleY = y;
    vehicleZ = z;
    RCLCPP_INFO(rclcpp::get_logger("cloud_interpolation"), 
                "Vehicle position set to: [%.2f, %.2f, %.2f]", x, y, z);
}

void CloudInterpolation::setNoDataArea(float minX, float maxX, float minY, float maxY) {
    gridMinX = minX;
    gridMaxX = maxX;
    gridMinY = minY;
    gridMaxY = maxY;
    RCLCPP_INFO(rclcpp::get_logger("cloud_interpolation"), 
                "No data area set to: [%.2f, %.2f] x [%.2f, %.2f]", 
                minX, maxX, minY, maxY);
}

void CloudInterpolation::setParameters(float voxelSize, int minPoints, float maxElev) {
    GRID_RESOLUTION = voxelSize;
    MIN_NEIGHBORS = 3;  // 固定为较小的值
    MAX_HEIGHT_DIFF = 0.15f;  // 使用固定的合理值，不使用maxElev
    SEARCH_RADIUS = voxelSize * 2.5f;  // 搜索半径设置为网格大小的2.5倍
    
    RCLCPP_INFO(rclcpp::get_logger("cloud_interpolation"), 
                "Parameters set: grid_resolution=%.2f, search_radius=%.2f, min_neighbors=%d, max_height_diff=%.2f",
                GRID_RESOLUTION, SEARCH_RADIUS, MIN_NEIGHBORS, MAX_HEIGHT_DIFF);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CloudInterpolation::interpolate() {
    if (!groundCloud) {
        RCLCPP_WARN(rclcpp::get_logger("cloud_interpolation"), 
                   "Input cloud is null!");
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr interpolatedCloud(new pcl::PointCloud<pcl::PointXYZI>());

    if (first_frame_count_ < 5) {
        RCLCPP_INFO(rclcpp::get_logger("cloud_interpolation"), 
                    "Processing frame %d for interpolation", first_frame_count_);
        patchGroundPoints(interpolatedCloud);
        first_frame_count_++;
    }

    RCLCPP_INFO(rclcpp::get_logger("cloud_interpolation"), 
                "Generated %lu interpolated points", interpolatedCloud->points.size());
    return interpolatedCloud;
}

void CloudInterpolation::patchGroundPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& interpolatedCloud) {
    if (!groundCloud || groundCloud->empty()) {
        RCLCPP_WARN(rclcpp::get_logger("cloud_interpolation"), 
                   "Ground cloud is empty or null!");
        return;
    }

    // 扩大搜索范围，特别是X轴方向
    float extendedMinX = gridMinX - 1.0;  // 向后延伸1米
    float extendedMaxX = gridMaxX + 2.0;  // 向前延伸2米
    float extendedMinY = gridMinY - 0.5;  // 向两侧各延伸0.5米
    float extendedMaxY = gridMaxY + 0.5;

    // 初始化网格
    int gridNumX = ceil((extendedMaxX - extendedMinX) / GRID_RESOLUTION);
    int gridNumY = ceil((extendedMaxY - extendedMinY) / GRID_RESOLUTION);
    
    // 用于存储每个网格的地面高度和点云信息
    struct GridInfo {
        float groundHeight = std::numeric_limits<float>::quiet_NaN();
        float pointHeight = std::numeric_limits<float>::quiet_NaN();
        float intensity = 0.0f;
        bool hasPoint = false;
    };
    std::vector<std::vector<GridInfo>> gridInfo(gridNumX, std::vector<GridInfo>(gridNumY));
    
    // 第一步：将现有点云映射到网格中，记录地面高度和点云信息
    for (const auto& point : groundCloud->points) {
        int gridX = (point.x - vehicleX - extendedMinX) / GRID_RESOLUTION;
        int gridY = (point.y - vehicleY - extendedMinY) / GRID_RESOLUTION;
        
        if (gridX >= 0 && gridX < gridNumX && gridY >= 0 && gridY < gridNumY) {
            auto& cell = gridInfo[gridX][gridY];
            if (!cell.hasPoint || point.z < cell.groundHeight) {
                cell.groundHeight = point.z;
                cell.pointHeight = point.z;
                cell.intensity = point.intensity;
                cell.hasPoint = true;
            }
        }
    }

    // 第二步：从远处向近处进行插值
    for (int i = gridNumX - 1; i >= 0; i--) {
        for (int j = 0; j < gridNumY; j++) {
            float x = extendedMinX + i * GRID_RESOLUTION + vehicleX;
            float y = extendedMinY + j * GRID_RESOLUTION + vehicleY;
            
            float local_x = x - vehicleX;
            float local_y = y - vehicleY;
            
            // 只处理指定区域内的点
            if (local_x >= gridMinX && local_x <= gridMaxX && 
                local_y >= gridMinY && local_y <= gridMaxY) {
                
                if (!gridInfo[i][j].hasPoint) {
                    // 在5x5范围内搜索有效的地面信息
                    std::vector<GridInfo> validNeighbors;
                    for (int dx = -2; dx <= 2; dx++) {
                        for (int dy = -2; dy <= 2; dy++) {
                            int nx = i + dx;
                            int ny = j + dy;
                            if (nx >= 0 && nx < gridNumX && ny >= 0 && ny < gridNumY) {
                                if (gridInfo[nx][ny].hasPoint) {
                                    validNeighbors.push_back(gridInfo[nx][ny]);
                                }
                            }
                        }
                    }
                    
                    // 如果找到有效的邻居信息，进行插值
                    if (!validNeighbors.empty()) {
                        // 找到最低的地面高度
                        float groundHeight = std::numeric_limits<float>::max();
                        float avgIntensity = 0.0f;
                        for (const auto& neighbor : validNeighbors) {
                            groundHeight = std::min(groundHeight, neighbor.groundHeight);
                            avgIntensity += neighbor.intensity;
                        }
                        avgIntensity /= validNeighbors.size();
                        
                        // 在网格中添加一个小型圆形点云
                        float centerX = x;
                        float centerY = y;
                        float radius = GRID_RESOLUTION * 0.4f;
                        int numPoints = 8;
                        
                        // 添加圆心点
                        pcl::PointXYZI centerPoint;
                        centerPoint.x = centerX;
                        centerPoint.y = centerY;
                        centerPoint.z = groundHeight;  // 使用地面高度
                        centerPoint.intensity = avgIntensity;  // 使用邻近点的平均intensity
                        interpolatedCloud->push_back(centerPoint);
                        
                        // 添加圆周上的点
                        for (int k = 0; k < numPoints; k++) {
                            float angle = k * 2 * M_PI / numPoints;
                            pcl::PointXYZI point;
                            point.x = centerX + radius * cos(angle);
                            point.y = centerY + radius * sin(angle);
                            point.z = groundHeight;  // 使用地面高度
                            point.intensity = avgIntensity;  // 使用邻近点的平均intensity
                            interpolatedCloud->push_back(point);
                        }
                        
                        // 更新网格信息，供后续网格参考
                        gridInfo[i][j].groundHeight = groundHeight;
                        gridInfo[i][j].pointHeight = groundHeight;
                        gridInfo[i][j].intensity = avgIntensity;
                        gridInfo[i][j].hasPoint = true;
                    }
                }
            }
        }
    }

    // 对结果进行体素滤波，减少点云密度
    if (!interpolatedCloud->empty()) {
        pcl::VoxelGrid<pcl::PointXYZI> voxelGrid;
        voxelGrid.setLeafSize(0.1f, 0.1f, 0.1f);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>());
        *tempCloud = *interpolatedCloud;
        
        voxelGrid.setInputCloud(tempCloud);
        voxelGrid.filter(*interpolatedCloud);
        
        RCLCPP_INFO(rclcpp::get_logger("cloud_interpolation"), 
                    "Generated %lu interpolated points after filtering", 
                    interpolatedCloud->points.size());
    }
}

} // namespace terrain_analysis 