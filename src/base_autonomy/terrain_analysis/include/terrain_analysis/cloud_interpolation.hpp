#ifndef CLOUD_INTERPOLATION_HPP
#define CLOUD_INTERPOLATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <limits>

namespace terrain_analysis {

class CloudInterpolation {
public:
    CloudInterpolation();
    
    // 参数设置接口
    void setLidarParams(int num_vertical_scans,
                       int num_horizontal_scans,
                       int ground_scan_index,
                       float vertical_angle_bottom,
                       float vertical_angle_top,
                       float sensor_mount_angle);
    
    void setProjectionParams(float scan_period,
                           float maximum_detection_range,
                           float minimum_detection_range,
                           float distance_for_patch_between_rings);
    
    // 主要接口
    void setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr process();

private:
    // LeGO-LOAM移植的方法
    void resetParameters();
    void projectPointCloud();
    void findStartEndAngle();
    void groundRemoval();
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud;      // 输入/输出混合点云(原始+补丁)
    pcl::PointCloud<pcl::PointXYZI>::Ptr fullCloud;        // 投影后的完整点云(带行列编码)
    pcl::PointCloud<pcl::PointXYZI>::Ptr fullInfoCloud;    // 投影后的完整点云(带距离信息)
    pcl::PointCloud<pcl::PointXYZI>::Ptr patched_ground;   // 地面补丁点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr patched_ground_edge; // 地面边缘点云
    
    // 矩阵
    Eigen::MatrixXf rangeMatrix;
    Eigen::MatrixXi groundMatrix;
    
    // 激光雷达参数
    int N_SCAN;
    int HORIZONTAL_SCAN;
    int groundScanIndex;
    float ang_bottom;
    float ang_resolution_X;
    float ang_resolution_Y;
    float sensorMountAngle;
    
    // 投影参数
    float scan_period;
    float minimum_range;
    float maximum_range;
    float distance_for_patch_between_rings;
    
    // 扫描参数
    float startOrientation;
    float endOrientation;
    float orientationDiff;
    
    // 处理状态
    int first_frame_count_;
    
    const double DEG_TO_RAD = M_PI / 180.0;
};

} // namespace terrain_analysis

#endif // CLOUD_INTERPOLATION_HPP