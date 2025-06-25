#ifndef CLOUD_INTERPOLATION_HPP
#define CLOUD_INTERPOLATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Dense>

namespace terrain_analysis {

class CloudInterpolation {
public:
    CloudInterpolation();
    
    // 主要接口
    void setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void setParams(int num_vertical_scans,
                  int num_horizontal_scans,
                  int ground_scan_index,
                  float vertical_angle_bottom,
                  float vertical_angle_top,
                  float sensor_mount_angle,
                  float scan_period,
                  float segment_theta,
                  float maximum_detection_range,
                  float minimum_detection_range,
                  float distance_for_patch_between_rings);
    pcl::PointCloud<pcl::PointXYZI>::Ptr process();

private:
    // LeGO-LOAM移植的方法
    void projectPointCloud();
    void findStartEndAngle();
    void groundRemoval();
    
    // 点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr fullCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr patched_ground;
    pcl::PointCloud<pcl::PointXYZI>::Ptr patched_ground_edge;
    
    // 矩阵
    Eigen::MatrixXf rangeMatrix;
    Eigen::MatrixXi groundMatrix;
    
    // 参数
    int N_SCAN;
    int HORIZONTAL_SCAN;
    int groundScanIndex;
    float ang_bottom;
    float ang_top;
    float ang_resolution_X;
    float ang_resolution_Y;
    float sensorMountAngle;
    float scan_period;
    float segment_theta;
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