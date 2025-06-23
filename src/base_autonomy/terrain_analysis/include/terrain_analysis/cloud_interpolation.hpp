#ifndef CLOUD_INTERPOLATION_HPP
#define CLOUD_INTERPOLATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <limits>
#include <cmath>

namespace terrain_analysis {

struct GridCell {
    bool hasPoint;
    float height;
    float intensity;
    int pointCount;
    
    GridCell() : hasPoint(false), height(0), intensity(0), pointCount(0) {}
};

class CloudInterpolation {
public:
    CloudInterpolation();
    ~CloudInterpolation() = default;

    void setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void setVehiclePosition(float x, float y, float z);
    void setNoDataArea(float minX, float maxX, float minY, float maxY);
    void setParameters(float voxelSize, int minPoints, float maxElev);
    pcl::PointCloud<pcl::PointXYZI>::Ptr interpolate();

private:
    void initializeGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void projectToGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void interpolateGaps();
    void smoothInterpolatedCloud();

    pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud;
    std::vector<std::vector<GridCell>> heightGrid;
    float gridMinX, gridMinY, gridMaxX, gridMaxY;
    int gridRows, gridCols;
    float vehicleX, vehicleY, vehicleZ;

    float GRID_RESOLUTION = 0.2f;
    float SEARCH_RADIUS = 0.5f;
    int MIN_NEIGHBORS = 3;
    float MAX_HEIGHT_DIFF = 0.15f;

    int first_frame_count_;

    // 插值函数
    void patchGroundPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& interpolatedCloud);
};

} // namespace terrain_analysis

#endif // CLOUD_INTERPOLATION_HPP 