#ifndef CLOUD_INTERPOLATION_HPP
#define CLOUD_INTERPOLATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <vector>

namespace terrain_analysis {

struct GridCell {
    bool hasPoint;
    float height;
    float intensity;
    GridCell() : hasPoint(false), height(0), intensity(0) {}
};

class CloudInterpolation {
public:
    CloudInterpolation();
    ~CloudInterpolation() = default;

    pcl::PointCloud<pcl::PointXYZI>::Ptr processCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud);

private:
    void initializeGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void projectToGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void interpolateGaps();
    void smoothInterpolatedCloud();

    pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud;
    std::vector<std::vector<GridCell>> heightGrid;
    float gridMinX, gridMinY, gridMaxX, gridMaxY;
    int gridRows, gridCols;

    const float GRID_RESOLUTION = 0.1f;
    const float SEARCH_RADIUS = 0.3f;
    const int MIN_NEIGHBORS = 3;
    const float MAX_HEIGHT_DIFF = 0.1f;
};

} // namespace terrain_analysis

#endif // CLOUD_INTERPOLATION_HPP 