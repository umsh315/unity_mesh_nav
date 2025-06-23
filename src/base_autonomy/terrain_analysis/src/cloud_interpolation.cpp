#include "terrain_analysis/cloud_interpolation.hpp"

namespace terrain_analysis {

CloudInterpolation::CloudInterpolation() {
    groundCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CloudInterpolation::processCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    *groundCloud = *inputCloud;
    
    if (!groundCloud->empty()) {
        initializeGrid(groundCloud);
        projectToGrid(groundCloud);
        interpolateGaps();
        smoothInterpolatedCloud();
    }
    
    return groundCloud;
}

void CloudInterpolation::initializeGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (cloud->empty()) return;
    
    gridMinX = gridMaxX = cloud->points[0].x;
    gridMinY = gridMaxY = cloud->points[0].y;
    
    for (const auto& point : cloud->points) {
        gridMinX = std::min(gridMinX, point.x);
        gridMaxX = std::max(gridMaxX, point.x);
        gridMinY = std::min(gridMinY, point.y);
        gridMaxY = std::max(gridMaxY, point.y);
    }

    gridMinX -= GRID_RESOLUTION;
    gridMinY -= GRID_RESOLUTION;
    gridMaxX += GRID_RESOLUTION;
    gridMaxY += GRID_RESOLUTION;

    gridCols = ceil((gridMaxX - gridMinX) / GRID_RESOLUTION);
    gridRows = ceil((gridMaxY - gridMinY) / GRID_RESOLUTION);

    heightGrid.resize(gridRows, std::vector<GridCell>(gridCols));
}

void CloudInterpolation::projectToGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    for (const auto& point : cloud->points) {
        int col = (point.x - gridMinX) / GRID_RESOLUTION;
        int row = (point.y - gridMinY) / GRID_RESOLUTION;

        if (row >= 0 && row < gridRows && col >= 0 && col < gridCols) {
            heightGrid[row][col].hasPoint = true;
            heightGrid[row][col].height = point.z;
            heightGrid[row][col].intensity = point.intensity;
        }
    }
}

void CloudInterpolation::interpolateGaps() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr interpolatedCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree(GRID_RESOLUTION);
    octree.setInputCloud(groundCloud);
    octree.addPointsFromInputCloud();

    for (int row = 0; row < gridRows; row++) {
        for (int col = 0; col < gridCols; col++) {
            if (!heightGrid[row][col].hasPoint) {
                float x = gridMinX + (col + 0.5f) * GRID_RESOLUTION;
                float y = gridMinY + (row + 0.5f) * GRID_RESOLUTION;

                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
                pcl::PointXYZI searchPoint;
                searchPoint.x = x;
                searchPoint.y = y;
                searchPoint.z = 0;

                if (octree.radiusSearch(searchPoint, SEARCH_RADIUS, pointIdxRadiusSearch, pointRadiusSquaredDistance) >= MIN_NEIGHBORS) {
                    float weightedSumZ = 0;
                    float weightedSumI = 0;
                    float weightSum = 0;

                    for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
                        float distance = sqrt(pointRadiusSquaredDistance[i]);
                        if (distance < 0.0001f) distance = 0.0001f;
                        float weight = 1.0f / distance;

                        weightedSumZ += groundCloud->points[pointIdxRadiusSearch[i]].z * weight;
                        weightedSumI += groundCloud->points[pointIdxRadiusSearch[i]].intensity * weight;
                        weightSum += weight;
                    }

                    pcl::PointXYZI interpolatedPoint;
                    interpolatedPoint.x = x;
                    interpolatedPoint.y = y;
                    interpolatedPoint.z = weightedSumZ / weightSum;
                    interpolatedPoint.intensity = weightedSumI / weightSum;

                    bool validHeight = true;
                    for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
                        if (fabs(interpolatedPoint.z - groundCloud->points[pointIdxRadiusSearch[i]].z) > MAX_HEIGHT_DIFF) {
                            validHeight = false;
                            break;
                        }
                    }

                    if (validHeight) {
                        interpolatedCloud->push_back(interpolatedPoint);
                    }
                }
            }
        }
    }

    *groundCloud += *interpolatedCloud;
}

void CloudInterpolation::smoothInterpolatedCloud() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree(GRID_RESOLUTION * 0.5f);
    octree.setInputCloud(groundCloud);
    octree.addPointsFromInputCloud();

    for (const auto& point : groundCloud->points) {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        
        if (octree.radiusSearch(point, GRID_RESOLUTION, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 1) {
            pcl::PointXYZI smoothedPoint = point;
            float totalWeight = 0;
            float weightedSumZ = 0;

            for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
                float weight = exp(-pointRadiusSquaredDistance[i] / (GRID_RESOLUTION * GRID_RESOLUTION));
                weightedSumZ += groundCloud->points[pointIdxRadiusSearch[i]].z * weight;
                totalWeight += weight;
            }

            smoothedPoint.z = weightedSumZ / totalWeight;
            smoothedCloud->push_back(smoothedPoint);
        } else {
            smoothedCloud->push_back(point);
        }
    }

    groundCloud = smoothedCloud;
}

} // namespace terrain_analysis 