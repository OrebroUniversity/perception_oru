#pragma once

#include <string>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>

namespace ndt_generic {

// Point cloud processing utils... 
void computeDirectionsAndRangesFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Vector3d &origin, std::vector<Eigen::Vector3d> &dirs, std::vector<double> &ranges) {
    ranges.reserve(cloud.size());
    dirs.reserve(cloud.size());
    pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud.points.begin();

    while (it != cloud.points.end()) {
        if(std::isnan(it->x) ||std::isnan(it->y) ||std::isnan(it->z))
        {
            it++;
            continue;
        }
        
        Eigen::Vector3d diff;
        diff << it->x-origin(0), it->y-origin(1), it->z-origin(2);
        ranges.push_back(diff.norm());

        diff.normalize();
        dirs.push_back(diff);

        it++;
    }
}

pcl::PointXYZ eigenToPCLPoint(const Eigen::Vector3d &pt) {
    pcl::PointXYZ p;
    p.x = pt[0]; p.y = pt[1]; p.z = pt[2];
    return p;
}

} // namespace
