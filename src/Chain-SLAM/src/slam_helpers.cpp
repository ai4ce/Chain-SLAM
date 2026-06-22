#include "slam_helpers.h"

std::shared_ptr<open3d::geometry::PointCloud> pclToOpen3D(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud)
{
    auto o3d_cloud = std::make_shared<open3d::geometry::PointCloud>();
    o3d_cloud->points_.resize(pcl_cloud->size());
    for (size_t i = 0; i < pcl_cloud->size(); ++i) {
        o3d_cloud->points_[i] = Eigen::Vector3d((*pcl_cloud)[i].x, (*pcl_cloud)[i].y, (*pcl_cloud)[i].z);
    }
    return o3d_cloud;
}
