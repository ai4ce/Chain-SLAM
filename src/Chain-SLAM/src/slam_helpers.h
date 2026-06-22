#pragma once

// -----------------------------------------------------------------------------
// Small shared helpers factored out of laserMapping.cpp to remove duplication.
// -----------------------------------------------------------------------------

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <open3d/geometry/PointCloud.h>

#include "common_lib.h"   // PointType

// Apply the standard loop-closure ICP configuration used across the node.
inline void configureLoopIcp(pcl::IterativeClosestPoint<PointType, PointType>& icp)
{
    icp.setMaxCorrespondenceDistance(150);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
}

// Convert a PCL PointCloud<PointXYZ> to an Open3D point cloud.
std::shared_ptr<open3d::geometry::PointCloud> pclToOpen3D(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);
