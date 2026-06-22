#pragma once

// -----------------------------------------------------------------------------
// Pure pose/geometry conversion helpers.
//
// These functions depend only on their arguments (no shared global state), so
// they live in a header-only unit that any module can include.
// -----------------------------------------------------------------------------

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>

#include "common_lib.h"   // PointType, PointCloudXYZI
#include "preprocess.h"   // PointTypePose

// PointTypePose -> gtsam::Pose3
inline gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

// float[6] {roll,pitch,yaw,x,y,z} -> gtsam::Pose3
inline gtsam::Pose3 trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

// PointTypePose -> Eigen::Affine3f
inline Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

// float[6] {roll,pitch,yaw,x,y,z} -> Eigen::Affine3f
inline Eigen::Affine3f trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

// float[6] {roll,pitch,yaw,x,y,z} -> PointTypePose
inline PointTypePose trans2PointTypePose(float transformIn[])
{
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];
    return thisPose6D;
}

// Distance from a point to the coordinate origin
inline float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

// Distance between two points
inline float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

// Euler angles (rad) -> normalized quaternion
inline Eigen::Quaterniond EulerToQuat(float roll_, float pitch_, float yaw_)
{
    Eigen::Quaterniond q;  // quaternions q and -q are equal
    Eigen::AngleAxisd roll(double(roll_), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(double(pitch_), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(double(yaw_), Eigen::Vector3d::UnitZ());
    q = yaw * pitch * roll;
    q.normalize();
    return q;
}

// Build a 6-DOF diagonal Gaussian noise model from per-axis variances
// (order: roll, pitch, yaw, x, y, z).
inline gtsam::noiseModel::Diagonal::shared_ptr makeDiagonalNoise6(
    double roll, double pitch, double yaw, double x, double y, double z)
{
    gtsam::Vector Vector6(6);
    Vector6 << roll, pitch, yaw, x, y, z;
    return gtsam::noiseModel::Diagonal::Variances(Vector6);
}
