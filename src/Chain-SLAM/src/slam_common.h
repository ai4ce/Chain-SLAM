#pragma once

// -----------------------------------------------------------------------------
// Umbrella header for the FAST-LIO-SAM node modules.
//
// Carries the full third-party include environment (ROS, PCL, GTSAM, Open3D),
// the shared global state, the pure conversion/helper headers, and forward
// declarations of every function that was split out of the original monolithic
// laserMapping.cpp. Each module .cpp includes just this header so all
// translation units see an identical set of types and declarations.
// -----------------------------------------------------------------------------

#include <omp.h>
#include <mutex>
#include <set>
#include <atomic>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <libgen.h>
#include <limits.h>
#include <algorithm>
#include <limits>
#include <functional>
#include <sstream>
#include <iomanip>

#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <livox_ros_driver/CustomMsg.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>

#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/KDTreeSearchParam.h>
#include <open3d/pipelines/registration/Feature.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/CorrespondenceChecker.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "preprocess.h"
#include "common_lib.h"
#include "IMU_Processing.hpp"
#include "GNSS_Processing.hpp"
#include <ikd-Tree/ikd_Tree.h>

#include "fast_lio_sam/save_map.h"
#include "fast_lio_sam/save_pose.h"
#include "fast_lio_sam/load_map.h"

#include "globals.h"
#include "geometry_conversions.hpp"
#include "slam_helpers.h"

// Simple 6-DOF pose used by the KITTI-format trajectory writer.
struct pose
{
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
};

// =============================================================================
// Function declarations (definitions live in the module .cpp files).
// =============================================================================

// --- ros_publishers ---
void updatePath(const PointTypePose &pose_in, nav_msgs::Path &path = globalPath);
sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame);
void visualizeLoopClosure();
void publish_frame_world(const ros::Publisher &pubLaserCloudFull);
void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body);
void publish_effect_world(const ros::Publisher &pubLaserCloudEffect);
void publish_map(const ros::Publisher &pubLaserCloudMap);
void publish_odometry(const ros::Publisher &pubOdomAftMapped);
void publish_path(const ros::Publisher pubPath);
void publish_path_update(const ros::Publisher pubPath);
void publish_gnss_path(const ros::Publisher pubPath);
void publish_ref_path(const ros::Publisher pubPath);
void publish_loaded_path();
void publishGlobalMap();

// --- lio_frontend ---
pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);
void getCurPose(state_ikfom cur_state);
void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s);
void pointBodyToWorld(PointType const *const pi, PointType *const po);
void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
void test_RGBpointBodyToWorld(PointType const *const pi, PointType *const po, Eigen::Vector3d pos, Eigen::Matrix3d rotation);
void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

// --- ikd_map ---
void points_cache_collect();
void lasermap_fov_segment();
void map_incremental();
void recontructIKdTree();

// --- sensor_callbacks ---
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg);
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
void gnss_cbk(const sensor_msgs::NavSatFixConstPtr& msg_in);
void reference_odom_cbk(const nav_msgs::OdometryConstPtr& msg_in);
bool sync_packages(MeasureGroup &meas);

// --- pose_graph ---
bool saveFrame();
void addOdomFactor();
bool rebuildIsamAfterRansac();
void addLoopFactor();
void matchReferenceOdometry(double lidar_time);
void addXYFactor();
void addGPSFactor();
void saveKeyFramesAndFactor();
void correctPoses();

// --- loop_closure ---
// 2D GNSS closure: translation-only init (from pose difference) + ICP. No RANSAC.
bool try2DMatchTranslationOnly(int loopKeyCur, int loopKeyPre,
    const pcl::PointCloud<PointType>::Ptr& cureKeyframeCloud,
    const pcl::PointCloud<PointType>::Ptr& prevKeyframeCloud,
    Eigen::Matrix4f* out_icpTransform, float* out_fitnessScore);
void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum, float voxelLeafSize = 0.0f);
bool detectLoopClosureDistance(int *latestID, int *closestID);
bool perform2DLoopClosure(int* out_loopKeyCur, int* out_loopKeyPre, Eigen::Matrix4f* out_icpTransform, float* out_fitnessScore);
void performLoopClosure();
void loopClosureThread();

// --- gps_map_merge ---
int doubleCheckPoseLoopClosuresAfterMerge();
bool performGpsMapMerge(int* out_loopKeyCur, int* out_loopKeyPre, std::unique_lock<std::mutex>* lock_map_merge);

// --- map_io_services ---
std::string getWorkspaceRoot();
std::string resolvePath(const std::string& destination, const std::string& defaultPath = "output");
bool CreateFile(std::ofstream& ofs, std::string file_path);
void WriteText(std::ofstream& ofs, pose data);
bool savePoseService(fast_lio_sam::save_poseRequest& req, fast_lio_sam::save_poseResponse& res);
bool saveMapService(fast_lio_sam::save_mapRequest& req, fast_lio_sam::save_mapResponse& res);
bool loadMapService(fast_lio_sam::load_mapRequest& req, fast_lio_sam::load_mapResponse& res);

// --- lifecycle (stay in laserMapping.cpp) ---
void SigHandle(int sig);
void dump_lio_state_to_log(FILE *fp);

// =============================================================================
// Template function definitions (must be visible to all callers).
// =============================================================================

// Transform a point from the lidar body frame to the world frame (Eigen variant).
template <typename T>
void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

// Fill a pose-stamped message from the current EKF state.
template <typename T>
void set_posestamp(T &out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}
