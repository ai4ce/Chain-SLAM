#ifndef FAST_LIO_SAM_GLOBALS_H
#define FAST_LIO_SAM_GLOBALS_H

// Shared global state for the FAST-LIO-SAM node. Definitions live in globals.cpp
// and are declared extern here so the node's functions can be split across module
// translation units without rewriting every access.

#include <mutex>
#include <set>
#include <map>
#include <atomic>
#include <condition_variable>
#include <vector>
#include <deque>
#include <string>

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/linear/NoiseModel.h>

#include "common_lib.h"
#include "preprocess.h"
#include "IMU_Processing.hpp"
#include "GNSS_Processing.hpp"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME (0.1)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

// Sentinel for "no valid GNSS UTM" entries (parallel to keyframe arrays).
constexpr double GNSS_INVALID_MARKER = 1e10;

// Scene index tracking: (start_index, end_index, scene_name) per loaded scene.
struct SceneRange {
    int start_idx;
    int end_idx;
    std::string scene_name;
};

// =============================================================================
// Global state (definitions in globals.cpp)
// =============================================================================

/*** Time log variables ***/
extern double kdtree_incremental_time, kdtree_search_time, kdtree_delete_time;
extern double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
extern double match_time, solve_time, solve_const_H_time;
extern int kdtree_size_st, kdtree_size_end, add_point_size, kdtree_delete_counter;
extern bool runtime_pos_log, pcd_save_en, time_sync_en, extrinsic_est_en, path_en;

extern float res_last[100000];
extern float DET_RANGE;
extern const float MOV_THRESHOLD;

extern std::mutex mtx_buffer;
extern std::condition_variable sig_buffer;

extern std::string root_dir;
extern std::string log_subfolder;
extern std::string map_file_path, lid_topic, imu_topic;

extern double res_mean_last, total_residual;
extern double last_timestamp_lidar, last_timestamp_imu;
extern double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
extern double laser_point_cov;
extern double filter_size_corner_min, filter_size_surf_min, filter_size_map_min, fov_deg;
extern double cube_len, HALF_FOV_COS, FOV_DEG, total_distance, lidar_end_time, first_lidar_time;
extern int effct_feat_num, time_log_counter, scan_count, publish_count;
extern int iterCount, feats_down_size, NUM_MAX_ITERATIONS, laserCloudValidNum, pcd_save_interval, pcd_index;
extern bool point_selected_surf[100000];
extern bool lidar_pushed, flg_first_scan, flg_exit, flg_EKF_inited;
extern std::atomic<bool> imu_init_complete;
extern bool scan_pub_en, dense_pub_en, scan_body_pub_en;

extern std::vector<std::vector<int>> pointSearchInd_surf;
extern std::vector<BoxPointType> cub_needrm;
extern std::vector<PointVector> Nearest_Points;
extern std::vector<double> extrinT;
extern std::vector<double> extrinR;
extern std::deque<double> time_buffer;
extern std::deque<size_t> frame_index_buffer;
extern size_t lidar_frame_counter;
extern std::deque<PointCloudXYZI::Ptr> lidar_buffer;
extern std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

extern PointCloudXYZI::Ptr featsFromMap;
extern PointCloudXYZI::Ptr feats_undistort;
extern PointCloudXYZI::Ptr feats_down_body;
extern PointCloudXYZI::Ptr feats_down_world;
extern PointCloudXYZI::Ptr normvec;
extern PointCloudXYZI::Ptr laserCloudOri;
extern PointCloudXYZI::Ptr corr_normvect;
extern PointCloudXYZI::Ptr _featsArray;

extern pcl::VoxelGrid<PointType> downSizeFilterSurf;
extern pcl::VoxelGrid<PointType> downSizeFilterMap;

extern KD_TREE ikdtree;

extern V3F XAxisPoint_body;
extern V3F XAxisPoint_world;
extern V3D euler_cur;
extern V3D position_last;
extern V3D Lidar_T_wrt_IMU;
extern M3D Lidar_R_wrt_IMU;

/*** EKF inputs and output ***/
extern MeasureGroup Measures;
extern esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
extern state_ikfom state_point;
extern vect3 pos_lid;

// Reference odometry initialization
extern bool use_reference_init;
extern std::string reference_odom_topic;
extern std::mutex mtx_reference_init;
extern bool ref_init_applied;
extern double first_processed_lidar_time;
extern size_t loadedKeyframeCount;
extern bool mapMergeSuccessful;
extern Eigen::Affine3f gpsRansacTransform;
extern bool gpsMapMergeRebuildDone;
extern std::vector<SceneRange> sceneIndexMap;
extern Eigen::Vector3d ref_current_position;
extern Eigen::Quaterniond ref_current_orientation;
extern bool ref_current_valid;

extern nav_msgs::Path path;
extern nav_msgs::Odometry odomAftMapped;
extern geometry_msgs::Quaternion geoQuat;
extern geometry_msgs::PoseStamped msg_body_pose;

extern std::shared_ptr<Preprocess> p_pre;
extern std::shared_ptr<ImuProcess> p_imu;

/*** Back end ***/
extern std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
extern std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

extern pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
extern pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
extern pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
extern pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

extern pcl::PointCloud<PointTypePose>::Ptr fastlio_unoptimized_cloudKeyPoses6D;
extern pcl::PointCloud<PointTypePose>::Ptr gnss_cloudKeyPoses6D;
extern std::vector<Eigen::Vector3d> gnss_raw_wgs84;

// voxel filter params
extern float odometrySurfLeafSize;
extern float mappingCornerLeafSize;
extern float mappingSurfLeafSize;

extern float z_tollerance;
extern float rotation_tollerance;

// CPU params
extern int numberOfCores;
extern double mappingProcessInterval;

/*** Loop closure ***/
extern bool startFlag;
extern bool loopClosureEnableFlag;
extern float loopClosureFrequency;
extern float historyKeyframeSearchRadius;
extern bool loopClosureDistance2D;
extern float historyKeyframeSearchTimeDiff;
extern int historyKeyframeSearchNum;
extern float historyKeyframeMSEThreshold;
extern int postMergeClosureSkipFrames;
extern bool addChainedClosures;
extern bool potentialLoopFlag;

extern ros::Publisher pubHistoryKeyFrames;
extern ros::Publisher pubIcpKeyFrames;
extern ros::Publisher pubRecentKeyFrames;
extern ros::Publisher pubRecentKeyFrame;
extern ros::Publisher pubCloudRegisteredRaw;
extern ros::Publisher pubLoopConstraintEdge;

extern bool aLoopIsClosed;
extern std::map<int, std::set<int>> loopIndexContainer;
extern std::set<std::pair<int, int>> directLoopClosureEdges;
extern std::map<std::pair<int, int>, double> loopClosureVariance;
extern std::set<std::pair<int, int>> loopClosurePairsAdded;
extern std::vector<std::pair<int, int>> loopIndexQueue;
extern std::vector<gtsam::Pose3> loopPoseQueue;
extern std::vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
extern std::deque<std_msgs::Float64MultiArray> loopInfoVec;

extern nav_msgs::Path globalPath;

extern pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
extern pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
extern pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
extern pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

extern pcl::VoxelGrid<PointType> downSizeFilterCorner;
extern pcl::VoxelGrid<PointType> downSizeFilterICP;
extern pcl::VoxelGrid<PointType> downSizeFilterGpsMerge;
extern pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;
extern pcl::VoxelGrid<PointType> downSizeFilterKeyframeStorage;

extern float keyframeStorageLeafSize;
extern bool keyframeStorageDownsampleEn;

extern float transformTobeMapped[6];

extern std::mutex mtx;
extern std::mutex mtx_map_merge;
extern std::mutex mtxLoopInfo;

// Per-frame runtime and memory for save_pose performance report
extern std::mutex mtx_perf;
extern std::vector<double> g_frame_timestamps;
extern std::vector<double> g_frame_runtimes_ms;
extern std::vector<double> g_frame_frontend_ms;
extern std::vector<double> g_frame_backend_ms;
extern std::vector<double> g_frame_mem_mb;

// Surrounding map
extern float surroundingkeyframeAddingDistThreshold;
extern float surroundingkeyframeAddingAngleThreshold;
extern float surroundingKeyframeDensity;
extern float surroundingKeyframeSearchRadius;

// gtsam
extern gtsam::NonlinearFactorGraph gtSAMgraph;
extern gtsam::Values initialEstimate;
extern gtsam::Values optimizedEstimate;
extern gtsam::ISAM2 *isam;
extern gtsam::Values isamCurrentEstimate;
extern Eigen::MatrixXd poseCovariance;

extern ros::Publisher pubLaserCloudSurround;
extern ros::Publisher pubOptimizedGlobalMap;

extern bool recontructKdTree;
extern int updateKdtreeCount;
extern bool visulize_IkdtreeMap;

// gnss
extern double last_timestamp_gnss;
extern std::deque<nav_msgs::Odometry> gnss_buffer;
extern geometry_msgs::PoseStamped msg_gnss_pose;

// GPS-based map merge
extern std::vector<Eigen::Vector3d> loadedKeyframeGnssPositions;
extern std::vector<Eigen::Vector3d> loadedKeyframeGnssWgs84;
extern pcl::PointCloud<PointType>::Ptr loadedGnssCloud;
extern pcl::KdTreeFLANN<PointType>::Ptr kdtreeLoadedGnss;
extern bool gpsMapMergeEnabled;
extern float gpsMapMergeDistanceThreshold;
extern float gpsMapMergeRansacFitnessThreshold;
extern float gpsMapMergeRansacMinRotationRad;
extern float gpsMapMergeRansacMinTranslationM;
extern float gpsMapMergeRansacInlierDist;
extern int gpsMapMergeRansacMaxIterations;
extern float gpsMapMergePostRebuildWaitSec;
extern float gpsMapMergeVoxelSize;
extern float gpsMapMergeFpfhRadius;
extern int gpsMapMergeFpfhMaxNn;
extern float gpsMapMergeNormalRadius;
extern bool gpsMapMergeMutualFilter;
extern int lastGpsMergeAttemptedKeyframe;

// 2D loop closure
extern bool loopClosure2DEnabled;
extern float loopClosure2DDistanceThreshold;

// reference odometry for XY factor
extern std::deque<nav_msgs::Odometry> reference_odom_buffer;
extern nav_msgs::Odometry current_frame_ref_odom;
extern bool current_frame_ref_odom_valid;
extern std::string gnss_topic;
extern bool useImuHeadingInitialization;
extern bool useGpsElevation;
extern bool useGnssOptimization;
extern bool useXYFactor;
extern float gpsCovThreshold;
extern float poseCovThreshold;
extern double xy_factor_noise_roll;
extern double xy_factor_noise_pitch;
extern double xy_factor_noise_yaw;
extern double xy_factor_noise_x;
extern double xy_factor_noise_y;
extern double xy_factor_noise_z;
extern double loop_closure_noise_scale;

extern M3D Gnss_R_wrt_Lidar;
extern V3D Gnss_T_wrt_Lidar;
extern bool gnss_inited;
extern std::shared_ptr<GnssProcess> p_gnss;
extern GnssProcess gnss_data;
extern ros::Publisher pubGnssPath;
extern nav_msgs::Path gps_path;
extern ros::Publisher pubRefPath;
extern nav_msgs::Path ref_path;
extern std::vector<ros::Publisher> pubLoadedScenePaths;
extern std::vector<nav_msgs::Path> loadedScenePaths;
extern std::vector<double> extrinT_Gnss2Lidar;
extern std::vector<double> extrinR_Gnss2Lidar;

// global map visualization
extern float globalMapVisualizationSearchRadius;
extern float globalMapVisualizationPoseDensity;
extern float globalMapVisualizationLeafSize;

// Local map / scan-timing state (used by ikd map management and sensor sync)
extern BoxPointType LocalMap_Points;
extern bool Localmap_Initialized;
extern double timediff_lidar_wrt_imu;
extern bool timediff_set_flg;
extern double lidar_mean_scantime;
extern int scan_num;
extern PointCloudXYZI::Ptr pcl_wait_save;

// save/load map services
extern ros::ServiceServer srvSaveMap;
extern ros::ServiceServer srvLoadMap;
extern ros::ServiceServer srvSavePose;
extern bool savePCD;
extern std::string savePCDDirectory;

#endif // FAST_LIO_SAM_GLOBALS_H
