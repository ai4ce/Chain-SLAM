// Definitions of the shared global state declared in globals.h.
#include "globals.h"

/*** Time log variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;

float res_last[100000] = {0.0}; // residuals (squared point-to-plane distances)
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

std::mutex mtx_buffer;
std::condition_variable sig_buffer;

std::string root_dir = ROOT_DIR;
std::string log_subfolder = "";
std::string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double laser_point_cov = 0.001;  // LiDAR measurement noise: higher = trust LiDAR less
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool point_selected_surf[100000] = {0}; // whether each point is a planar feature
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
std::atomic<bool> imu_init_complete(false);
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

std::vector<std::vector<int>> pointSearchInd_surf;
std::vector<BoxPointType> cub_needrm; // ikd-tree boxes to remove from the local map
std::vector<PointVector> Nearest_Points;
std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
std::deque<double> time_buffer;               // lidar frame timestamps
std::deque<size_t> frame_index_buffer;        // original frame indices
size_t lidar_frame_counter = 0;
std::deque<PointCloudXYZI::Ptr> lidar_buffer; // preprocessed lidar feature clouds
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());  // downsampled current frame, lidar frame
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI()); // downsampled current frame, world frame
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1)); // matched-map-point plane params, world frame
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;                            // points to remove from the ikd-tree map

pcl::VoxelGrid<PointType> downSizeFilterSurf; // per-frame downsampling voxel grid
pcl::VoxelGrid<PointType> downSizeFilterMap;  // unused

KD_TREE ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d); // T lidar to imu (imu = r * lidar + t)
M3D Lidar_R_wrt_IMU(Eye3d);  // R lidar to imu (imu = r * lidar + t)

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf; // state dim, noise dim, input
state_ikfom state_point;
vect3 pos_lid; // lidar position in world frame

// Reference odometry initialization
bool use_reference_init = false;
std::string reference_odom_topic = "";
std::mutex mtx_reference_init;
bool ref_init_applied = false;
double first_processed_lidar_time = -1.0;  // first LiDAR frame time with valid reference; ref_path only includes poses >= this
size_t loadedKeyframeCount = 0; // keyframes loaded from map (for loop closure only)
bool mapMergeSuccessful = false;  // true after first successful GPS RANSAC merge
Eigen::Affine3f gpsRansacTransform;  // RANSAC transform from merge; applied to new poses added after merge
bool gpsMapMergeRebuildDone = false;  // true after post-merge rebuild; prevents rebuild on regular loop closures
std::vector<SceneRange> sceneIndexMap; // which keyframe indices belong to which scene
Eigen::Vector3d ref_current_position(0.0, 0.0, 0.0);
Eigen::Quaterniond ref_current_orientation(1.0, 0.0, 0.0, 0.0);
bool ref_current_valid = false;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

std::shared_ptr<Preprocess> p_pre(new Preprocess());
std::shared_ptr<ImuProcess> p_imu(new ImuProcess());

/*** Back end ***/
std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames; // corner points per keyframe (downsampled)
std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;   // surface points per keyframe (downsampled)

pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D(new pcl::PointCloud<PointType>());         // keyframe positions
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); // keyframe 6D poses
pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());

pcl::PointCloud<PointTypePose>::Ptr fastlio_unoptimized_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); // unoptimized FAST-LIO poses
pcl::PointCloud<PointTypePose>::Ptr gnss_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); // GNSS trajectory
std::vector<Eigen::Vector3d> gnss_raw_wgs84;  // (lat, lon, alt) parallel to gnss_cloudKeyPoses6D, for UTM conversion

// voxel filter params
float odometrySurfLeafSize;
float mappingCornerLeafSize;
float mappingSurfLeafSize;

float z_tollerance;
float rotation_tollerance;

// CPU params
int numberOfCores = 4;
double mappingProcessInterval;

/*** Loop closure ***/
bool startFlag = true;
bool loopClosureEnableFlag;
float loopClosureFrequency; // loop detection frequency
float historyKeyframeSearchRadius;   // radius KD-tree search radius for loop detection
bool loopClosureDistance2D;          // if true, use only x,y for distance (ignore z)
float historyKeyframeSearchTimeDiff; // min time gap between keyframes
int historyKeyframeSearchNum;        // number of keyframes merged into a submap
float historyKeyframeMSEThreshold;   // ICP MSE threshold
int postMergeClosureSkipFrames = 0;  // skip n keyframes after each successful post-merge closure
bool addChainedClosures = true;  // when a new closure matches a loaded keyframe, also add Between() to all connected loaded keyframes (BFS)
bool potentialLoopFlag = false;

ros::Publisher pubHistoryKeyFrames; // loop history keyframe submap
ros::Publisher pubIcpKeyFrames;
ros::Publisher pubRecentKeyFrames;
ros::Publisher pubRecentKeyFrame;
ros::Publisher pubCloudRegisteredRaw;
ros::Publisher pubLoopConstraintEdge;

bool aLoopIsClosed = false;
std::map<int, std::set<int>> loopIndexContainer; // cur -> set of keyframes cur is loop-closed to (direct + chained)
std::set<std::pair<int, int>> directLoopClosureEdges; // canonical (min,max) edges added as direct closure (viz color)
std::map<std::pair<int, int>, double> loopClosureVariance; // canonical (min,max) -> isotropic variance (save/load)
std::set<std::pair<int, int>> loopClosurePairsAdded;  // canonical (min,max) to prevent duplicate factors
std::vector<std::pair<int, int>> loopIndexQueue;
std::vector<gtsam::Pose3> loopPoseQueue;
std::vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
std::deque<std_msgs::Float64MultiArray> loopInfoVec;

nav_msgs::Path globalPath;

// Local keyframe map cloud and its KD-tree, used for scan-to-map nearest-neighbor search
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<PointType>());

// Downsampling filters
pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterICP;
pcl::VoxelGrid<PointType> downSizeFilterGpsMerge;  // GPS map merge: gpsMapMergeVoxelSize
pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // surrounding key poses for scan-to-map optimization
pcl::VoxelGrid<PointType> downSizeFilterKeyframeStorage;    // downsample keyframes before storing

// Keyframe storage density: finest among consumers.
float keyframeStorageLeafSize;
bool keyframeStorageDownsampleEn = true;  // false: store at full resolution (more memory, finer loop closure)

float transformTobeMapped[6]; // current frame pose in world frame

std::mutex mtx;
std::mutex mtx_map_merge;  // pause main thread from RANSAC apply through ISAM rebuild
std::mutex mtxLoopInfo;

// Per-frame runtime and memory for save_pose performance report
std::mutex mtx_perf;
std::vector<double> g_frame_timestamps;
std::vector<double> g_frame_runtimes_ms;
std::vector<double> g_frame_frontend_ms;
std::vector<double> g_frame_backend_ms;
std::vector<double> g_frame_mem_mb;

// Surrounding map
float surroundingkeyframeAddingDistThreshold;  // distance threshold for adding a keyframe
float surroundingkeyframeAddingAngleThreshold; // angle threshold for adding a keyframe
float surroundingKeyframeDensity;
float surroundingKeyframeSearchRadius;

// gtsam
gtsam::NonlinearFactorGraph gtSAMgraph;
gtsam::Values initialEstimate;
gtsam::Values optimizedEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;
Eigen::MatrixXd poseCovariance;

ros::Publisher pubLaserCloudSurround;
ros::Publisher pubOptimizedGlobalMap;            // publishes the final optimized map

bool    recontructKdTree = false;
int updateKdtreeCount = 0 ;        // rebuild every N updates
bool visulize_IkdtreeMap = false;            // visualize ikd-tree submap

// gnss
double last_timestamp_gnss = -1.0 ;
std::deque<nav_msgs::Odometry> gnss_buffer;
geometry_msgs::PoseStamped msg_gnss_pose;

// GPS-based map merge (align new run to loaded map when GPS is close)
std::vector<Eigen::Vector3d> loadedKeyframeGnssPositions;  // GNSS UTM per loaded keyframe, invalid = (1e10,1e10,1e10)
std::vector<Eigen::Vector3d> loadedKeyframeGnssWgs84;     // WGS84 (lat,lon,alt) for re-saving loaded keyframes
pcl::PointCloud<PointType>::Ptr loadedGnssCloud(new pcl::PointCloud<PointType>());  // GNSS positions for KD-tree radius search
pcl::KdTreeFLANN<PointType>::Ptr kdtreeLoadedGnss(new pcl::KdTreeFLANN<PointType>());
bool gpsMapMergeEnabled = true;
float gpsMapMergeDistanceThreshold = 5.0f;
float gpsMapMergeRansacFitnessThreshold = 0.01f;  // min RANSAC fitness (inliers/target_points) to succeed [0,1]
float gpsMapMergeRansacMinRotationRad = 0.02f;   // reject near-identity: when rot_diff < this AND trans < min_translation
float gpsMapMergeRansacMinTranslationM = 1.0f;    // reject near-identity: when trans_norm < this AND rot < min_rotation
float gpsMapMergeRansacInlierDist = 0.5f;   // RANSAC inlier distance (m)
int gpsMapMergeRansacMaxIterations = 100000; // RANSAC max iterations
float gpsMapMergePostRebuildWaitSec = 0.5f;  // wait after ISAM2 rebuild before double-check (lets KD-trees settle)
float gpsMapMergeVoxelSize = 1.0f;           // voxel downsample before FPFH (m)
float gpsMapMergeFpfhRadius = 0.5f;          // FPFH search radius (m)
int gpsMapMergeFpfhMaxNn = 0;                // FPFH max neighbors (0=use radius only; 100 to match Open3D)
float gpsMapMergeNormalRadius = 0.5f;       // normal estimation radius (m)
bool gpsMapMergeMutualFilter = true;         // mutual NN in feature matching
int lastGpsMergeAttemptedKeyframe = -1;  // skip re-attempting same keyframe

// 2D loop closure (GNSS 2D search over ALL keyframes when 3D fails)
bool loopClosure2DEnabled = false;
float loopClosure2DDistanceThreshold = 10.0f;

// reference odometry for XY factor
std::deque<nav_msgs::Odometry> reference_odom_buffer;
nav_msgs::Odometry current_frame_ref_odom;
bool current_frame_ref_odom_valid = false;
std::string gnss_topic ;
bool useImuHeadingInitialization;
bool useGpsElevation;             // use GPS elevation in optimization
bool useGnssOptimization = false; // enable GNSS optimization
bool useXYFactor = false;         // enable XY factor optimization
float gpsCovThreshold;          // covariance threshold for GPS heading and elevation
float poseCovThreshold;       // pose covariance threshold from isam2
// XY factor noise model variances (roll, pitch, yaw, x, y, z)
double xy_factor_noise_roll = 1e-6;
double xy_factor_noise_pitch = 1e-6;
double xy_factor_noise_yaw = 1e-6;
double xy_factor_noise_x = 1e-4;
double xy_factor_noise_y = 1e-4;
double xy_factor_noise_z = 1e2;
double loop_closure_noise_scale = 1.0;  // loop closure noise scale (lower = stronger constraint)

M3D Gnss_R_wrt_Lidar(Eye3d) ;         // GNSS-to-lidar extrinsic rotation
V3D Gnss_T_wrt_Lidar(Zero3d);
bool gnss_inited = false ;            // whether GNSS initialization is complete
std::shared_ptr<GnssProcess> p_gnss(new GnssProcess());
GnssProcess gnss_data;
ros::Publisher pubGnssPath ;
nav_msgs::Path gps_path ;
ros::Publisher pubRefPath ;
nav_msgs::Path ref_path ;
std::vector<ros::Publisher> pubLoadedScenePaths; // per-scene path publishers
std::vector<nav_msgs::Path> loadedScenePaths; // per-scene paths
std::vector<double>       extrinT_Gnss2Lidar(3, 0.0);
std::vector<double>       extrinR_Gnss2Lidar(9, 0.0);

// global map visualization
float globalMapVisualizationSearchRadius;
float globalMapVisualizationPoseDensity;
float globalMapVisualizationLeafSize;

// Local map / scan-timing state
BoxPointType LocalMap_Points;   // bounding box corners of the local map in the ikd-tree
bool Localmap_Initialized = false;
double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;  // whether lidar/imu time compensation has been done
double lidar_mean_scantime = 0.0;
int scan_num = 0;
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());  // accumulated world-frame cloud for PCD saving

// save/load map services
ros::ServiceServer srvSaveMap;
ros::ServiceServer srvLoadMap;
ros::ServiceServer srvSavePose;
bool savePCD;               // whether to save the map
std::string savePCDDirectory;
