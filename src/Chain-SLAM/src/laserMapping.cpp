// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "slam_common.h"

#ifdef __linux__
static double getRssMb() {
    std::ifstream s("/proc/self/status");
    std::string line;
    while (s && std::getline(s, line))
        if (line.compare(0, 6, "VmRSS:") == 0) return std::stol(line.substr(6)) / 1024.0;
    return 0;
}
#endif

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    double abs_time = Measures.lidar_beg_time;
    double rel_time = abs_time - first_lidar_time;
    size_t frame_idx = Measures.lidar_frame_idx;
    fprintf(fp, "%zu %lf %lf ", frame_idx, abs_time, rel_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                            // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));    // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));    // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                                 // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));       // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));       // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

int main(int argc, char **argv)
{
    for (int i = 0; i < 6; ++i)
    {
        transformTobeMapped[i] = 0;
    }

    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
    nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
    nh.param<string>("map_file_path", map_file_path, "");
    nh.param<string>("log_subfolder", log_subfolder, "");
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("filter_size_corner", filter_size_corner_min, 0.5);
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("cube_side_length", cube_len, 200);
    nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
    nh.param<double>("mapping/fov_degree", fov_deg, 180);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("mapping/laser_point_cov", laser_point_cov, 0.001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    bool pre_deskewed = false;
    nh.param<bool>("preprocess/pre_deskewed", pre_deskewed, false);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    int max_ini_count = 20;
    nh.param<int>("imuInit/max_ini_count", max_ini_count, 20);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    cout << "p_pre->lidar_type " << p_pre->lidar_type << endl;

    nh.param<float>("odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
    nh.param<float>("mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
    nh.param<float>("mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

    nh.param<float>("z_tollerance", z_tollerance, FLT_MAX);
    nh.param<float>("rotation_tollerance", rotation_tollerance, FLT_MAX);

    nh.param<int>("numberOfCores", numberOfCores, 2);
    nh.param<double>("mappingProcessInterval", mappingProcessInterval, 0.15);

    // save keyframes
    nh.param<float>("surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 20.0);
    nh.param<float>("surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
    nh.param<float>("surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
    nh.param<float>("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

    // loop clousre
    nh.param<bool>("loopClosureEnableFlag", loopClosureEnableFlag, false);
    nh.param<float>("loopClosureFrequency", loopClosureFrequency, 1.0);
    nh.param<double>("loopClosure/noise_scale", loop_closure_noise_scale, 1.0);
    nh.param<bool>("loopClosure/add_chained_closures", addChainedClosures, true);
    nh.param<bool>("loopClosure2D/enabled", loopClosure2DEnabled, false);
    nh.param<float>("loopClosure2D/distance_threshold", loopClosure2DDistanceThreshold, 10.0f);
    nh.param<float>("historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
    nh.param<bool>("loopClosureDistance2D", loopClosureDistance2D, false);
    nh.param<float>("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
    nh.param<int>("historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    nh.param<float>("historyKeyframeMSEThreshold", historyKeyframeMSEThreshold, 0.3);
    nh.param<int>("postMergeClosureSkipFrames", postMergeClosureSkipFrames, 0);
    nh.param<bool>("gpsMapMerge/enabled", gpsMapMergeEnabled, true);
    nh.param<float>("gpsMapMerge/distance_threshold", gpsMapMergeDistanceThreshold, 5.0f);
    nh.param<float>("gpsMapMerge/ransac_fitness_threshold", gpsMapMergeRansacFitnessThreshold, 0.01f);
    nh.param<float>("gpsMapMerge/ransac_min_rotation_rad", gpsMapMergeRansacMinRotationRad, 0.02f);
    nh.param<float>("gpsMapMerge/ransac_min_translation_m", gpsMapMergeRansacMinTranslationM, 1.0f);
    nh.param<float>("gpsMapMerge/post_rebuild_wait_sec", gpsMapMergePostRebuildWaitSec, 0.5f);
    nh.param<float>("gpsMapMerge/ransac_inlier_dist", gpsMapMergeRansacInlierDist, 0.5f);
    nh.param<int>("gpsMapMerge/ransac_max_iterations", gpsMapMergeRansacMaxIterations, 100000);
    nh.param<float>("gpsMapMerge/voxel_size", gpsMapMergeVoxelSize, 1.0f);
    nh.param<float>("gpsMapMerge/fpfh_radius", gpsMapMergeFpfhRadius, 0.5f);
    nh.param<int>("gpsMapMerge/fpfh_max_nn", gpsMapMergeFpfhMaxNn, 0);
    nh.param<float>("gpsMapMerge/normal_radius", gpsMapMergeNormalRadius, 0.5f);
    nh.param<bool>("gpsMapMerge/mutual_filter", gpsMapMergeMutualFilter, true);

    // gnss
    nh.param<string>("common/gnss_topic", gnss_topic,"/gps/fix");
    nh.param<vector<double>>("mapping/extrinR_Gnss2Lidar", extrinR_Gnss2Lidar, vector<double>());
    nh.param<vector<double>>("mapping/extrinT_Gnss2Lidar", extrinT_Gnss2Lidar, vector<double>());
    nh.param<bool>("useImuHeadingInitialization", useImuHeadingInitialization, false);
    nh.param<bool>("useGpsElevation", useGpsElevation, false);
    nh.param<bool>("useGnssOptimization", useGnssOptimization, false);
    nh.param<bool>("useXYFactor", useXYFactor, false);
    nh.param<float>("gpsCovThreshold", gpsCovThreshold, 2.0);
    nh.param<float>("poseCovThreshold", poseCovThreshold, 25.0);
    // XY factor noise model parameters
    nh.param<double>("xyFactor/noise_roll", xy_factor_noise_roll, 1e-6);
    nh.param<double>("xyFactor/noise_pitch", xy_factor_noise_pitch, 1e-6);
    nh.param<double>("xyFactor/noise_yaw", xy_factor_noise_yaw, 1e-6);
    nh.param<double>("xyFactor/noise_x", xy_factor_noise_x, 1e-4);
    nh.param<double>("xyFactor/noise_y", xy_factor_noise_y, 1e-4);
    nh.param<double>("xyFactor/noise_z", xy_factor_noise_z, 1e2);

    // Reference odometry initialization
    nh.param<bool>("reference_init/enable", use_reference_init, false);
    nh.param<string>("reference_init/topic", reference_odom_topic, "");
    if (use_reference_init)
    {
        ROS_INFO("[REF_INIT] Reference odometry initialization enabled. Topic: %s", reference_odom_topic.c_str());
        ROS_INFO("[REF_INIT] After IMU initialization, the first reference odometry will set the starting state.");
    }

    // Visualization
    nh.param<float>("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
    nh.param<float>("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
    nh.param<float>("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

    // visual ikdtree map
    nh.param<bool>("visulize_IkdtreeMap", visulize_IkdtreeMap, false);

    // reconstruct ikdtree 
    nh.param<bool>("recontructKdTree", recontructKdTree, false);

    // savMap
    nh.param<bool>("savePCD", savePCD, false);
    nh.param<std::string>("savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    // downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    // Keyframe storage: use finest density. Downstream voxel filters work independently on current positions; no adjustment needed.
    float configKeyframeStorage = 0.0f;
    nh.param<float>("keyframeStorageLeafSize", configKeyframeStorage, 0.0f);
    if (configKeyframeStorage > 0.0f) {
        keyframeStorageLeafSize = configKeyframeStorage;
    } else {
        keyframeStorageLeafSize = std::min({mappingSurfLeafSize, (float)gpsMapMergeVoxelSize, globalMapVisualizationLeafSize});
    }
    nh.param<bool>("keyframeStorageDownsampleEn", keyframeStorageDownsampleEn, true);
    downSizeFilterKeyframeStorage.setLeafSize(keyframeStorageLeafSize, keyframeStorageLeafSize, keyframeStorageLeafSize);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterGpsMerge.setLeafSize(gpsMapMergeVoxelSize, gpsMapMergeVoxelSize, gpsMapMergeVoxelSize);
    if (keyframeStorageDownsampleEn)
        ROS_INFO("[KEYFRAME_STORAGE] Storing keyframes at %.3f m voxel", keyframeStorageLeafSize);
    else
        ROS_INFO("[KEYFRAME_STORAGE] Storing keyframes at full resolution (no downsample)");

    // ISAM2 parameters
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG)*0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    // Set IMU-lidar extrinsics and IMU params
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_skip_deskew(pre_deskewed);
    p_imu->set_max_ini_count(max_ini_count);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov)); // accelerometer covariance
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    // Set GNSS extrinsics
    Gnss_T_wrt_Lidar<<VEC_FROM_ARRAY(extrinT_Gnss2Lidar);
    Gnss_R_wrt_Lidar<<MAT_FROM_ARRAY(extrinR_Gnss2Lidar);

    double epsi[23] = {0.001};
    fill(epsi, epsi + 23, 0.001);
    // Init EKF; h_share_model defines plane search and residual computation
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** Create log subfolder directory if specified ***/
    if (!log_subfolder.empty()) {
        string log_dir_path = string(ROOT_DIR) + "Log/" + log_subfolder;
        int unused_mkdir = system((std::string("mkdir -p ") + log_dir_path).c_str());
        (void)unused_mkdir; // Suppress unused variable warning
    }

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = GetLogFilePath("pos_log.txt");
    fp = fopen(pos_log_dir.c_str(), "w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(GetLogFilePath("mat_pre.txt"), ios::out);
    fout_out.open(GetLogFilePath("mat_out.txt"), ios::out);
    fout_dbg.open(GetLogFilePath("dbg.txt"), ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);        // dense cloud in world frame
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);      // dense cloud in body frame
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 1e00000);

    ros::Publisher pubPathUpdate = nh.advertise<nav_msgs::Path>("fast_lio_sam/path_update", 100000);                   // path after isam update
    pubGnssPath = nh.advertise<nav_msgs::Path>("/gnss_path", 100000);
    pubRefPath = nh.advertise<nav_msgs::Path>("/ref_path", 100000);  // reference odometry trajectory
    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/keyframe_submap", 1); // local keyframe submap features
    pubOptimizedGlobalMap = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/map_global_optimized", 1);

    // loop closure
    // matched loop keyframe local map
    pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/icp_loop_closure_history_cloud", 1);
    // current keyframe cloud after loop-closure correction
    pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
    // loop edges (shown as lines between loop frames in rviz)
    pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/fast_lio_sam/mapping/loop_closure_constraints", 1);

    // gnss
    ros::Subscriber sub_gnss = nh.subscribe(gnss_topic, 200000, gnss_cbk);
    
    // reference odometry
    ros::Subscriber sub_reference_odom;
    if ((use_reference_init || useXYFactor) && !reference_odom_topic.empty())
    {
        sub_reference_odom = nh.subscribe(reference_odom_topic, 200000, reference_odom_cbk);
        if (use_reference_init)
        {
            ROS_INFO("[REF_INIT] Subscribed to reference odometry topic: %s", reference_odom_topic.c_str());
        }
    }
    
    // saveMap service
    srvSaveMap  = nh.advertiseService("/save_map" ,  &saveMapService);

    // loadMap service
    srvLoadMap  = nh.advertiseService("/load_map" ,  &loadMapService);

    // savePose service
    srvSavePose  = nh.advertiseService("/save_pose" ,  &savePoseService);

    // Loop closure detection thread
    std::thread loopthread(&loopClosureThread);

    //------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit)
            break;
        ros::spinOnce();

        // Collect current lidar data and the IMU sequence spanning the scan into Measures
        if (sync_packages(Measures))
        {
            // Block on mtx_map_merge: loop thread holds it during RANSAC->ICP->push.
            // We hold it for the entire frame (IMU, matching, solve, ICP, backend) so no processing runs during map merge.
            std::lock_guard<std::mutex> lock_map_merge(mtx_map_merge);

            // Match reference odometry to current frame (for XY factor and timestamp-aligned ref_init)
            if (useXYFactor || use_reference_init)
                matchReferenceOdometry(Measures.lidar_end_time);
            
            // Mark first LiDAR frame with valid reference (ref_path will only include poses >= this time)
            if ((useXYFactor || use_reference_init) && current_frame_ref_odom_valid && first_processed_lidar_time < 0)
                first_processed_lidar_time = Measures.lidar_end_time;
            
            // First lidar frame
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time; // record first frame absolute time
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();

            // Forward-propagate IMU to deskew the cloud (sampling/feature extraction already done)
            // feats_undistort: deskewed cloud in lidar frame
            p_imu->Process(Measures, kf, feats_undistort);
            // Set flag when IMU initialization completes
            if (!imu_init_complete && p_imu->isInitialized())
            {
                imu_init_complete = true;
                ROS_INFO("[REF_INIT] IMU initialization complete. Starting to buffer reference odometry.");
            }
            state_point = kf.get_x();                                               // predicted body state after forward propagation
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I; // lidar position in global frame

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            // EKF considered initialized once enough time has elapsed since the first lidar frame
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            // Initialize odometry prediction with reference odometry after IMU init
            // Use first LiDAR frame that has a timestamp-matched reference (no fallback)
            if (use_reference_init && flg_EKF_inited && !ref_init_applied && current_frame_ref_odom_valid)
            {
                Eigen::Vector3d init_pos(current_frame_ref_odom.pose.pose.position.x,
                                        current_frame_ref_odom.pose.pose.position.y,
                                        current_frame_ref_odom.pose.pose.position.z);
                Eigen::Quaterniond init_rot(current_frame_ref_odom.pose.pose.orientation.w,
                                            current_frame_ref_odom.pose.pose.orientation.x,
                                            current_frame_ref_odom.pose.pose.orientation.y,
                                            current_frame_ref_odom.pose.pose.orientation.z);
                
                // Set EKF state to reference odometry (xyz and orientation)
                // This aligns the coordinate frame of predicted odometry with reference odometry
                state_ikfom state_ref = state_point;  // Copy current state to preserve other variables (vel, biases, gravity, extrinsics)
                state_ref.pos = init_pos;  // Set position from reference
                state_ref.rot = init_rot;  // Set orientation from reference
                
                // Update EKF state
                kf.change_x(state_ref);
                state_point = kf.get_x();
                
                // Only clear ikdtree if we don't have a loaded map (i.e., if cloudKeyPoses3D is empty)
                if (cloudKeyPoses3D->points.empty()) {
                    PointVector empty_cloud;
                    ikdtree.reconstruct(empty_cloud);  // thread-safe (holds rebuild locks)
                    Localmap_Initialized = false;
                }

                ref_init_applied = true;
                pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
                ROS_WARN("[REF_INIT] Initialized odometry prediction with reference odometry (timestamp-matched to first LiDAR frame):");
                ROS_WARN("  Position: (%.3f, %.3f, %.3f)", init_pos(0), init_pos(1), init_pos(2));
                ROS_WARN("  Orientation: (w=%.3f, x=%.3f, y=%.3f, z=%.3f)", 
                         init_rot.w(), init_rot.x(), init_rot.y(), init_rot.z());
            }

            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment(); // re-center the local map bounding box on the lidar pose and drop far points

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size(); // downsampled point count of current frame

            /*** initialize the map kdtree ***/
            if (ikdtree.Root_Node == nullptr)
            {
                if (feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i])); // transform point to world frame
                    }
                    // Build the ikd-tree from the world-frame downsampled cloud
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            // lidar --> imu
            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                     << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << endl;

            if (visulize_IkdtreeMap) // If you need to see map point, change to "if(1)"
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
                publish_map(pubLaserCloudMap);
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(laser_point_cov, solve_H_time); // predict + update
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I; // lidar position in world frame
            geoQuat.x = state_point.rot.coeffs()[0];                                // current IMU orientation quaternion in world frame
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();
            getCurPose(state_point); // update transformTobeMapped
            double t_frontend_end = omp_get_wtime();
            /*back end*/
            // Gate keyframe, add odom/GPS/loop factors, optimize the factor graph,
            // then store optimized pose/covariance and the keyframe cloud
            size_t kf_count_before = 0;
            { std::lock_guard<std::mutex> lock(mtx); kf_count_before = cloudKeyPoses3D->size(); }
            saveKeyFramesAndFactor();
            // Update all keyframe poses, refresh trajectory, and reconstruct ikdtree
            correctPoses();
            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);
            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            /******* Publish points *******/
            if (path_en){
                publish_path(pubPath);
                publish_gnss_path(pubGnssPath);                        // gnss trajectory
                publish_path_update(pubPathUpdate);             // isam2-optimized path
                publish_loaded_path();             // loaded map path
                if (useXYFactor || use_reference_init)
                {
                    publish_ref_path(pubRefPath);                      // reference odometry trajectory
                }
                static int jjj = 0;
                jjj++;
                if (jjj % 100 == 0)
                {
                    publishGlobalMap();             // local feature map
                }
            }
            if (scan_pub_en || pcd_save_en)
                publish_frame_world(pubLaserCloudFull);        // cloud in world frame
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body);         // cloud in imu frame

#ifdef __linux__
            printf("[MEM] total: %.1f MB (ikdtree %d pts)\n", getRssMb(), ikdtree.size());
#endif

            // Per-keyframe runtime and memory for save_pose performance report
            {
                size_t kf_count_after = 0;
                { std::lock_guard<std::mutex> lock(mtx); kf_count_after = cloudKeyPoses3D->size(); }
                if (kf_count_after > kf_count_before) {
                    std::lock_guard<std::mutex> lock(mtx_perf);
                    g_frame_timestamps.push_back(Measures.lidar_beg_time);
                    g_frame_runtimes_ms.push_back((t5 - t0) * 1000.0);
                    g_frame_frontend_ms.push_back((t_frontend_end - t0) * 1000.0);
                    g_frame_backend_ms.push_back((t5 - t_frontend_end) * 1000.0);
#ifdef __linux__
                    g_frame_mem_mb.push_back(getRssMb());
#else
                    g_frame_mem_mb.push_back(0.0);
#endif
                }
            }

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1) / frame_num + (kdtree_incremental_time) / frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time + solve_H_time) / frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter++;
                // printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n", t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose() << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose() << " " << state_point.vel.transpose()
                         << " " << state_point.bg.transpose() << " " << state_point.ba.transpose() << " " << state_point.grav << " " << feats_undistort->points.size() << endl;
                dump_lio_state_to_log(fp);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = GetLogFilePath("fast_lio_time_log.csv");
        fp2 = fopen(log_dir.c_str(), "w");
        fprintf(fp2, "time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0; i < time_log_counter; i++)
        {
            fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n", T1[i], s_plot[i], int(s_plot2[i]), s_plot3[i], s_plot4[i], int(s_plot5[i]), s_plot6[i], int(s_plot7[i]), int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    startFlag = false;
    loopthread.join();

    return 0;
}
