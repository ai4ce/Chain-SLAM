// GTSAM/ISAM2 pose-graph back end: keyframe gating, factor construction, optimization.
#include "slam_common.h"


/**
 * Decide whether the current frame becomes a keyframe based on pose change vs. the previous frame.
 */
bool saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal = trans2Affine3f(transformTobeMapped);

    // Relative pose increment between previous and current frame
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    // Skip keyframe if both rotation and translation are below thresholds
    if (abs(roll) < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
        return false;
    return true;
}


/**
 * Add the lidar odometry factor.
 */
void addOdomFactor()
{
    // Determine if this is the first NEW keyframe (after loaded keyframes, if any)
    // Loaded keyframes are already in ISAM2 at indices 0..(loadedKeyframeCount-1)
    // New keyframes will be added starting at index loadedKeyframeCount
    size_t currentKeyframeIndex = cloudKeyPoses3D->size(); // Index in cloudKeyPoses3D (includes loaded + new)
    size_t isamKeyframeIndex = isamCurrentEstimate.size(); // Index in ISAM2 (includes loaded + new)
    bool isFirstNewKeyframe = (currentKeyframeIndex == loadedKeyframeCount);
    
    if (isFirstNewKeyframe)
    {
        // First new keyframe after loading (or first keyframe if no map was loaded)
        // Add prior factor to start a new disconnected component (do NOT connect to last loaded keyframe)
        // Loop closure will connect them when detected
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) <<1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished()); // rad*rad, meter*meter   // indoor 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12    //  1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(isamKeyframeIndex, trans2gtsamPose(transformTobeMapped), priorNoise));
        // Set initial value for the variable node
        initialEstimate.insert(isamKeyframeIndex, trans2gtsamPose(transformTobeMapped));
    }
    else
    {
        // Subsequent new keyframes: add between factor from previous new keyframe to current
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        // Get previous keyframe pose from ISAM2 estimate
        gtsam::Pose3 poseFrom = isamCurrentEstimate.at<gtsam::Pose3>(isamKeyframeIndex - 1);
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
        // BetweenFactor(prevId, curId, relative pose as observation, noise covariance)
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(isamKeyframeIndex - 1, isamKeyframeIndex, poseFrom.between(poseTo), odometryNoise));
        // Set initial value for the variable node
        initialEstimate.insert(isamKeyframeIndex, poseTo);
    }
}


/**
 * Rebuild ISAM2 with RANSAC-transformed poses after a successful GPS map merge.
 * This reinitializes the optimizer so new keyframes continue to be optimized by ISAM from the correct alignment.
 * Called from saveKeyFramesAndFactor when mapMergeSuccessful and rebuild not yet done.
 * Must run whenever RANSAC was applied, even if ICP failed (poses were transformed; graph must be consistent).
 */
bool rebuildIsamAfterRansac()
{
    // Rebuild only once after GPS RANSAC merge; never for regular loop closures
    if (!mapMergeSuccessful || gpsMapMergeRebuildDone || isam == nullptr)
        return false;

    const gtsam::NonlinearFactorGraph& isamFactors = isam->getFactorsUnsafe();
    gtsam::NonlinearFactorGraph fullGraph;
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
    for (size_t i = 0; i < isamFactors.size(); ++i) {
        auto f = isamFactors.at(i);
        // PriorFactor for first new keyframe pins to pre-RANSAC pose; replace with RANSAC
        auto priorFactor = boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(f);
        if (priorFactor && priorFactor->key() == (gtsam::Key)loadedKeyframeCount)
            continue;  // Skip; add RANSAC Prior below
        fullGraph.add(f);
    }
    // Anchor first new keyframe to RANSAC pose (was pinned to pre-RANSAC in old graph)
    fullGraph.add(gtsam::PriorFactor<gtsam::Pose3>(loadedKeyframeCount, pclPointTogtsamPose3(cloudKeyPoses6D->points[loadedKeyframeCount]), priorNoise));

    // Add current factors (Odom, GPS, XY for new keyframe) from gtSAMgraph
    for (size_t i = 0; i < gtSAMgraph.size(); ++i)
        fullGraph.add(gtSAMgraph.at(i));

    // Add loop closure factors from queue
    for (size_t i = 0; i < loopIndexQueue.size(); ++i) {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        fullGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, loopPoseQueue[i], loopNoiseQueue[i]));
    }

    // Build initial estimate: RANSAC poses for 0..N-1, RANSAC-transformed pose for new keyframe N
    size_t N = cloudKeyPoses6D->size();  // new keyframe index
    gtsam::Values initEst;
    for (size_t i = 0; i < N; ++i)
        initEst.insert((gtsam::Key)i, pclPointTogtsamPose3(cloudKeyPoses6D->points[i]));

    // New keyframe pose in RANSAC frame: apply gpsRansacTransform to transformTobeMapped
    gtsam::Pose3 poseWrong = trans2gtsamPose(transformTobeMapped);
    Eigen::Matrix4d T_ransac = gpsRansacTransform.matrix().cast<double>();
    Eigen::Matrix4d T_correct = T_ransac * poseWrong.matrix();
    initEst.insert((gtsam::Key)N, gtsam::Pose3(T_correct));

    // Rebuild ISAM2
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    delete isam;
    isam = new gtsam::ISAM2(params);

    isam->update(fullGraph, initEst);
    for (int _ = 0; _ < 5; ++_)
        isam->update();
    isamCurrentEstimate = isam->calculateBestEstimate();

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    gtSAMgraph.resize(0);
    initialEstimate.clear();
    aLoopIsClosed = true;
    gpsMapMergeRebuildDone = true;  // Rebuild only once; future keyframes use incremental update

    ROS_INFO("[GPS_MAP_MERGE] ISAM2 rebuilt with RANSAC poses. New keyframes will be optimized from aligned frame.");
    return true;
}


/**
 * Add loop closure factors.
 */
void addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    int numAdded = 0;
    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        std::pair<int, int> canonicalPair(std::min(indexFrom, indexTo), std::max(indexFrom, indexTo));
        if (loopClosurePairsAdded.find(canonicalPair) != loopClosurePairsAdded.end())
            continue;
        numAdded++;
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, loopPoseQueue[i], loopNoiseQueue[i]));
        loopClosurePairsAdded.insert(canonicalPair);
    }

    if (numAdded > 0)
        aLoopIsClosed = true;

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
}


/**
 * Match reference odometry to current LiDAR frame
 */
void matchReferenceOdometry(double lidar_time)
{
    current_frame_ref_odom_valid = false;
    mtx_buffer.lock();
    if (reference_odom_buffer.empty())
    {
        mtx_buffer.unlock();
        return;
    }
    
    double min_diff = 0.01;
    int best_idx = -1;
    for (size_t i = 0; i < reference_odom_buffer.size(); i++)
    {
        double diff = fabs(reference_odom_buffer[i].header.stamp.toSec() - lidar_time);
        if (diff < min_diff)
        {
            min_diff = diff;
            best_idx = i;
        }
        if (reference_odom_buffer[i].header.stamp.toSec() > lidar_time + 0.01)
            break;
    }
    
    if (best_idx >= 0)
    {
        current_frame_ref_odom = reference_odom_buffer[best_idx];
        current_frame_ref_odom_valid = true;
        for (int i = 0; i <= best_idx; i++)
            if (!reference_odom_buffer.empty())
                reference_odom_buffer.pop_front();
    }
    mtx_buffer.unlock();
}


/**
 * Add XY factor: constrains all 6DOF (xyz, rpy) of the predicted odometry toward the reference odometry.
 */
void addXYFactor()
{
    if (cloudKeyPoses3D->points.empty())
        return;
    if (!current_frame_ref_odom_valid)
        return;
    
    nav_msgs::Odometry ref_odom = current_frame_ref_odom;

    // Reference odometry XYZ
    double ref_x = ref_odom.pose.pose.position.x;
    double ref_y = ref_odom.pose.pose.position.y;
    double ref_z = ref_odom.pose.pose.position.z;

    // Reference odometry roll/pitch/yaw
    tf::Quaternion q_ref(
        ref_odom.pose.pose.orientation.x,
        ref_odom.pose.pose.orientation.y,
        ref_odom.pose.pose.orientation.z,
        ref_odom.pose.pose.orientation.w);
    tf::Matrix3x3 m_ref(q_ref);
    double roll_ref, pitch_ref, yaw_ref;
    m_ref.getRPY(roll_ref, pitch_ref, yaw_ref);

    // Constrained pose: all 6DOF (xyz, rpy) taken from the reference
    gtsam::Point3 constrained_pos(ref_x, ref_y, ref_z);
    gtsam::Rot3 constrained_rot = gtsam::Rot3::RzRyRx(roll_ref, pitch_ref, yaw_ref);
    gtsam::Pose3 constrained_pose(constrained_rot, constrained_pos);

    // Noise model: strong constraint (low variance) on all 6DOF
    // Order: roll, pitch, yaw, x, y, z (rad*rad, meter*meter); variances configurable via YAML
    gtsam::noiseModel::Diagonal::shared_ptr xy_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << xy_factor_noise_roll, xy_factor_noise_pitch, xy_factor_noise_yaw, 
                             xy_factor_noise_x, xy_factor_noise_y, xy_factor_noise_z).finished());

    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(
        cloudKeyPoses3D->size(), constrained_pose, xy_noise));
}


/**
 * Add GPS factor.
*/
void addGPSFactor()
{
    if (gnss_buffer.empty())
        return;
    // Skip if no keyframes or the first-to-last keyframe distance is under 5m
    if (cloudKeyPoses3D->points.empty())
        return;
    else
    {
        if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
            return;
    }
    // Pose covariance is small; no need for GPS correction
    if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
        return;
    static PointType lastGPSPoint;
    while (!gnss_buffer.empty())
    {
        // Drop odometry older than the current frame
        if (gnss_buffer.front().header.stamp.toSec() < lidar_end_time - 0.05)
        {
            gnss_buffer.pop_front();
        }
        // Stop once past the current frame
        else if (gnss_buffer.front().header.stamp.toSec() > lidar_end_time + 0.05)
        {
            break;
        }
        else
        {
            nav_msgs::Odometry thisGPS = gnss_buffer.front();
            gnss_buffer.pop_front();
            float noise_x = thisGPS.pose.covariance[0];         // x covariance
            float noise_y = thisGPS.pose.covariance[7];
            float noise_z = thisGPS.pose.covariance[14];      // z (elevation) covariance
            if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                continue; // GPS noise covariance too large, unusable
            float gps_x = thisGPS.pose.pose.position.x;
            float gps_y = thisGPS.pose.pose.position.y;
            float gps_z = thisGPS.pose.pose.position.z;
            if (!useGpsElevation)           // whether to use GPS elevation
            {
                gps_z = transformTobeMapped[5];
                noise_z = 0.01;
            }

            // (0,0,0) is invalid data
            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                continue;
            // Add a GPS factor every 5m
            PointType curGPSPoint;
            curGPSPoint.x = gps_x;
            curGPSPoint.y = gps_y;
            curGPSPoint.z = gps_z;
            if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                continue;
            else
                lastGPSPoint = curGPSPoint;
            gtsam::Vector Vector3(3);
            Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
            gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
            gtSAMgraph.add(gps_factor);
            aLoopIsClosed = true;
            ROS_INFO("GPS Factor Added");
            break;
        }
    }
}


void saveKeyFramesAndFactor()
{
    if (saveFrame() == false)
        return;
    // When using XY factor or reference init, only create keyframes after first LiDAR frame with valid reference.
    // This aligns path_update trajectory start with /path trajectory (both start at first_processed_lidar_time).
    if ((useXYFactor || use_reference_init) && first_processed_lidar_time < 0)
        return;
    // Lidar odometry factor (from fast-lio): relative inter-frame pose in body frame
    addOdomFactor();
    // GPS factor (UTM -> WGS84)
    if (useGnssOptimization)
    {
        addGPSFactor();
    }
    // XY factor: constrain pose toward reference odometry
    if (useXYFactor)
    {
        addXYFactor();
    }
    // After GPS RANSAC: rebuild ISAM2 with RANSAC-transformed poses so optimization continues from aligned frame
    if (rebuildIsamAfterRansac())
    {
        ros::Duration(gpsMapMergePostRebuildWaitSec).sleep();
        doubleCheckPoseLoopClosuresAfterMerge();
    }
    else
    {
        // Loop closure factors (Euclidean-distance based detection)
        addLoopFactor();
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        if (aLoopIsClosed == true) // run extra updates when a loop factor was added
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }
        // Clear the staged graph after update (history is retained inside ISAM)
        gtSAMgraph.resize(0);
        initialEstimate.clear();
    }

    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 latestEstimate;

    // Optimization result
    isamCurrentEstimate = isam->calculateBestEstimate();
    // Current-frame pose
    latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);

    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size(); // use intensity as the frame index

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = lidar_end_time;

    {
        std::lock_guard<std::mutex> lock(mtx);
        cloudKeyPoses3D->push_back(thisPose3D);
        cloudKeyPoses6D->push_back(thisPose6D);
    }

    // Pose covariance
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

    // Update ESKF state with optimized pose
    state_ikfom state_updated = kf.get_x();
    Eigen::Vector3d pos(latestEstimate.translation().x(), latestEstimate.translation().y(), latestEstimate.translation().z());
    Eigen::Quaterniond q = EulerToQuat(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(), latestEstimate.rotation().yaw());

    state_updated.pos = pos;
    state_updated.rot =  q;
    state_point = state_updated; // also used for visualization
    kf.change_x(state_updated);  // apply ISAM2-optimized correction to cur_pose

    // TODO: P correction unverified; modifying P (per yanliangwang's approach) caused divergence
    // esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P_updated = kf.get_P();
    // P_updated.setIdentity();
    // P_updated(6, 6) = P_updated(7, 7) = P_updated(8, 8) = 0.00001;
    // P_updated(9, 9) = P_updated(10, 10) = P_updated(11, 11) = 0.00001;
    // P_updated(15, 15) = P_updated(16, 16) = P_updated(17, 17) = 0.0001;
    // P_updated(18, 18) = P_updated(19, 19) = P_updated(20, 20) = 0.001;
    // P_updated(21, 21) = P_updated(22, 22) = 0.00001;
    // kf.change_P(P_updated);

    // Downsampled feature cloud for the current keyframe
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*feats_undistort, *thisSurfKeyFrame);
    if (keyframeStorageDownsampleEn) {
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrameDS(new pcl::PointCloud<PointType>());
        downSizeFilterKeyframeStorage.setInputCloud(thisSurfKeyFrame);
        downSizeFilterKeyframeStorage.filter(*thisSurfKeyFrameDS);
        surfCloudKeyFrames.push_back(thisSurfKeyFrameDS);
    } else {
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);
    }

    updatePath(thisPose6D);
}


/**
 * Update poses of all variable nodes (all historical keyframes) and refresh the odometry trajectory.
 */
void correctPoses()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true)
    {
        // Clear the odometry trajectory
        globalPath.poses.clear();
        // Update all keyframe poses from the optimized estimate
        int numPoses = (int)isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();
        }
        
        // Add keyframes to optimized path, but create visual break between loaded and new keyframes
        // First, add loaded keyframes (if any)
        for (int i = 0; i < (int)loadedKeyframeCount && i < numPoses; ++i)
        {
            updatePath(cloudKeyPoses6D->points[i]);
        }
        
        // Clear path to create visual break, then add new keyframes
        if (loadedKeyframeCount > 0 && numPoses > (int)loadedKeyframeCount)
        {
            globalPath.poses.clear();
        }
        
        // Add new keyframes (starting from loadedKeyframeCount)
        for (int i = loadedKeyframeCount; i < numPoses; ++i)
        {
            updatePath(cloudKeyPoses6D->points[i]);
        }
        
        // Reconstruct the ikdtree submap
        recontructIKdTree();
        ROS_INFO("ISMA2 Update");
        aLoopIsClosed = false;
    }
}
