// Loop closure: candidate detection (3D radius, 2D GNSS), ICP matching, worker thread.
#include "slam_common.h"


// =========== Pose-based loop closure ===========
// new-to-new and new-to-loaded. GPS-based: new-to-loaded only.
bool detectLoopClosureDistance(int *latestID, int *closestID)
{
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;

    // Skip if this keyframe already has a loop closure association
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;
    // When map is pre-loaded AND gpsMapMerge is enabled: wait for GPS-based merge first (aligns coordinate frames).
    // If gpsMapMerge is disabled: always allow pose-based loop closure (including new-to-loaded).
    if (gpsMapMergeEnabled && loadedKeyframeCount > 0 && !mapMergeSuccessful)
        return false;
    // Find nearest historical keyframes to the current keyframe
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    const pcl::PointCloud<PointType>::Ptr searchCloud = [&]() {
        if (loopClosureDistance2D) {
            pcl::PointCloud<PointType>::Ptr cloud2D(new pcl::PointCloud<PointType>());
            for (const auto& p : copy_cloudKeyPoses3D->points) {
                PointType q;
                q.x = p.x; q.y = p.y; q.z = 0;
                cloud2D->push_back(q);
            }
            return cloud2D;
        }
        return copy_cloudKeyPoses3D;
    }();
    PointType queryPoint = copy_cloudKeyPoses3D->back();
    if (loopClosureDistance2D)
        queryPoint.z = 0;
    if (searchCloud->size() < 2)  // FLANN middleSplit requires >= 2 points
        return false;
    kdtreeHistoryKeyPoses->setInputCloud(searchCloud);
    kdtreeHistoryKeyPoses->radiusSearch(queryPoint, historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    // Among candidates, pick those far enough in time from the current frame
    // Skip timestamp filtering if matching with loaded map keyframes
    // Use current keyframe's timestamp (not lidar_end_time) to avoid race with main thread
    double curKfTime = copy_cloudKeyPoses6D->points[loopKeyCur].time;
    float minDist = std::numeric_limits<float>::max();
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        bool isLoadedKeyframe = (id < (int)loadedKeyframeCount);
        bool timeCheckPassed = (fabs(copy_cloudKeyPoses6D->points[id].time - curKfTime) > historyKeyframeSearchTimeDiff);
        
        // If matching with loaded map, skip timestamp check; otherwise require time difference
        if (isLoadedKeyframe || timeCheckPassed)
        {
            // Find the closest candidate among valid ones
            if (pointSearchSqDisLoop[i] < minDist)
            {
                minDist = pointSearchSqDisLoop[i];
                loopKeyPre = id;
            }
        }
    }
    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    double preKfTime = copy_cloudKeyPoses6D->points[loopKeyPre].time;
    double timeDiffSec = fabs(curKfTime - preKfTime);
    ROS_INFO("Found pose-based loop closure candidate %d %d (time_diff=%.2f s)", loopKeyCur, loopKeyPre, timeDiffSec);
    return true;
}


/**
 * Gather feature points from keyframes adjacent to `key` (+-searchNum) and downsample.
 * @param voxelLeafSize when > 0 use this (e.g. gpsMapMergeVoxelSize for GPS merge); else use downSizeFilterICP (mappingSurfLeafSize)
 */
void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum, float voxelLeafSize)
{
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    auto surfcloud_keyframes_size = surfCloudKeyFrames.size() ;
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize)
            continue;

        if (keyNear < 0 || keyNear >= surfcloud_keyframes_size)
            continue;

        // cloudKeyPoses6D stores T_w_b; clouds are in lidar frame. transformPointCloud applies the extrinsic (cf. pointBodyToWorld)
        *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]); // fast-lio does no feature extraction; cloud is treated as surf
    }

    if (nearKeyframes->empty())
        return;

    // Downsample (voxelLeafSize > 0: gpsMapMergeVoxelSize for GPS merge; else mappingSurfLeafSize for pose loop closure)
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    if (voxelLeafSize > 0.0f) {
        downSizeFilterGpsMerge.setLeafSize(voxelLeafSize, voxelLeafSize, voxelLeafSize);
        downSizeFilterGpsMerge.setInputCloud(nearKeyframes);
        downSizeFilterGpsMerge.filter(*cloud_temp);
    } else {
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
    }
    *nearKeyframes = *cloud_temp;
}


/**
 * 2D GNSS closure: translation from pose difference + ICP. No RANSAC.
 */
bool try2DMatchTranslationOnly(int loopKeyCur, int loopKeyPre,
    const pcl::PointCloud<PointType>::Ptr& cureKeyframeCloud,
    const pcl::PointCloud<PointType>::Ptr& prevKeyframeCloud,
    Eigen::Matrix4f* out_icpTransform, float* out_fitnessScore)
{
    if (cureKeyframeCloud->size() < 50 || prevKeyframeCloud->size() < 50) return false;

    const auto& curPose = copy_cloudKeyPoses3D->points[loopKeyCur];
    const auto& prevPose = copy_cloudKeyPoses3D->points[loopKeyPre];
    float dx = prevPose.x - curPose.x;
    float dy = prevPose.y - curPose.y;
    float dz = prevPose.z - curPose.z;

    Eigen::Matrix4f T_translate = Eigen::Matrix4f::Identity();
    T_translate(0, 3) = dx;
    T_translate(1, 3) = dy;
    T_translate(2, 3) = dz;

    pcl::PointCloud<PointType>::Ptr curTranslated(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*cureKeyframeCloud, *curTranslated, T_translate);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    configureLoopIcp(icp);
    icp.setInputSource(curTranslated);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (!icp.hasConverged() || icp.getFitnessScore() > historyKeyframeMSEThreshold) {
        ROS_INFO("[2D_MATCH] ICP failed: converged=%d, mse=%.4f (threshold=%.4f)",
                 icp.hasConverged() ? 1 : 0, icp.getFitnessScore(), historyKeyframeMSEThreshold);
        return false;
    }

    Eigen::Matrix4f fullCorrection = icp.getFinalTransformation() * T_translate;
    if (out_icpTransform) *out_icpTransform = fullCorrection;
    if (out_fitnessScore) *out_fitnessScore = icp.getFitnessScore();
    ROS_INFO("[2D_MATCH] ICP ok: KF %d <-> %d, mse=%.4f, t_init=[%.2f, %.2f, %.2f]", loopKeyCur, loopKeyPre, icp.getFitnessScore(), dx, dy, dz);
    return true;
}


/**
 * 2D loop closure (independent of map merge): GNSS 2D search over ALL keyframes (loaded + new).
 * Called when 3D pose-based loop closure fails. Does NOT apply transforms; just adds BetweenFactor.
 */
bool perform2DLoopClosure(int* out_loopKeyCur, int* out_loopKeyPre, Eigen::Matrix4f* out_icpTransform, float* out_fitnessScore)
{
    if (!loopClosure2DEnabled) return false;
    if (gnss_raw_wgs84.empty() || gnss_cloudKeyPoses6D->empty()) return false;

    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    if (loopKeyCur < 0) return false;
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end()) return false;

    pcl::PointCloud<PointType>::Ptr allGnssCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)copy_cloudKeyPoses6D->size(); ++i) {
        Eigen::Vector3d utmPos(GNSS_INVALID_MARKER, GNSS_INVALID_MARKER, GNSS_INVALID_MARKER);
        if (i < (int)loadedKeyframeCount && i < (int)loadedKeyframeGnssPositions.size()) {
            const auto& g = loadedKeyframeGnssPositions[i];
            if (fabs(g.x()) < 1e9) utmPos = g;
        } else {
            double kfTime = copy_cloudKeyPoses6D->points[i].time;
            double bestTimeDiff = 1e9;
            int bestGnssIdx = -1;
            for (size_t j = 0; j < gnss_cloudKeyPoses6D->size(); ++j) {
                double diff = fabs(gnss_cloudKeyPoses6D->points[j].time - kfTime);
                if (diff < bestTimeDiff && diff < 0.5 && j < gnss_raw_wgs84.size()) {
                    bestTimeDiff = diff;
                    bestGnssIdx = j;
                }
            }
            if (bestGnssIdx >= 0) {
                double e, n, a;
                Wgs84ToUtm(gnss_raw_wgs84[bestGnssIdx].x(), gnss_raw_wgs84[bestGnssIdx].y(), gnss_raw_wgs84[bestGnssIdx].z(), e, n, a);
                utmPos = Eigen::Vector3d(e, n, a);
            }
        }
        if (fabs(utmPos.x()) >= 1e9) continue;
        PointType pt;
        pt.x = utmPos.x(); pt.y = utmPos.y(); pt.z = utmPos.z();
        pt.intensity = i;
        allGnssCloud->push_back(pt);
    }
    if (allGnssCloud->size() < 2) return false;

    pcl::KdTreeFLANN<PointType> kdtreeAllGnss;
    kdtreeAllGnss.setInputCloud(allGnssCloud);

    double kfTime = copy_cloudKeyPoses6D->points[loopKeyCur].time;
    double bestTimeDiff = 1e9;
    int bestGnssIdx = -1;
    for (size_t j = 0; j < gnss_cloudKeyPoses6D->size(); ++j) {
        double diff = fabs(gnss_cloudKeyPoses6D->points[j].time - kfTime);
        if (diff < bestTimeDiff && diff < 0.5 && j < gnss_raw_wgs84.size()) {
            bestTimeDiff = diff;
            bestGnssIdx = j;
        }
    }
    if (bestGnssIdx < 0) return false;
    double curE, curN, curA;
    Wgs84ToUtm(gnss_raw_wgs84[bestGnssIdx].x(), gnss_raw_wgs84[bestGnssIdx].y(), gnss_raw_wgs84[bestGnssIdx].z(), curE, curN, curA);
    PointType queryPt;
    queryPt.x = curE; queryPt.y = curN; queryPt.z = curA;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeAllGnss.radiusSearch(queryPt, loopClosure2DDistanceThreshold, pointSearchInd, pointSearchSqDis, 0);
    int loopKeyPre = -1;
    float bestDistSq = loopClosure2DDistanceThreshold * loopClosure2DDistanceThreshold;
    for (size_t i = 0; i < pointSearchInd.size(); ++i) {
        int idx = (int)allGnssCloud->points[pointSearchInd[i]].intensity;
        if (idx == loopKeyCur) continue;
        if (fabs(copy_cloudKeyPoses6D->points[idx].time - kfTime) < historyKeyframeSearchTimeDiff) continue;
        int a = std::min(loopKeyCur, idx), b = std::max(loopKeyCur, idx);
        if (loopClosurePairsAdded.find({a, b}) != loopClosurePairsAdded.end()) continue;
        if (pointSearchSqDis[i] < bestDistSq) {
            bestDistSq = pointSearchSqDis[i];
            loopKeyPre = idx;
        }
    }
    if (loopKeyPre < 0) return false;

    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, historyKeyframeSearchNum, gpsMapMergeVoxelSize);
    loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, gpsMapMergeVoxelSize);
    if (cureKeyframeCloud->empty() || prevKeyframeCloud->empty()) return false;

    ROS_INFO("[2D_REGULAR_CLOSURE] Trying 2D regular loop closure: KF %d <-> candidate KF %d (GNSS dist=%.2fm)", loopKeyCur, loopKeyPre, sqrt(bestDistSq));
    Eigen::Matrix4f correction2D;
    float fitness2D;
    if (!try2DMatchTranslationOnly(loopKeyCur, loopKeyPre, cureKeyframeCloud, prevKeyframeCloud, &correction2D, &fitness2D))
        return false;

    *out_loopKeyCur = loopKeyCur;
    *out_loopKeyPre = loopKeyPre;
    *out_icpTransform = correction2D;
    if (out_fitnessScore) *out_fitnessScore = fitness2D;
    ROS_INFO("[2D_LOOP_CLOSURE] GNSS-based match KF %d <-> %d, dist=%.2fm", loopKeyCur, loopKeyPre, sqrt(bestDistSq));
    return true;
}


void performLoopClosure()
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);
    string odometryFrame = "camera_init";

    if (cloudKeyPoses3D->points.empty() == true)
    {
        return;
    }

    mtx.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    // Block 1: Map merge (independent). Block 2: 3D pose-based. Block 3: 2D GNSS-based.
    int loopKeyCur;
    int loopKeyPre;
    bool loopClosureSuccess = false;
    Eigen::Matrix4f loopClosureCorrection = Eigen::Matrix4f::Identity();
    float loopClosureNoiseScore = 0.0f;
    std::unique_lock<std::mutex> lock_map_merge(mtx_map_merge, std::defer_lock);
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    configureLoopIcp(icp);

    if (gpsMapMergeEnabled && loadedKeyframeCount > 0 && !mapMergeSuccessful)
    {
        if (performGpsMapMerge(&loopKeyCur, &loopKeyPre, &lock_map_merge))
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
            icp.setInputSource(cureKeyframeCloud);
            icp.setInputTarget(prevKeyframeCloud);
            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
            icp.align(*unused_result);
            if (icp.hasConverged() && icp.getFitnessScore() <= historyKeyframeMSEThreshold)
            {
                loopClosureSuccess = true;
                loopClosureCorrection = icp.getFinalTransformation();
                loopClosureNoiseScore = icp.getFitnessScore();
            }
            else
            {
                ROS_INFO("[LOOP_CLOSURE] Failed (GPS merge): KF %d <-> %d, converged=%d, mse=%.6f (threshold=%.4f)",
                         loopKeyCur, loopKeyPre, icp.hasConverged() ? 1 : 0, icp.getFitnessScore(), historyKeyframeMSEThreshold);
            }
        }
    }

    if (!loopClosureSuccess)
    {
        bool from3D = detectLoopClosureDistance(&loopKeyCur, &loopKeyPre);
        if (from3D)
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
            icp.setInputSource(cureKeyframeCloud);
            icp.setInputTarget(prevKeyframeCloud);
            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
            icp.align(*unused_result);
            if (icp.hasConverged() && icp.getFitnessScore() <= historyKeyframeMSEThreshold)
            {
                loopClosureSuccess = true;
                loopClosureCorrection = icp.getFinalTransformation();
                loopClosureNoiseScore = icp.getFitnessScore();
            }
            else
            {
                ROS_INFO("[LOOP_CLOSURE] Failed (3D pose): KF %d <-> %d, converged=%d, mse=%.6f (threshold=%.4f)",
                         loopKeyCur, loopKeyPre, icp.hasConverged() ? 1 : 0, icp.getFitnessScore(), historyKeyframeMSEThreshold);
            }
        }
    }

    if (!loopClosureSuccess && loopClosure2DEnabled)
    {
        Eigen::Matrix4f correction2D;
        float fitness2D;
        if (perform2DLoopClosure(&loopKeyCur, &loopKeyPre, &correction2D, &fitness2D))
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
            loopClosureSuccess = true;
            loopClosureCorrection = correction2D;
            loopClosureNoiseScore = fitness2D;
        }
    }

    if (!loopClosureSuccess)
        return;

    // Publish current keyframe cloud after applying the loop-closure correction
    if (pubIcpKeyFrames.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, loopClosureCorrection);
        publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
    }

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame.matrix() = loopClosureCorrection;

    // Current-frame pose before loop correction
    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    // Current-frame pose after loop correction
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    // Pose of the matched loop keyframe
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = loopClosureNoiseScore; // loop closure noise from icp
    // Apply scaling factor to give loop closure more/less weight relative to other factors
    // Lower scale = lower variance = stronger constraint (higher weight)
    float scaledNoiseScore = noiseScore * loop_closure_noise_scale;
    Vector6 << scaledNoiseScore, scaledNoiseScore, scaledNoiseScore, scaledNoiseScore, scaledNoiseScore, scaledNoiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

    // Queue data needed to add the loop-closure factor
    mtx.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    mtx.unlock();

    loopIndexContainer[loopKeyCur].insert(loopKeyPre); // direct closure
    directLoopClosureEdges.insert(std::make_pair(std::min(loopKeyCur, loopKeyPre), std::max(loopKeyCur, loopKeyPre)));
    loopClosureVariance[std::make_pair(std::min(loopKeyCur, loopKeyPre), std::max(loopKeyCur, loopKeyPre))] = scaledNoiseScore;

    // Trace all connected keyframes: if matched keyframe is from loaded map, find all connected keyframes
    // Only add chained Between factors when addChainedClosures is enabled
    if (addChainedClosures && loopKeyPre < (int)loadedKeyframeCount) {
        std::set<int> connectedKeyframes;
        std::vector<int> toVisit;
        toVisit.push_back(loopKeyPre);
        connectedKeyframes.insert(loopKeyPre);
        
        // BFS traversal to find all connected keyframes in the loop closure graph
        while (!toVisit.empty()) {
            int current = toVisit.back();
            toVisit.pop_back();
            
            // Forward: all keyframes current is loop-closed to
            auto it = loopIndexContainer.find(current);
            if (it != loopIndexContainer.end()) {
                for (int next : it->second) {
                    if (next < (int)loadedKeyframeCount && connectedKeyframes.find(next) == connectedKeyframes.end()) {
                        connectedKeyframes.insert(next);
                        toVisit.push_back(next);
                    }
                }
            }
            // Reverse: all keyframes that are loop-closed to current
            for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it) {
                if (it->second.count(current) && it->first < (int)loadedKeyframeCount) {
                    if (connectedKeyframes.find(it->first) == connectedKeyframes.end()) {
                        connectedKeyframes.insert(it->first);
                        toVisit.push_back(it->first);
                    }
                }
            }
        }
        
        // Add constraints from new keyframe to all connected keyframes
        // For each chained closure: run ICP cur vs conn and use actual ICP MSE for noise
        mtx.lock();
        pcl::PointCloud<PointType>::Ptr connKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::IterativeClosestPoint<PointType, PointType> icpChained;
        configureLoopIcp(icpChained);
        std::vector<std::pair<int, float>> chainedAdded;
        int numChainedSkipped = 0;
        for (int connIdx : connectedKeyframes) {
            if (connIdx == loopKeyPre) continue; // Already added main constraint
            loopFindNearKeyframes(connKeyframeCloud, connIdx, historyKeyframeSearchNum);
            if (connKeyframeCloud->empty()) { numChainedSkipped++; continue; }
            icpChained.setInputSource(cureKeyframeCloud);
            icpChained.setInputTarget(connKeyframeCloud);
            pcl::PointCloud<PointType>::Ptr unused_conn(new pcl::PointCloud<PointType>());
            icpChained.align(*unused_conn);
            if (!icpChained.hasConverged() || icpChained.getFitnessScore() > historyKeyframeMSEThreshold) { numChainedSkipped++; continue; }
            float connNoiseScore = icpChained.getFitnessScore();
            float scaledConnNoise = connNoiseScore * loop_closure_noise_scale;
            Eigen::Affine3f tWrongCur = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
            Eigen::Affine3f connCorrection;
            connCorrection.matrix() = icpChained.getFinalTransformation();
            Eigen::Affine3f connPoseFromAffine = connCorrection * tWrongCur;
            float cx, cy, cz, croll, cpitch, cyaw;
            pcl::getTranslationAndEulerAngles(connPoseFromAffine, cx, cy, cz, croll, cpitch, cyaw);
            gtsam::Pose3 connPoseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(croll, cpitch, cyaw), gtsam::Point3(cx, cy, cz));
            gtsam::Pose3 connPoseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[connIdx]);
            gtsam::Pose3 connPoseBetween = connPoseFrom.between(connPoseTo);
            gtsam::Vector6 Vector6Conn;
            Vector6Conn << scaledConnNoise, scaledConnNoise, scaledConnNoise,
                          scaledConnNoise, scaledConnNoise, scaledConnNoise;
            gtsam::noiseModel::Diagonal::shared_ptr connNoise = gtsam::noiseModel::Diagonal::Variances(Vector6Conn);
            loopIndexQueue.push_back(make_pair(loopKeyCur, connIdx));
            loopPoseQueue.push_back(connPoseBetween);
            loopNoiseQueue.push_back(connNoise);
            loopIndexContainer[loopKeyCur].insert(connIdx);  // persist chained closure for save/BFS
            loopClosureVariance[std::make_pair(std::min(loopKeyCur, connIdx), std::max(loopKeyCur, connIdx))] = scaledConnNoise;
            chainedAdded.push_back({connIdx, connNoiseScore});
        }
        mtx.unlock();
        ROS_INFO("[LOOP_CLOSURE] Direct: %d<->%d (MSE=%.6f)", loopKeyCur, loopKeyPre, loopClosureNoiseScore);
        for (size_t i = 0; i < chainedAdded.size(); ++i) {
            ROS_INFO("[LOOP_CLOSURE] Chained: %d<->%d (MSE=%.6f)", loopKeyCur, chainedAdded[i].first, chainedAdded[i].second);
        }
        ROS_INFO("");
    } else {
        ROS_INFO("[LOOP_CLOSURE] Direct: %d<->%d (MSE=%.6f) (no chained)", loopKeyCur, loopKeyPre, loopClosureNoiseScore);
        ROS_INFO("");
    }
}


// Loop closure detection thread
void loopClosureThread()
{
    if (loopClosureEnableFlag == false)
    {
        std::cout << "loopClosureEnableFlag   ==  false " << endl;
        return;
    }

    ros::Rate rate(loopClosureFrequency);
    while (ros::ok() && startFlag)
    {
        rate.sleep();
        performLoopClosure();
        visualizeLoopClosure(); // show loop edges in rviz
    }
}
