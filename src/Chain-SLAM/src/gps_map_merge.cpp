// GPS-based map merge: align a new run to a loaded map via Open3D RANSAC + post-merge re-check.
#include "slam_common.h"


/**
 * After map merge rebuild: double-check all transformed new keyframes against loaded map
 * for additional pose-based loop closures. Adds any found to ISAM2 and prints the count.
 */
int doubleCheckPoseLoopClosuresAfterMerge()
{
    if (loadedKeyframeCount <= 0 || !mapMergeSuccessful || isam == nullptr)
        return 0;

    size_t numNew = cloudKeyPoses3D->size() - loadedKeyframeCount;
    if (numNew == 0)
        return 0;

    // Update copy clouds so loopFindNearKeyframes uses RANSAC-transformed poses
    {
        std::lock_guard<std::mutex> lock(mtx);
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    }

    // Build KD-tree of loaded keyframe positions for radius search
    pcl::PointCloud<PointType>::Ptr loadedPosesCloud(new pcl::PointCloud<PointType>());
    for (size_t i = 0; i < loadedKeyframeCount; ++i) {
        PointType p;
        p.x = cloudKeyPoses3D->points[i].x;
        p.y = cloudKeyPoses3D->points[i].y;
        p.z = cloudKeyPoses3D->points[i].z;
        loadedPosesCloud->push_back(p);
    }
    if (loadedPosesCloud->empty() || loadedPosesCloud->size() < 2)
        return 0;  // FLANN middleSplit requires >= 2 points

    pcl::KdTreeFLANN<PointType> kdtreeLoaded;
    kdtreeLoaded.setInputCloud(loadedPosesCloud);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    configureLoopIcp(icp);

    int numAdded = 0;
    std::vector<std::tuple<int, int, gtsam::Pose3, gtsam::noiseModel::Diagonal::shared_ptr>> newFactors;
    int skipRemaining = 0;

    for (size_t ii = 0; ii < numNew; ++ii) {
        int loopKeyCur = (int)loadedKeyframeCount + (int)ii;
        if (loopKeyCur >= (int)surfCloudKeyFrames.size())
            break;

        if (skipRemaining > 0) {
            skipRemaining--;
            continue;
        }

        bool addedForThisKf = false;

        // 3D pose-based
        PointType queryPoint = cloudKeyPoses3D->points[loopKeyCur];
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeLoaded.radiusSearch(queryPoint, historyKeyframeSearchRadius, pointSearchInd, pointSearchSqDis, 0);

        if (!pointSearchInd.empty()) {
            int loopKeyPre = pointSearchInd[0];
            if (loopKeyPre < (int)surfCloudKeyFrames.size()) {
                std::pair<int, int> canonicalPair(std::min(loopKeyCur, loopKeyPre), std::max(loopKeyCur, loopKeyPre));
                if (loopClosurePairsAdded.find(canonicalPair) == loopClosurePairsAdded.end()) {
                    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
                    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
                    loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0, 0.0f);
                    loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, 0.0f);

                    if (!cureKeyframeCloud->empty() && !prevKeyframeCloud->empty()) {
                        icp.setInputSource(cureKeyframeCloud);
                        icp.setInputTarget(prevKeyframeCloud);
                        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
                        icp.align(*unused_result);

                        if (icp.hasConverged() && icp.getFitnessScore() <= historyKeyframeMSEThreshold) {
                            Eigen::Affine3f correctionLidarFrame;
                            correctionLidarFrame.matrix() = icp.getFinalTransformation();
                            Eigen::Affine3f tWrong = pclPointToAffine3f(cloudKeyPoses6D->points[loopKeyCur]);
                            Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
                            float x, y, z, roll, pitch, yaw;
                            pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
                            gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
                            gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[loopKeyPre]);
                            float noiseScore = icp.getFitnessScore();
                            float scaledNoiseScore = noiseScore * loop_closure_noise_scale;
                            gtsam::Vector6 Vector6;
                            Vector6 << scaledNoiseScore, scaledNoiseScore, scaledNoiseScore, scaledNoiseScore, scaledNoiseScore, scaledNoiseScore;
                            gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
                            newFactors.push_back(std::make_tuple(loopKeyCur, loopKeyPre, poseFrom.between(poseTo), constraintNoise));
                            loopClosurePairsAdded.insert(canonicalPair);
                            loopIndexContainer[loopKeyCur].insert(loopKeyPre);
                            directLoopClosureEdges.insert(canonicalPair);
                            loopClosureVariance[canonicalPair] = scaledNoiseScore;
                            numAdded++;
                            addedForThisKf = true;
                            skipRemaining = postMergeClosureSkipFrames;
                            float kfDist = pointDistance(cloudKeyPoses3D->points[loopKeyCur], cloudKeyPoses3D->points[loopKeyPre]);
                            ROS_INFO("[MAP_MERGE_DOUBLE_CHECK] Factor %d (3D): new KF %d <-> loaded KF %d, dist=%.2fm, mse=%.4f", numAdded, loopKeyCur, loopKeyPre, kfDist, noiseScore);
                        }
                    }
                }
            }
        }

        // Failed 3D closest distance (pose-based): for debugging when we fall back to 2D
        float closestDist3D = (pointSearchInd.empty() ? -1.0f : sqrt(pointSearchSqDis[0]));

        // 2D fallback when 3D failed
        if (!addedForThisKf && loopClosure2DEnabled && !gnss_raw_wgs84.empty() && loadedGnssCloud->size() > 0) {
            double kfTime = cloudKeyPoses6D->points[loopKeyCur].time;
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
                double curE, curN, curA;
                Wgs84ToUtm(gnss_raw_wgs84[bestGnssIdx].x(), gnss_raw_wgs84[bestGnssIdx].y(), gnss_raw_wgs84[bestGnssIdx].z(), curE, curN, curA);
                PointType queryPt;
                queryPt.x = curE; queryPt.y = curN; queryPt.z = curA;

                std::vector<int> nearestInd(1);
                std::vector<float> nearestSqDis(1);
                kdtreeLoadedGnss->nearestKSearch(queryPt, 1, nearestInd, nearestSqDis);
                float closestDist2D = (nearestInd.empty() ? -1.0f : sqrt(nearestSqDis[0]));
                int closestLoadedKf = (nearestInd.empty() ? -1 : (int)loadedGnssCloud->points[nearestInd[0]].intensity);
                if (closestLoadedKf >= (int)loadedKeyframeCount || fabs(loadedKeyframeGnssPositions[closestLoadedKf].x()) >= 1e9)
                    closestLoadedKf = -1;

                std::vector<int> gnssSearchInd;
                std::vector<float> gnssSearchSqDis;
                kdtreeLoadedGnss->radiusSearch(queryPt, loopClosure2DDistanceThreshold, gnssSearchInd, gnssSearchSqDis, 0);

                int loopKeyPre = -1;
                float bestDistSq = loopClosure2DDistanceThreshold * loopClosure2DDistanceThreshold;
                for (size_t i = 0; i < gnssSearchInd.size(); ++i) {
                    int idx = (int)loadedGnssCloud->points[gnssSearchInd[i]].intensity;
                    if (idx >= (int)loadedKeyframeCount) continue;
                    if (fabs(loadedKeyframeGnssPositions[idx].x()) >= 1e9) continue;
                    std::pair<int, int> canonicalPair(std::min(loopKeyCur, idx), std::max(loopKeyCur, idx));
                    if (loopClosurePairsAdded.find(canonicalPair) != loopClosurePairsAdded.end()) continue;
                    if (gnssSearchSqDis[i] < bestDistSq) {
                        bestDistSq = gnssSearchSqDis[i];
                        loopKeyPre = idx;
                    }
                }

                int showKf = (loopKeyPre >= 0) ? loopKeyPre : closestLoadedKf;
                ROS_INFO("[MAP_MERGE_DOUBLE_CHECK] 2D fallback (3D failed): cur KF %d, closest 3D=%.2fm, closest 2D=%.2fm, loaded KF %d, within_2D_thresh=%d",
                         loopKeyCur, closestDist3D >= 0.0f ? closestDist3D : -1.0f, closestDist2D, showKf, loopKeyPre >= 0 ? 1 : 0);

                if (loopKeyPre >= 0) {
                    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
                    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
                    loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, historyKeyframeSearchNum, gpsMapMergeVoxelSize);
                    loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, gpsMapMergeVoxelSize);

                    if (!cureKeyframeCloud->empty() && !prevKeyframeCloud->empty()) {
                        Eigen::Matrix4f correction2D;
                        float fitness2D;
                        if (try2DMatchTranslationOnly(loopKeyCur, loopKeyPre, cureKeyframeCloud, prevKeyframeCloud, &correction2D, &fitness2D)) {
                            Eigen::Affine3f correctionLidarFrame;
                            correctionLidarFrame.matrix() = correction2D;
                            Eigen::Affine3f tWrong = pclPointToAffine3f(cloudKeyPoses6D->points[loopKeyCur]);
                            Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
                            float x, y, z, roll, pitch, yaw;
                            pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
                            gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
                            gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[loopKeyPre]);
                            float noiseScore = fitness2D;
                            float scaledNoiseScore = noiseScore * loop_closure_noise_scale;
                            gtsam::Vector6 Vector6;
                            Vector6 << scaledNoiseScore, scaledNoiseScore, scaledNoiseScore, scaledNoiseScore, scaledNoiseScore, scaledNoiseScore;
                            gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
                            std::pair<int, int> canonicalPair(std::min(loopKeyCur, loopKeyPre), std::max(loopKeyCur, loopKeyPre));
                            newFactors.push_back(std::make_tuple(loopKeyCur, loopKeyPre, poseFrom.between(poseTo), constraintNoise));
                            loopClosurePairsAdded.insert(canonicalPair);
                            loopIndexContainer[loopKeyCur].insert(loopKeyPre);
                            directLoopClosureEdges.insert(canonicalPair);
                            loopClosureVariance[canonicalPair] = scaledNoiseScore;
                            numAdded++;
                            skipRemaining = postMergeClosureSkipFrames;
                            float kfDist = pointDistance(cloudKeyPoses3D->points[loopKeyCur], cloudKeyPoses3D->points[loopKeyPre]);
                            ROS_INFO("[MAP_MERGE_DOUBLE_CHECK] Factor %d (2D): new KF %d <-> loaded KF %d, dist=%.2fm, mse=%.4f", numAdded, loopKeyCur, loopKeyPre, kfDist, fitness2D);
                        }
                    }
                }
            }
        }
    }

    // Add factors to graph and run optimization
    for (const auto& t : newFactors) {
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(std::get<0>(t), std::get<1>(t), std::get<2>(t), std::get<3>(t)));
    }

    if (numAdded > 0) {
        isam->update(gtSAMgraph, gtsam::Values());
        for (int _ = 0; _ < 5; ++_)
            isam->update();
        isamCurrentEstimate = isam->calculateBestEstimate();
        gtSAMgraph.resize(0);
        aLoopIsClosed = true;
    }

    ROS_INFO("[MAP_MERGE_DOUBLE_CHECK] Pose-based loop closure after rebuild: added %d factors.", numAdded);
    return numAdded;
}


/**
 * GPS-based map merge: when current frame's GNSS is close to a loaded keyframe's GNSS,
 * add loop closure to align the new map with the preloaded map.
 * On success, outputs loopKeyCur and loopKeyPre for pose-based loop closure (ICP + BetweenFactor).
 */
bool performGpsMapMerge(int* out_loopKeyCur, int* out_loopKeyPre, std::unique_lock<std::mutex>* lock_map_merge)
{
    if (mapMergeSuccessful) {
        return false;  // Already merged once, do not attempt again
    }
    if (!gpsMapMergeEnabled || loadedKeyframeCount <= 0 || loadedKeyframeGnssPositions.empty()) {
        if (!gpsMapMergeEnabled)
            ROS_INFO("[GPS_MAP_MERGE] Skipped: gpsMapMerge disabled");
        else if (loadedKeyframeCount <= 0)
            ROS_INFO("[GPS_MAP_MERGE] Skipped: no loaded keyframes");
        else
            ROS_INFO("[GPS_MAP_MERGE] Skipped: no loaded GNSS data");
        return false;
    }
    
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    if (loopKeyCur < (int)loadedKeyframeCount) {
        ROS_INFO("[GPS_MAP_MERGE] Skipped: no new keyframes yet (current %d, loaded %zu)", loopKeyCur, loadedKeyframeCount);
        return false;
    }
    
    if (loopIndexContainer.find(loopKeyCur) != loopIndexContainer.end()) {
        ROS_INFO("[GPS_MAP_MERGE] Skipped: keyframe %d already has loop closure", loopKeyCur);
        return false;
    }
    
    if (loopKeyCur == lastGpsMergeAttemptedKeyframe) {
        ROS_DEBUG("[GPS_MAP_MERGE] Skipped: keyframe %d already tested", loopKeyCur);
        return false;
    }
    lastGpsMergeAttemptedKeyframe = loopKeyCur;
    
    // Get current keyframe GNSS: find closest by time, use raw WGS84 and convert to UTM (universal meters)
    double kfTime = copy_cloudKeyPoses6D->points[loopKeyCur].time;
    double bestTimeDiff = 1e9;
    int bestGnssIdx = -1;
    for (size_t j = 0; j < gnss_cloudKeyPoses6D->size(); ++j) {
        double diff = fabs(gnss_cloudKeyPoses6D->points[j].time - kfTime);
        if (diff < bestTimeDiff && diff < 0.5) {
            bestTimeDiff = diff;
            bestGnssIdx = j;
        }
    }
    if (bestGnssIdx < 0 || bestGnssIdx >= (int)gnss_raw_wgs84.size()) {
        ROS_INFO("[GPS_MAP_MERGE] No valid GNSS found for current frame");
        return false;  // No valid GNSS for current keyframe
    }
    // Convert WGS84 to UTM (universal meters, run-independent)
    Eigen::Vector3d rawWgs84 = gnss_raw_wgs84[bestGnssIdx];
    double altForUtm = ((rawWgs84.z() != rawWgs84.z()) || !std::isfinite(rawWgs84.z())) ? 0.0 : rawWgs84.z();  // NaN/Inf -> 0
    double curEasting, curNorthing, curAlt;
    Wgs84ToUtm(rawWgs84.x(), rawWgs84.y(), altForUtm, curEasting, curNorthing, curAlt);
    Eigen::Vector3d curGnss(curEasting, curNorthing, curAlt);
    // Sanitize query point: PCL radiusSearch asserts on NaN/Inf
    if ((curGnss.x() != curGnss.x()) || (curGnss.y() != curGnss.y()) || (curGnss.z() != curGnss.z()) ||
        !std::isfinite(curGnss.x()) || !std::isfinite(curGnss.y()) || !std::isfinite(curGnss.z())) {
        ROS_INFO("[GPS_MAP_MERGE] Skipped: current GNSS has NaN/Inf (lat=%.6f lon=%.6f alt=%.2f)", rawWgs84.x(), rawWgs84.y(), rawWgs84.z());
        return false;
    }
    
    ROS_INFO("[GPS_MAP_MERGE] Searching for potential GPS match (keyframe %d, loaded=%zu)", loopKeyCur, loadedKeyframeCount);
    // Find closest loaded keyframe by GNSS distance in UTM (same frame as curGnss)
    PointType queryPoint;
    queryPoint.x = curGnss.x();
    queryPoint.y = curGnss.y();
    queryPoint.z = curGnss.z();
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeLoadedGnss->radiusSearch(queryPoint, gpsMapMergeDistanceThreshold, pointSearchInd, pointSearchSqDis, 0);
    int bestLoadedIdx = -1;
    float bestDistSq = gpsMapMergeDistanceThreshold * gpsMapMergeDistanceThreshold;
    for (size_t i = 0; i < pointSearchInd.size(); ++i) {
        int j = pointSearchInd[i];
        const Eigen::Vector3d& loadedGnss = loadedKeyframeGnssPositions[j];
        if (fabs(loadedGnss.x()) >= 1e9)
            continue;
        if (pointSearchSqDis[i] < bestDistSq) {
            bestDistSq = pointSearchSqDis[i];
            bestLoadedIdx = j;
        }
    }
    // DEBUG print if no match
    if (bestLoadedIdx < 0) {
        std::vector<int> nearestInd(1);
        std::vector<float> nearestSqDis(1);
        kdtreeLoadedGnss->nearestKSearch(queryPoint, 1, nearestInd, nearestSqDis);
        if (!nearestInd.empty() && nearestInd[0] < (int)loadedKeyframeGnssPositions.size()) {
            const Eigen::Vector3d& closestGnss = loadedKeyframeGnssPositions[nearestInd[0]];
            ROS_INFO("[GPS_MAP_MERGE] No match within %.1fm. Current kf %d UTM (%.1f, %.1f, %.1f), closest kf %d UTM (%.1f, %.1f, %.1f), distance %.2fm",
                     gpsMapMergeDistanceThreshold, loopKeyCur, curGnss.x(), curGnss.y(), curGnss.z(),
                     nearestInd[0], closestGnss.x(), closestGnss.y(), closestGnss.z(), sqrt(nearestSqDis[0]));
        } else {
            ROS_INFO("[GPS_MAP_MERGE] No match within %.1fm. Current kf %d UTM (%.1f, %.1f, %.1f), distance N/A",
                     gpsMapMergeDistanceThreshold, loopKeyCur, curGnss.x(), curGnss.y(), curGnss.z());
        }
        return false;
    }
    
    const Eigen::Vector3d& foundGnss = loadedKeyframeGnssPositions[bestLoadedIdx];
    ROS_INFO("[GPS_MAP_MERGE] Found GNSS within %.1fm radius. Current kf %d UTM (%.1f, %.1f, %.1f), loaded kf %d UTM (%.1f, %.1f, %.1f), distance %.2fm",
             gpsMapMergeDistanceThreshold, loopKeyCur, curGnss.x(), curGnss.y(), curGnss.z(),
             bestLoadedIdx, foundGnss.x(), foundGnss.y(), foundGnss.z(), sqrt(bestDistSq));
    
    int loopKeyPre = bestLoadedIdx;
    
    // Extract clouds (stack both curr and prev with +-historyKeyframeSearchNum frames for dense ISS/FPFH/RANSAC)
    // Use gpsMapMergeVoxelSize for downsampling (pose loop closure uses mappingSurfLeafSize via voxelLeafSize=0)
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, historyKeyframeSearchNum, gpsMapMergeVoxelSize);
    loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum, gpsMapMergeVoxelSize);
    if (cureKeyframeCloud->empty() || prevKeyframeCloud->empty()) {
        ROS_INFO("[GPS_MAP_MERGE] Failed: empty keyframe clouds (current %zu pts, prev %zu pts)", cureKeyframeCloud->size(), prevKeyframeCloud->size());
        return false;
    }

    Eigen::Vector4f curCentroid, prevCentroid;
    pcl::compute3DCentroid(*cureKeyframeCloud, curCentroid);
    pcl::compute3DCentroid(*prevKeyframeCloud, prevCentroid);
    ROS_INFO("[GPS_MAP_MERGE] Comparing keyframe %d (cur, %zu pts) vs %d (loaded, %zu pts)", loopKeyCur, cureKeyframeCloud->size(), loopKeyPre, prevKeyframeCloud->size());
    ROS_INFO("[GPS_MAP_MERGE] Cur center (%.2f, %.2f, %.2f) | Loaded center (%.2f, %.2f, %.2f)", curCentroid.x(), curCentroid.y(), curCentroid.z(), prevCentroid.x(), prevCentroid.y(), prevCentroid.z());

    // // TEMP: save GPS-matched clouds for debugging (remove when done)
    // {
    //     string debugDir = string(ROOT_DIR) + "Log/gps_merge_debug";
    //     system(("mkdir -p " + debugDir).c_str());
    //     string curFile = debugDir + "/gps_merge_cur_kf" + to_string(loopKeyCur) + "_vs_" + to_string(loopKeyPre) + ".pcd";
    //     string prevFile = debugDir + "/gps_merge_prev_kf" + to_string(loopKeyCur) + "_vs_" + to_string(loopKeyPre) + ".pcd";
    //     pcl::io::savePCDFileBinary(curFile, *cureKeyframeCloud);
    //     pcl::io::savePCDFileBinary(prevFile, *prevKeyframeCloud);
    //     ROS_INFO("[GPS_MAP_MERGE] Saved debug clouds to %s", debugDir.c_str());
    // }

    // Pipeline: [ISS keypoints] -> FPFH descriptor -> RANSAC global registration -> ICP refinement
    // Use PointXYZ throughout (avoid PointXYZINormal) to prevent free(): invalid pointer

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cureKeyframeCloud, *cloud_src);
    pcl::copyPointCloud(*prevKeyframeCloud, *cloud_tgt);
    if (cloud_src->size() < 50 || cloud_tgt->size() < 50) {
        ROS_INFO("[GPS_MAP_MERGE] Failed: clouds too small");
        return false;
    }

    ROS_INFO("[GPS_MAP_MERGE] FPFH on full clouds (src=%zu, tgt=%zu)", cloud_src->size(), cloud_tgt->size());

    // Open3D pipeline: normals -> FPFH -> RANSAC
    auto o3d_cloud_src = pclToOpen3D(cloud_src);
    auto o3d_cloud_tgt = pclToOpen3D(cloud_tgt);
    double fpfh_radius = static_cast<double>(gpsMapMergeFpfhRadius);
    int fpfh_max_nn = (gpsMapMergeFpfhMaxNn > 0) ? gpsMapMergeFpfhMaxNn : 100;

    o3d_cloud_src->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(gpsMapMergeNormalRadius * 2.0, 30));
    o3d_cloud_tgt->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(gpsMapMergeNormalRadius * 2.0, 30));

    auto fpfh_src = open3d::pipelines::registration::ComputeFPFHFeature(
        *o3d_cloud_src, open3d::geometry::KDTreeSearchParamHybrid(fpfh_radius, fpfh_max_nn));
    auto fpfh_tgt = open3d::pipelines::registration::ComputeFPFHFeature(
        *o3d_cloud_tgt, open3d::geometry::KDTreeSearchParamHybrid(fpfh_radius, fpfh_max_nn));
    auto o3d_src = o3d_cloud_src;
    auto o3d_tgt = o3d_cloud_tgt;

    double max_corr_dist = static_cast<double>(gpsMapMergeRansacInlierDist);
    open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength checker_edge(0.9);
    open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance checker_dist(max_corr_dist);
    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> checkers;
    checkers.push_back(std::cref(checker_edge));
    checkers.push_back(std::cref(checker_dist));

    auto result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
        *o3d_src, *o3d_tgt, *fpfh_src, *fpfh_tgt, gpsMapMergeMutualFilter, max_corr_dist,
        open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
        3, checkers,
        open3d::pipelines::registration::RANSACConvergenceCriteria(gpsMapMergeRansacMaxIterations, 0.999));

    size_t n_inliers = result.correspondence_set_.size();
    ROS_INFO("[GPS_MAP_MERGE] RANSAC (Open3D): %zu inliers, fitness %.4f. Keyframes %d vs %d",
             n_inliers, result.fitness_, loopKeyCur, loopKeyPre);

    Eigen::Matrix4f ransacTransform = result.transformation_.cast<float>();
    ROS_INFO("[GPS_MAP_MERGE] RANSAC transform:\n[%.4f %.4f %.4f %.4f]\n[%.4f %.4f %.4f %.4f]\n[%.4f %.4f %.4f %.4f]\n[%.4f %.4f %.4f %.4f]",
             ransacTransform(0,0), ransacTransform(0,1), ransacTransform(0,2), ransacTransform(0,3),
             ransacTransform(1,0), ransacTransform(1,1), ransacTransform(1,2), ransacTransform(1,3),
             ransacTransform(2,0), ransacTransform(2,1), ransacTransform(2,2), ransacTransform(2,3),
             ransacTransform(3,0), ransacTransform(3,1), ransacTransform(3,2), ransacTransform(3,3));

    // Reject near-identity transform (no meaningful alignment)
    {
        Eigen::Matrix3f R = ransacTransform.block<3,3>(0,0);
        Eigen::Vector3f t = ransacTransform.block<3,1>(0,3);
        Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
        float rotDiff = (R - I).norm();
        float transNorm = t.norm();
        if (rotDiff < gpsMapMergeRansacMinRotationRad && transNorm < gpsMapMergeRansacMinTranslationM) {
            ROS_INFO("[GPS_MAP_MERGE] Rejected: RANSAC transform too close to identity (rot_diff=%.4f < %.4f, trans=%.4f < %.4f)", rotDiff, gpsMapMergeRansacMinRotationRad, transNorm, gpsMapMergeRansacMinTranslationM);
            return false;
        }
    }

    if (result.fitness_ < static_cast<double>(gpsMapMergeRansacFitnessThreshold)) {
        ROS_INFO("[GPS_MAP_MERGE] Failed: RANSAC fitness %.4f < threshold %.4f. Keyframes %d vs %d", result.fitness_, gpsMapMergeRansacFitnessThreshold, loopKeyCur, loopKeyPre);
        return false;
    }

    ROS_INFO("[GPS_MAP_MERGE] Fitness %.4f >= %.4f: applying RANSAC transform directly to all new-map poses (no factor).", result.fitness_, gpsMapMergeRansacFitnessThreshold);

    // Block main thread only when we have successful RANSAC; hold until caller finishes ICP and push
    if (lock_map_merge)
        lock_map_merge->lock();

    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame.matrix() = ransacTransform;

    // Apply same transform to unoptimized trajectory (path and fastlio poses)
    for (size_t i = 0; i < fastlio_unoptimized_cloudKeyPoses6D->size(); ++i) {
        ROS_INFO("[RANSAC_DEBUG] fastlio_unoptimized: transforming pose %zu", i);
        Eigen::Affine3f tWrong = pclPointToAffine3f(fastlio_unoptimized_cloudKeyPoses6D->points[i]);
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        float xi, yi, zi, rolli, pitchi, yawi;
        pcl::getTranslationAndEulerAngles(tCorrect, xi, yi, zi, rolli, pitchi, yawi);
        fastlio_unoptimized_cloudKeyPoses6D->points[i].x = xi;
        fastlio_unoptimized_cloudKeyPoses6D->points[i].y = yi;
        fastlio_unoptimized_cloudKeyPoses6D->points[i].z = zi;
        fastlio_unoptimized_cloudKeyPoses6D->points[i].roll = rolli;
        fastlio_unoptimized_cloudKeyPoses6D->points[i].pitch = pitchi;
        fastlio_unoptimized_cloudKeyPoses6D->points[i].yaw = yawi;
    }
    for (size_t i = 0; i < path.poses.size(); ++i) {
        ROS_INFO("[RANSAC_DEBUG] path.poses: transforming pose %zu", i);
        auto& p = path.poses[i].pose;
        tf::Quaternion qIn(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(qIn).getRPY(roll, pitch, yaw);
        Eigen::Affine3f tWrong = pcl::getTransformation(p.position.x, p.position.y, p.position.z, roll, pitch, yaw);
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        float xi, yi, zi, rolli, pitchi, yawi;
        pcl::getTranslationAndEulerAngles(tCorrect, xi, yi, zi, rolli, pitchi, yawi);
        p.position.x = xi; p.position.y = yi; p.position.z = zi;
        tf::Quaternion qOut = tf::createQuaternionFromRPY(rolli, pitchi, yawi);
        p.orientation.x = qOut.x(); p.orientation.y = qOut.y(); p.orientation.z = qOut.z(); p.orientation.w = qOut.w();
    }

    // apply ransac transform to optimized poses
    for (int i = (int)loadedKeyframeCount; i < (int)cloudKeyPoses6D->size(); ++i) {
        ROS_INFO("[RANSAC_DEBUG] cloudKeyPoses6D/3D: transforming keyframe %d", i);
        Eigen::Affine3f tWrong = pclPointToAffine3f(cloudKeyPoses6D->points[i]);
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        float xi, yi, zi, rolli, pitchi, yawi;
        pcl::getTranslationAndEulerAngles(tCorrect, xi, yi, zi, rolli, pitchi, yawi);
        cloudKeyPoses6D->points[i].x = xi;
        cloudKeyPoses6D->points[i].y = yi;
        cloudKeyPoses6D->points[i].z = zi;
        cloudKeyPoses6D->points[i].roll = rolli;
        cloudKeyPoses6D->points[i].pitch = pitchi;
        cloudKeyPoses6D->points[i].yaw = yawi;
        cloudKeyPoses3D->points[i].x = xi;
        cloudKeyPoses3D->points[i].y = yi;
        cloudKeyPoses3D->points[i].z = zi;
    }
    // globalPath is built from cloudKeyPoses6D (already RANSAC-transformed above); no separate transform needed
    globalPath.poses.clear();
    if (loadedKeyframeCount > 0) {
        for (int i = 0; i < (int)loadedKeyframeCount && i < (int)cloudKeyPoses6D->size(); ++i)
            updatePath(cloudKeyPoses6D->points[i]);
        globalPath.poses.clear();
    }
    for (int i = (int)loadedKeyframeCount; i < (int)cloudKeyPoses6D->size(); ++i)
        updatePath(cloudKeyPoses6D->points[i]);

    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    
    gpsRansacTransform = correctionLidarFrame;
    mapMergeSuccessful = true;
    
    if (out_loopKeyCur) *out_loopKeyCur = loopKeyCur;
    if (out_loopKeyPre) *out_loopKeyPre = loopKeyPre;
    
    ROS_INFO("[GPS_MAP_MERGE] Applied transform to %d new keyframes (indices %zu..%zu). Proceeding to pose-based loop closure (ICP) for kf %d <-> %d. GNSS dist=%.2fm, fitness=%.4f",
             (int)cloudKeyPoses6D->size() - (int)loadedKeyframeCount, loadedKeyframeCount, cloudKeyPoses6D->size() - 1, loopKeyCur, loopKeyPre, sqrt(bestDistSq), result.fitness_);
    
    return true;
}
