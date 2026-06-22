// ROS publishing helpers split out of laserMapping.cpp.
#include "slam_common.h"



/**
 * Append a pose to the odometry path
 */
void updatePath(const PointTypePose &pose_in, nav_msgs::Path &path)
{
    string odometryFrame = "camera_init";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);

    pose_stamped.header.frame_id = odometryFrame;
    pose_stamped.pose.position.x =  pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z =  pose_in.z;
    tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    path.poses.push_back(pose_stamped);
}


/**
 * Publish thisCloud and return its ROS message form
 */
sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}


/**
 * Visualize loop closure edges in rviz
 */
void visualizeLoopClosure()
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);
    string odometryFrame = "camera_init";

    if (loopIndexContainer.empty())
        return;

    visualization_msgs::MarkerArray markerArray;
    // loop closure nodes
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odometryFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // loop closure edges: direct (green) vs chained (yellow)
    visualization_msgs::Marker markerEdgeDirect;
    markerEdgeDirect.header.frame_id = odometryFrame;
    markerEdgeDirect.header.stamp = timeLaserInfoStamp;
    markerEdgeDirect.action = visualization_msgs::Marker::ADD;
    markerEdgeDirect.type = visualization_msgs::Marker::LINE_LIST;
    markerEdgeDirect.ns = "loop_edges_direct";
    markerEdgeDirect.id = 1;
    markerEdgeDirect.pose.orientation.w = 1;
    markerEdgeDirect.scale.x = 0.03;
    // markerEdgeDirect.scale.x = 0.6;
    markerEdgeDirect.color.r = 1.0;
    markerEdgeDirect.color.g = 0.0;
    markerEdgeDirect.color.b = 1.0;
    // markerEdgeDirect.color.r = 17.f / 255.f;
    // markerEdgeDirect.color.g = 85.f / 255.f;
    // markerEdgeDirect.color.b = 204.f / 255.f;
    markerEdgeDirect.color.a = 1.0;

    visualization_msgs::Marker markerEdgeChained;
    markerEdgeChained.header.frame_id = odometryFrame;
    markerEdgeChained.header.stamp = timeLaserInfoStamp;
    markerEdgeChained.action = visualization_msgs::Marker::ADD;
    markerEdgeChained.type = visualization_msgs::Marker::LINE_LIST;
    markerEdgeChained.ns = "loop_edges_chained";
    markerEdgeChained.id = 2;
    markerEdgeChained.pose.orientation.w = 1;
    markerEdgeChained.scale.x = 0.03;
    // markerEdgeChained.scale.x = 0.6;
    markerEdgeChained.color.r = 30.f / 255.f;
    markerEdgeChained.color.g = 240.f / 255.f;
    markerEdgeChained.color.b = 0.f / 255.f;
    markerEdgeChained.color.a = 1.0;

    // iterate loop closures (each cur may have multiple targets: direct + chained)
    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        for (int key_pre : it->second) {
            std::pair<int, int> canonical(std::min(key_cur, key_pre), std::max(key_cur, key_pre));
            bool isDirect = (directLoopClosureEdges.count(canonical) != 0);
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            if (isDirect) {
                markerEdgeDirect.points.push_back(p);
            } else {
                markerEdgeChained.points.push_back(p);
            }
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            if (isDirect) {
                markerEdgeDirect.points.push_back(p);
            } else {
                markerEdgeChained.points.push_back(p);
            }
        }
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdgeDirect);
    markerArray.markers.push_back(markerEdgeChained);
    pubLoopConstraintEdge.publish(markerArray);
}


void publish_frame_world(const ros::Publisher &pubLaserCloudFull)         // convert dense cloud from imu to world frame
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i],
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
        {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}


void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body)          // publish cloud in body (imu) frame
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                               &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}


void publish_effect_world(const ros::Publisher &pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld(
        new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i],
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}


void publish_map(const ros::Publisher &pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}


void publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));
}


void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    // Only add poses >= first processed LiDAR frame (no pre-first-frame stem)
    if (first_processed_lidar_time >= 0 && lidar_end_time < first_processed_lidar_time)
        return;

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
        
        //  save  unoptimized pose
         V3D rot_ang(Log( state_point.rot.toRotationMatrix())); // rotation vector
        PointTypePose thisPose6D;  
        thisPose6D.x = msg_body_pose.pose.position.x ;
        thisPose6D.y = msg_body_pose.pose.position.y ;
        thisPose6D.z = msg_body_pose.pose.position.z ;
        thisPose6D.roll = rot_ang(0) ;
        thisPose6D.pitch = rot_ang(1) ;
        thisPose6D.yaw = rot_ang(2) ;
        fastlio_unoptimized_cloudKeyPoses6D->push_back(thisPose6D);   
    }
}


void publish_path_update(const ros::Publisher pubPath)
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);
    string odometryFrame = "camera_init";
    if (pubPath.getNumSubscribers() != 0)
    {
        /*** if path is too large, the rvis will crash ***/
        static int kkk = 0;
        kkk++;
        if (kkk % 10 == 0)
        {
            // path.poses.push_back(globalPath);
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
    }
}


// publish gnss trajectory
void publish_gnss_path(const ros::Publisher pubPath)
{
    gps_path.header.stamp = ros::Time().fromSec(lidar_end_time);
    gps_path.header.frame_id = "camera_init";

    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        pubPath.publish(gps_path);
    }
}


// publish reference odometry trajectory
void publish_ref_path(const ros::Publisher pubPath)
{
    ref_path.header.stamp = ros::Time().fromSec(lidar_end_time);
    ref_path.header.frame_id = "camera_init";

    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        pubPath.publish(ref_path);
    }
}


// publish loaded map paths
void publish_loaded_path()
{
    // Publish each scene's path to its own topic
    if (loadedKeyframeCount == 0 || cloudKeyPoses6D->empty() || sceneIndexMap.empty()) {
        return;
    }
    
    // Publish each scene separately
    for (size_t sceneIdx = 0; sceneIdx < sceneIndexMap.size() && sceneIdx < pubLoadedScenePaths.size(); ++sceneIdx) {
        const SceneRange& scene = sceneIndexMap[sceneIdx];
        int sceneStart = scene.start_idx;
        int sceneEnd = scene.end_idx;
        
        // Only process scenes within loaded keyframes
        if (sceneStart >= (int)loadedKeyframeCount) {
            continue;
        }
        if (sceneEnd >= (int)loadedKeyframeCount) {
            sceneEnd = loadedKeyframeCount - 1;
        }
        
        // Build path for this scene
        loadedScenePaths[sceneIdx].poses.clear();
        for (int i = sceneStart; i <= sceneEnd && i < (int)cloudKeyPoses6D->size(); ++i) {
            updatePath(cloudKeyPoses6D->points[i], loadedScenePaths[sceneIdx]);
        }
        
        // Use current time if lidar_end_time is not set yet
        double pathTime = (lidar_end_time > 0) ? lidar_end_time : ros::Time::now().toSec();
        loadedScenePaths[sceneIdx].header.stamp = ros::Time().fromSec(pathTime);
        loadedScenePaths[sceneIdx].header.frame_id = "camera_init";
        
        // Publish with throttling to avoid overwhelming RViz
        static int publishCounter = 0;
        publishCounter++;
        if (publishCounter % 10 == 0 || loadedScenePaths[sceneIdx].poses.size() > 0) {
            pubLoadedScenePaths[sceneIdx].publish(loadedScenePaths[sceneIdx]);
        }
    }
}


/**
 * Publish the feature cloud of nearby keyframes
 */
void publishGlobalMap()
{
    /*** if path is too large, the rvis will crash ***/
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);
    string odometryFrame = "camera_init";
    if (pubLaserCloudSurround.getNumSubscribers() == 0)
        return;

    if (cloudKeyPoses3D->points.empty() == true)
        return;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
    ;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

    // kdtree search for keyframes near the latest keyframe
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
        globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    // downsample
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;
    downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
    // extract feature clouds for nearby keyframes
    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
    {
        if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]); // fast_lio only uses surfCloud
    }
    // downsample then publish
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;
    downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize);
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    publishCloud(&pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
}
