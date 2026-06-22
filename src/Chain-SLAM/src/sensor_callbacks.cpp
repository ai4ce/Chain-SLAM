// Sensor input callbacks (lidar, IMU, GNSS, reference odom) and measurement sync.
#include "slam_common.h"


void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
        frame_index_buffer.clear();
        time_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    frame_index_buffer.push_back(lidar_frame_counter++);
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
        frame_index_buffer.clear();
        time_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());

    // feature extraction or interval sampling
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    frame_index_buffer.push_back(lidar_frame_counter++);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    // if lidar/imu time diff is large and time sync enabled, correct the imu timestamp
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp =
            ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp; // update imu time

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}


void gnss_cbk(const sensor_msgs::NavSatFixConstPtr& msg_in)
{
    //  ROS_INFO("GNSS DATA IN ");
    double timestamp = msg_in->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_gnss)
    {
        ROS_WARN("gnss loop back, clear buffer");
        gnss_buffer.clear();
    }

    last_timestamp_gnss = timestamp;

    // convert ROS NavSatFix to GeographicLib compatible GNSS message:
    gnss_data.time = msg_in->header.stamp.toSec();
    gnss_data.status = msg_in->status.status;
    gnss_data.service = msg_in->status.service;
    gnss_data.pose_cov[0] = msg_in->position_covariance[0];
    gnss_data.pose_cov[1] = msg_in->position_covariance[4];
    gnss_data.pose_cov[2] = msg_in->position_covariance[8];

    mtx_buffer.unlock();
   
    if(!gnss_inited){           // init origin position
        gnss_data.InitOriginPosition(msg_in->latitude, msg_in->longitude, msg_in->altitude) ;
        gnss_inited = true ;
    }else{                               // already initialized
        gnss_data.UpdateXYZ(msg_in->latitude, msg_in->longitude, msg_in->altitude) ;             //  WGS84 -> ENU

        Eigen::Matrix4d gnss_pose = Eigen::Matrix4d::Identity();
        gnss_pose(0,3) = gnss_data.local_N ;
        gnss_pose(1,3) = gnss_data.local_E ;
        gnss_pose(2,3) = gnss_data.local_U ;

        Eigen::Isometry3d gnss_to_lidar(Gnss_R_wrt_Lidar) ;
        gnss_to_lidar.pretranslate(Gnss_T_wrt_Lidar);
        gnss_pose  =  gnss_to_lidar  *  gnss_pose ;                    // transform gnss into lidar frame (Gnss_T_wrt_Lidar is only a rough initial guess)

        nav_msgs::Odometry gnss_data_enu ;
        // add new message to buffer:
        gnss_data_enu.header.stamp = ros::Time().fromSec(gnss_data.time);
        gnss_data_enu.pose.pose.position.x =  gnss_pose(0,3) ;
        gnss_data_enu.pose.pose.position.y =  gnss_pose(1,3) ;
        gnss_data_enu.pose.pose.position.z =  gnss_pose(2,3) ;

        gnss_data_enu.pose.pose.orientation.x =  geoQuat.x ;                // gnss orientation is unobservable, used only for visualization, taken from imu
        gnss_data_enu.pose.pose.orientation.y =  geoQuat.y;
        gnss_data_enu.pose.pose.orientation.z =  geoQuat.z;
        gnss_data_enu.pose.pose.orientation.w =  geoQuat.w;

        gnss_data_enu.pose.covariance[0] = gnss_data.pose_cov[0] ;
        gnss_data_enu.pose.covariance[7] = gnss_data.pose_cov[1] ;
        gnss_data_enu.pose.covariance[14] = gnss_data.pose_cov[2] ;

        gnss_buffer.push_back(gnss_data_enu);

        // visial gnss path in rviz:
        msg_gnss_pose.header.frame_id = "camera_init";
        msg_gnss_pose.header.stamp = ros::Time().fromSec(gnss_data.time);

        msg_gnss_pose.pose.position.x = gnss_pose(0,3) ;  
        msg_gnss_pose.pose.position.y = gnss_pose(1,3) ;
        msg_gnss_pose.pose.position.z = gnss_pose(2,3) ;

        gps_path.poses.push_back(msg_gnss_pose);

        //  save_gnss path
        PointTypePose thisPose6D;  
        thisPose6D.x = msg_gnss_pose.pose.position.x ;
        thisPose6D.y = msg_gnss_pose.pose.position.y ;
        thisPose6D.z = msg_gnss_pose.pose.position.z ;
        thisPose6D.intensity = 0;
        thisPose6D.roll = 0;
        thisPose6D.pitch = 0;
        thisPose6D.yaw = 0;
        thisPose6D.time = gnss_data.time;  // GNSS timestamp for time-based matching
        gnss_cloudKeyPoses6D->push_back(thisPose6D);
        gnss_raw_wgs84.push_back(Eigen::Vector3d(msg_in->latitude, msg_in->longitude, msg_in->altitude));
    }


}


void reference_odom_cbk(const nav_msgs::OdometryConstPtr& msg_in)
{
    if (!use_reference_init && !useXYFactor)
        return;
    
    // Extract position and orientation from reference odometry
    Eigen::Vector3d p_ref(msg_in->pose.pose.position.x, 
                          msg_in->pose.pose.position.y, 
                          msg_in->pose.pose.position.z);
    Eigen::Quaterniond q_ref(msg_in->pose.pose.orientation.w,
                             msg_in->pose.pose.orientation.x,
                             msg_in->pose.pose.orientation.y,
                             msg_in->pose.pose.orientation.z);
    
    // Update current reference odometry for debug comparison
    if (use_reference_init)
    {
        mtx_reference_init.lock();
        ref_current_position = p_ref;
        ref_current_orientation = q_ref;
        ref_current_valid = true;
        mtx_reference_init.unlock();
    }
    
    // Add to buffer for XY factor and/or ref_init (buffer as soon as IMU init is done)
    if ((useXYFactor || use_reference_init) && (imu_init_complete || p_imu->isInitialized()))
    {
        if (!imu_init_complete && p_imu->isInitialized())
        {
            imu_init_complete = true;
        }
        mtx_buffer.lock();
        reference_odom_buffer.push_back(*msg_in);
        mtx_buffer.unlock();
    }
    
    // Add to reference path for visualization (only poses >= first LiDAR frame, no pre-first-frame stem)
    if ((useXYFactor || use_reference_init) && (imu_init_complete || p_imu->isInitialized()) &&
        first_processed_lidar_time >= 0 && msg_in->header.stamp.toSec() >= first_processed_lidar_time)
    {
        if (!imu_init_complete && p_imu->isInitialized())
        {
            imu_init_complete = true;
        }
        geometry_msgs::PoseStamped msg_ref_pose;
        msg_ref_pose.header.frame_id = "camera_init";
        msg_ref_pose.header.stamp = msg_in->header.stamp;
        msg_ref_pose.pose.position.x = p_ref(0);
        msg_ref_pose.pose.position.y = p_ref(1);
        msg_ref_pose.pose.position.z = p_ref(2);
        msg_ref_pose.pose.orientation.w = q_ref.w();
        msg_ref_pose.pose.orientation.x = q_ref.x();
        msg_ref_pose.pose.orientation.y = q_ref.y();
        msg_ref_pose.pose.orientation.z = q_ref.z();
        
        ref_path.poses.push_back(msg_ref_pose);
    }
}


bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();         // oldest lidar scan
        meas.lidar_beg_time = time_buffer.front(); // earliest time
        meas.lidar_frame_idx = frame_index_buffer.empty() ? 0 : frame_index_buffer.front();

        // determine scan end time
        if (meas.lidar->points.size() <= 1) // too few points
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) // last point time < half a scan period
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000); // beg time + last point relative time
            // running average of per-scan duration
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec(); // oldest IMU time
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) // collect imu samples up to lidar scan end time
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    if (!frame_index_buffer.empty())
        frame_index_buffer.pop_front();
    lidar_pushed = false;
    return true;
}
