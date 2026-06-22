// LIO front-end: point transforms and the iterated-EKF measurement model.
#include "slam_common.h"


/**
 * Transform cloudIn by transformIn and return the result. Modified from LIO-SAM to account for the lidar-IMU extrinsic.
 */
pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    
    Eigen::Isometry3d T_b_lidar(state_point.offset_R_L_I);       // body->lidar extrinsic
    T_b_lidar.pretranslate(state_point.offset_T_L_I);

    Eigen::Affine3f T_w_b_ = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    Eigen::Isometry3d T_w_b ;          // world->body
    T_w_b.matrix() = T_w_b_.matrix().cast<double>();

    Eigen::Isometry3d  T_w_lidar  =  T_w_b * T_b_lidar  ;           // world->lidar

    Eigen::Isometry3d transCur = T_w_lidar;

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}


// Copy the updated pose into transformTobeMapped
void getCurPose(state_ikfom cur_state)
{
    Eigen::Vector3d eulerAngle = cur_state.rot.matrix().eulerAngles(2,1,0);        // yaw pitch roll (radians)

    // eulerAngles(2,1,0) returns ypr order
    transformTobeMapped[0] = eulerAngle(2);                // roll
    transformTobeMapped[1] = eulerAngle(1);                // pitch
    transformTobeMapped[2] = eulerAngle(0);                // yaw
    transformTobeMapped[3] = cur_state.pos(0);          // x
    transformTobeMapped[4] = cur_state.pos(1);          // y
    transformTobeMapped[5] = cur_state.pos(2);          // z
}


void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


// Transform a local point to the world frame using the current body(lidar) state
void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    // world <-- imu <-- lidar
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void RGBpointBodyToWorld(PointType const *const pi, PointType *const po)        //  lidar2world
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void test_RGBpointBodyToWorld(PointType const *const pi, PointType *const po,Eigen::Vector3d pos, Eigen::Matrix3d rotation)        //  lidar2world
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(rotation * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}


// Build the measurement Jacobian H
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++) // check whether each point's neighborhood fits a plane
    {
        PointType &point_body = feats_down_body->points[i];   // lidar frame
        PointType &point_world = feats_down_world->points[i]; // world frame

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            // Require enough neighbors and the farthest within 5m
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                                : true;
        }

        if (!point_selected_surf[i])
            continue;

        VF(4)  pabcd;          // plane params a b c d
        point_selected_surf[i] = false; // second-pass plane filtering
        // Fit local plane; returns whether enough inliers within distance threshold
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            // plane distance
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm()); // weight: 1 - 0.9 * (point-to-plane dist / point-to-lidar dist)

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2; // store point-to-plane residual in intensity
                res_last[i] = abs(pd2);             // residual (distance)
            }
        }
    }

    effct_feat_num = 0; // number of valid matches

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i]; // body-frame plane feature
            corr_normvect->points[effct_feat_num] = normvec->points[i];         // world-frame plane params
            total_residual += res_last[i];                                      // residual sum
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num; // mean residual (distance)
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i]; // lidar-frame plane feature
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I; // point in current IMU frame
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this); // skew matrix of point in IMU frame

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z); // local normal, world frame

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec); // normal rotated into IMU frame (corr_normal_I)
        V3D A(point_crossmat * C);           // residual-wrt-angle term: P(IMU)^ [R(imu<--w) * normal_w]
        if (extrinsic_est_en)
        {
            // B = lidar_p^ R(L <-- I) * corr_normal_I
            // B = lidar_p^ R(L <-- I) * R(I <-- W) * normal_W
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); // s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}
