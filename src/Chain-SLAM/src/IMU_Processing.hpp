#pragma once

#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"

/// *************Preconfiguration

inline const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_max_ini_count(int max_count);
  void set_skip_deskew(bool skip) { skip_deskew_ = skip; }
  bool isInitialized() const { return !imu_need_init_; }
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;
  V3D cov_acc; //vector
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;
  int max_ini_count_ = 20;  // Maximum IMU initialization iterations (configurable)

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;
  sensor_msgs::ImuConstPtr last_imu_;
  deque<sensor_msgs::ImuConstPtr> v_imu_;
  vector<Pose6D> IMUpose; // (time, acc, angular vel, vel, pos, rotation matrix)
  vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc; // mean acceleration obtained during initialization
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_iter_num = 1; // number of IMU samples processed during initialization
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
  bool   skip_deskew_ = false;  // true: skip point undistortion (pre-deskewed input)
};

inline ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_iter_num = 1;
  Q = process_noise_cov();
  cov_acc       = V3D(0.1, 0.1, 0.1);
  cov_gyr       = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::Imu());
}

inline ImuProcess::~ImuProcess() {}

inline void ImuProcess::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  angvel_last       = Zero3d;
  imu_need_init_    = true;
  start_timestamp_  = -1;
  init_iter_num     = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

inline void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

inline void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

inline void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

inline void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

inline void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_scale = scaler;
}

inline void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyr = b_g;
}

inline void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

inline void ImuProcess::set_max_ini_count(int max_count)
{
  max_ini_count_ = max_count;
}

inline void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

  V3D cur_acc, cur_gyr;

  if (b_first_frame_)     // grab the first IMU frame
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z; // init acc mean with the first sample
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z; // init gyro mean with the first sample
    first_lidar_time = meas.lidar_beg_time; // absolute lidar start time
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N; // update acc mean
    mean_gyr      += (cur_gyr - mean_gyr) / N; // update gyro mean

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N); // update acc covariance
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N); // update gyro covariance

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;
  }
  state_ikfom init_state = kf_state.get_x(); // initial state
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2); // gravity normalized by acc mean magnitude (negative, on S2 manifold)

  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr; // init gyro bias with the mean angular velocity
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;        //   t_lidar_imu extrinsic translation
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;       //   R_lidar_imu extrinsic rotation
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity(); // initialize covariance below
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001; // extrinsic r
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;// extrinsic t
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;//bias g
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;//bias a
  init_P(21,21) = init_P(22,22) = 0.00001; // S2 (gravity)
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back(); // record the last imu
}

// IMU backward propagation and point cloud distortion correction
inline void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu; // imu data sequence
  v_imu.push_front(last_imu_); // prepend the previous imu sample
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;

  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list); // sort points by time ascending (old to new)
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  // set the initial relative state (time relative to integration start, prev acc, prev angular vel, vel, pos, rotation matrix)
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;// input: acc and angular velocity
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    
    if (tail->header.stamp.toSec() < last_lidar_end_time_)    continue;

    // mean of acc and angular velocity over samples j and j+1
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    // rescale acceleration to the gravity scale using the init acc mean
    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(head->header.stamp.toSec() < last_lidar_end_time_) // head imu time before previous lidar end time
    {
      dt = tail->header.stamp.toSec() - last_lidar_end_time_; // tail imu time minus previous lidar end time
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();// time diff between adjacent imu samples
    }

    // record input mean and covariance (measurements)
    in.acc = acc_avr;
    in.gyro = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    // forward propagation (dt between imu samples, covariance matrix, input)
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg; // w = w - bg
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);// a(world) = r * ( a(body) - ba)
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];// a(world) + g (negative)
    }
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;// time of sample m+1 relative to lidar start
    // save pose at sample m+1, prev/current imu mean (world frame), v, p, r
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);// time diff between lidar end and imu end
  kf_state.predict(dt, Q, in); // in is the last imu sample; propagate to the later of lidar-end / imu-end

  imu_state = kf_state.get_x();// imu pose at the later of lidar-end / imu-end
  last_imu_ = meas.imu.back(); // keep the last imu as previous, used to start the next propagation
  last_lidar_end_time_ = pcl_end_time;// keep cloud end time as previous frame end

  /*** undistort each lidar point (backward propagation) - skip if pre-deskewed ***/
  if (!skip_deskew_)
  {
  auto it_pcl = pcl_out.points.end() - 1;// iterate points back to front (time descending); already sorted ascending
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) // iterate imu poses back to front
  {
    auto head = it_kp - 1;// previous imu pose, j-1
    auto tail = it_kp;// next imu pose, j
    // p, v, r of imu at time j-1
    R_imu<<MAT_FROM_ARRAY(head->rot);
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);
    pos_imu<<VEC_FROM_ARRAY(head->pos);

    // acc and angular velocity at time j
    acc_imu<<VEC_FROM_ARRAY(tail->acc);
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);

    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --) // head->offset_time: imu time at j-1 relative to propagation start
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;// time diff relative to previous imu sample

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      M3D R_i(R_imu * Exp(angvel_avr, dt));//R(global <-- i)
      // global <-- head <-- i

      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);// point i in lidar frame
      // T_ei: translation from imu end position to imu position at time i (world frame), i.e. T_i - T_e
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);//T(i <-- end)

      /*** transform: end-time lidar <-- end-time imu (body) <-- corrected to global <-- end-time imu (body) <-- end-time lidar
       * distortion correction done in the imu (body) frame, may be slightly inaccurate
       * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I): point i from lidar to imu frame using the end-time extrinsic
       * (R_i * (P) + T_i): distortion-correct point i into the global frame
       * (imu_state.rot.conjugate() * (P - T_e): rotate point i into the end-time imu frame
       * imu_state.offset_R_L_I.conjugate() * (P - imu_state.offset_T_L_I): transform into the end-time lidar frame
       ***/
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * \
      (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!
      
      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
  }
}

inline void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  if(meas.imu.empty()) {return;};
  ROS_ASSERT(meas.lidar != nullptr);

  // imu initialization; distortion correction only runs after init is done
  if (imu_need_init_)
  {
      /** 1. initializing the gravity, gyro bias, acc and gyro covariance
        * 2. normalize the acceleration measurenments to unit gravity **/
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;
    
    last_imu_   = meas.imu.back(); // record the last imu sample

    state_ikfom imu_state = kf_state.get_x(); // get the state
    if (init_iter_num > max_ini_count_) // enough imu samples collected for init
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
      ROS_INFO("IMU Initial Done");
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(GetLogFilePath("imu.txt"),ios::out);
    }

    return;
  }

  // distortion correction
  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}