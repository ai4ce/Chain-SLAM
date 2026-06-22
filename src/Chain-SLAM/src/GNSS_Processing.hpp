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

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

class GnssProcess
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double time ;
    double latitude ;
    double longitude ;
    double altitude ;
    double local_E ;
    double local_N ;
    double local_U ;
    int status ;
    int service ;

    double origin_longitude;
    double origin_latitude;
    double origin_altitude;

    V3D pose_cov ;

    GnssProcess();
    ~GnssProcess();

    void InitOriginPosition(double latitude, double longitude, double altitude);
    void UpdateXYZ(double latitude, double longitude, double altitude);

    void Reverse(
      const double &local_E, const double &local_N, const double &local_U,
      double &lat, double &lon, double &alt
    );

    void set_extrinsic(const V3D &transl, const M3D &rot);
    void set_extrinsic(const V3D &transl);
    void set_extrinsic(const MD(4,4) &T);

  private:
    GeographicLib::LocalCartesian geo_converter;

    M3D Gnss_R_wrt_Lidar ;
    V3D Gnss_T_wrt_Lidar;
};

inline GnssProcess::GnssProcess()
{
    time = 0.0;
    local_E = 0.0;
    local_N = 0.0;
    local_U = 0.0;
    status = 0;
    service = 0;

    origin_longitude = 0 ;
    origin_latitude = 0;
    origin_altitude = 0;
    pose_cov = Zero3d;
    Gnss_T_wrt_Lidar = Zero3d;  
    Gnss_R_wrt_Lidar = Eye3d;
}

inline GnssProcess::~GnssProcess() {}

// init origin, WGS84 -> ENU
inline void GnssProcess::InitOriginPosition(double latitude, double longitude, double altitude)
{
    geo_converter.Reset(latitude, longitude, altitude);
    ROS_INFO("Init    Gnss  OriginPosition");   
    origin_latitude = latitude;
    origin_longitude = longitude;
    origin_altitude = altitude;
}

// compute updated ENU coordinates
inline void GnssProcess::UpdateXYZ(double latitude, double longitude, double altitude) {
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

inline void GnssProcess::Reverse(
    const double &local_E, const double &local_N, const double &local_U,
    double &lat, double &lon, double &alt
) {
    geo_converter.Reverse(local_E, local_N, local_U, lat, lon, alt);
}

inline void GnssProcess::set_extrinsic(const MD(4,4) &T)
{
  Gnss_T_wrt_Lidar = T.block<3,1>(0,3);
  Gnss_R_wrt_Lidar = T.block<3,3>(0,0);
}

inline void GnssProcess::set_extrinsic(const V3D &transl)
{
  Gnss_T_wrt_Lidar = transl;
  Gnss_R_wrt_Lidar.setIdentity();
}

inline void GnssProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Gnss_T_wrt_Lidar = transl;
  Gnss_R_wrt_Lidar = rot;
}

// Convert WGS84 (lat deg, lon deg, alt m) to UTM (easting, northing, alt) in meters (universal, run-independent)
inline void Wgs84ToUtm(double lat, double lon, double alt, double& easting, double& northing, double& out_alt) {
    int zone; bool northp;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, easting, northing);
    out_alt = alt;
}