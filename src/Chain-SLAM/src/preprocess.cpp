#include "preprocess.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
  :feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS   = 6;
  SCAN_RATE = 10;
  group_size = 8;
  disA = 0.01;
  disA = 0.1; // B? todo
  p2l_ratio = 225;
  limit_maxmid =6.25;
  limit_midmin =6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit/180*M_PI);
  jump_down_limit = cos(jump_down_limit/180*M_PI);
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{  
  avia_handler(msg);
  *pcl_out = pl_surf; // store the sampled points
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  case RS128:
      rs_handler(msg);
      break;

  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg->point_num;
  // cout<<"plsie: "<<plsize<<endl;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  // clear cached clouds and reserve space
  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  // feature extraction
  if (feature_enabled)
  {
      // split the cloud by scan line
    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

        bool is_new = false;
        // skip points too close to the previous one (poor for feature extraction)
        if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }
    static int count = 0;
    static double time = 0.0;
    count ++;
    double t0 = omp_get_wtime();
    for(int j=0; j<N_SCANS; j++)
    {
      if(pl_buff[j].size() <= 5) continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j];// current scan-line cloud
      plsize = pl.size();
      vector<orgtype> &types = typess[j];// per-point attributes (range, angle, feature type, etc.)
      types.clear();
      types.resize(plsize);
      plsize--;
      for(uint i=0; i<plsize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
      }
      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      give_feature(pl, types); // compute features from points and their attributes
      // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  else
  {
    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num ++;
        if (valid_num % point_filter_num == 0)
        {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

          if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
              || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
              || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)
              && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
          {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t / 1e6;
      if(pl_orig.points[i].ring < N_SCANS)
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
      if (range < (blind * blind)) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t / 1e6; // curvature unit: ms

      pl_surf.points.push_back(added_pt);
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)//todo check pl_orig.points[plsize - 1].time
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578; // yaw of the first point (index 0), in degrees
      double yaw_end  = yaw_first;
      int layer_first = pl_orig.points[0].ring; // layer index of the first point (index 0)
      for (uint i = plsize - 1; i > 0; i--) // scan backward to find the last point on the same layer as the first point
      {
        if (pl_orig.points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;// yaw of that last same-layer point
          break;
        }
      }
    }

    if(feature_enabled)
    {
      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
      }

      // compute time and convert to PointType, forward order
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        int layer  = pl_orig.points[i].ring;
        if (layer >= N_SCANS) continue;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time / 1000.0; // units: ms

        if (!given_offset_time)
        {
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957; // current point yaw, in degrees
          if (is_first[layer]) // first point of this layer
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle; // record as the start yaw of this layer
              is_first[layer]=false;
              added_pt.curvature = 0.0; // zero the current point curvature (time)
              yaw_last[layer]=yaw_angle; // tentatively record as the end yaw of this layer
              time_last[layer]=added_pt.curvature;
              continue;
          }

          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle; // yaw of the last point on this layer
          time_last[layer]=added_pt.curvature; // time of the last point on this layer
        }

        pl_buff[layer].points.push_back(added_pt);
      }

      for (int j = 0; j < N_SCANS; j++)
      {
        PointCloudXYZI &pl = pl_buff[j]; // points_line
        int linesize = pl.size();
        if (linesize < 2) continue;
        vector<orgtype> &types = typess[j]; // per-point attributes for this scan line
        types.clear();
        types.resize(linesize);
        linesize--;
        for (uint i = 0; i < linesize; i++)
        {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          vx = pl[i].x - pl[i + 1].x;
          vy = pl[i].y - pl[i + 1].y;
          vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz;
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        give_feature(pl, types);
      }
    }
    else
    {
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
        
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time / 1000.0;  // curvature unit: ms

        if (!given_offset_time)
        {
          int layer = pl_orig.points[i].ring;
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          // compute offset time
          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        if (i % point_filter_num == 0)
        {
          if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
          {
            pl_surf.points.push_back(added_pt);
          }
        }
      }
    }
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size(); // raw point count of the input scan line
  int plsize2; // number of points used for feature estimation
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;
  // advance head to the first point beyond the blind range
  while(types[head].range < blind)
  {
    head++;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0; // points on this layer excluding the last group_size

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

    // starting from the first point beyond blind, detect plane features
  for(uint i=head; i<plsize2; i++)
  {
    if(types[i].range < blind) // skip points inside the blind range
    {
      continue;
    }

    i2 = i; // record current point index

    // outputs:
    // i_nex: index of the last point in the local group
    // curr_direct: normalized vector from first to last point of the group (i_cur --> i_nex)
    // return 1 normal exit, 0 break early with curr_direct zeroed
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if(plane_type == 1) // plane_judge normal exit
    {
      for(uint j=i; j<=i_nex; j++)
      {
        if(j!=i && j!=i_nex)
        {
          types[j].ftype = Real_Plane; // interior points of the group are real plane points
        }
        else
        {
          types[j].ftype = Poss_Plane; // boundary points of the group are possible plane points
        }
      }

      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if(last_state==1 && last_direct.norm()>0.1) // classify the start point from the previous state (was a plane) and the vector norm
      {
        double mod = last_direct.transpose() * curr_direct;
        if(mod>-0.707 && mod<0.707) // plane angle 30 degrees
        {
          types[i].ftype = Edge_Plane; // plane-junction edge
        }
        else
        {
          types[i].ftype = Real_Plane;// plane
        }
      }
      
      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
    // from head+3 onward, decide whether non-plane points are edges and their type
  for(uint i=head+3; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i].ftype>=Real_Plane) // too close, or already classified as edge/plane
    {
      continue;
    }

    if(types[i-1].dista<1e-16 || types[i].dista<1e-16) // neighbor spacing too small
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z); // ray from sensor to current point
    Eigen::Vector3d vecs[2]; // vectors from current point to its prev/next neighbors

    for(int j=0; j<2; j++) // compute vectors to prev/next point and the edge property in each direction
    {
      int m = -1;
      if(j == 1)
      {
        m = 1;
      }

      if(types[i+m].range < blind) // prev (m=-1) or next (m=1) neighbor is very close
      {
        if(types[i].range > inf_bound) // classify edge jump by the current point's planar distance
        {
          types[i].edj[j] = Nr_inf; // near far end
        }
        else
        {
          types[i].edj[j] = Nr_blind; // near blind end
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z); // vector from sensor to the prev/next point
      vecs[j] = vecs[j] - vec_a; // vector from current point to the prev/next point

      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm(); // cos angle between ray and that vector
      if(types[i].angle[j] < jump_up_limit) // jump_up_limit defaults to cos(170deg)
      {
        types[i].edj[j] = Nr_180; // angle near 180 degrees
      }
      else if(types[i].angle[j] > jump_down_limit) // jump_down_limit defaults to 8deg
      {
        types[i].edj[j] = Nr_zero;// angle near 0 degrees
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm(); // cos of the angle subtended by the two neighbors
    // classify the edge type from prev/next edge-jump types and the neighbor spacings
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
     
    }
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
      {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize-1;
  double ratio;// ratio of the larger to smaller neighbor spacing
  for(uint i=head+1; i<plsize2; i++)// classify the feature type of unclassified points
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }
    
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    if(types[i].ftype == Nor)
    {
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      // smallp_intersect: cos of 172.5deg by default
      // smallp_ratio: 1.2 by default
      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)// wide angle and similar spacing: treat as a real plane
      {
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;// index of the previous surface feature
  for(uint j=head; j<plsize; j++)
  {
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }
    
      if(j == uint(last_surface+point_filter_num-1))// pick a plane point every point_filter_num indices
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);// record it

        last_surface = -1;
      }
    }
    else
    {
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);// record the edge feature
      }
      if(last_surface != -1)
      {
          // average all points between this edge and the previous surface as a surface feature
          //todo check for overflow
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.intensity /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

// (scan-line cloud, point attributes, current point index, out last index, out direction)
int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA*types[i_cur].range + disB; // local-range threshold for feature computation
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr; // spacing to the next point (between adjacent valid points on the line)
  disarr.reserve(20);

  // iterate over the first group_size points (points needed for a feature, default 8)
  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind) // a point within group_size is inside blind: zero direction and return 2
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista); // save spacing to the next point
  }

  for(;;) // keep iterating, save spacings within group_dis, record the last local index in i_nex
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz; // squared distance from the last point to the current point
    if(two_dis >= group_dis) // break once outside the local-range constraint
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0; // max cross-product magnitude in the group (max parallelogram area)
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++) // iterate from current point to i_nex (max index of the group)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    // vector (i_cur --> j)
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;
    // vx,vy,vz is the vector (i_cur --> i_nex); below is the cross product
    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2]; // cross-product magnitude (parallelogram area)
    if(lw > leng_wid)
    {
      leng_wid = lw;
    }
  }


  if((two_dis*two_dis/leng_wid) < p2l_ratio) // ratio of squared distance to parallelogram area below threshold: return 0
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++) // sort disarr in descending order (leng_wid reused as temp)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid; // swap j and k
      }
    }
  }

  if(disarr[disarr.size()-2] < 1e-16) // second-smallest spacing
  {
    curr_direct.setZero();
    return 0;
  }

  if(lidar_type==AVIA)
  {
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize-2]; // max-to-min spacing ratio
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  
  curr_direct << vx, vy, vz;// vx,vy,vz is the vector (i_cur --> i_nex)
  curr_direct.normalize();
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;
    }
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }
  // d1,d2 are the forward/backward neighbor spacings, direction controlled by nor_dir
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  // make d1 the larger spacing
  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

 // spacing exceeds edgea (2x) or the spacing difference exceeds edgeb (0.1)
  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }
  
  return true;
}

void Preprocess::rs_handler(const sensor_msgs::PointCloud2_<allocator<void>>::ConstPtr &msg) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<rslidar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)//todo check pl_orig.points[plsize - 1].time
    {
        given_offset_time = true;
    }
    else
    {
        given_offset_time = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578; // yaw of the first point (index 0), in degrees
        double yaw_end  = yaw_first;
        int layer_first = pl_orig.points[0].ring; // layer index of the first point (index 0)
        for (uint i = plsize - 1; i > 0; i--) // scan backward to find the last point on the same layer as the first point
        {
            if (pl_orig.points[i].ring == layer_first)
            {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;// yaw of that last same-layer point
                break;
            }
        }
    }

    if(feature_enabled)
    {
        for (int i = 0; i < N_SCANS; i++)
        {
            pl_buff[i].clear();
            pl_buff[i].reserve(plsize);
        }

        // compute time and convert to PointType, forward order
        for (int i = 0; i < plsize; i++)
        {
            PointType added_pt;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            int layer  = pl_orig.points[i].ring;
            if (layer >= N_SCANS) continue;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.curvature = pl_orig.points[i].time / 1000.0; // units: ms

            if (!given_offset_time)
            {
                double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957; // current point yaw, in degrees
                if (is_first[layer]) // first point of this layer
                {
                    // printf("layer: %d; is first: %d", layer, is_first[layer]);
                    yaw_fp[layer]=yaw_angle; // record as the start yaw of this layer
                    is_first[layer]=false;
                    added_pt.curvature = 0.0; // zero the current point curvature (time)
                    yaw_last[layer]=yaw_angle; // tentatively record as the end yaw of this layer
                    time_last[layer]=added_pt.curvature;
                    continue;
                }

                if (yaw_angle <= yaw_fp[layer])
                {
                    added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
                }
                else
                {
                    added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
                }

                if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

                yaw_last[layer] = yaw_angle; // yaw of the last point on this layer
                time_last[layer]=added_pt.curvature; // time of the last point on this layer
            }

            pl_buff[layer].points.push_back(added_pt);
        }

        for (int j = 0; j < N_SCANS; j++)
        {
            PointCloudXYZI &pl = pl_buff[j]; // points_line
            int linesize = pl.size();
            if (linesize < 2) continue;
            vector<orgtype> &types = typess[j]; // per-point attributes for this scan line
            types.clear();
            types.resize(linesize);
            linesize--;
            for (uint i = 0; i < linesize; i++)
            {
                types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
                vx = pl[i].x - pl[i + 1].x;
                vy = pl[i].y - pl[i + 1].y;
                vz = pl[i].z - pl[i + 1].z;
                types[i].dista = vx * vx + vy * vy + vz * vz;
            }
            types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
            give_feature(pl, types);
        }
    }
    else
    {
        for (int i = 0; i < plsize; i++)
        {
            PointType added_pt;
            // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.curvature = pl_orig.points[i].time / 1000.0;  // curvature unit: ms

            if (!given_offset_time)
            {
                int layer = pl_orig.points[i].ring;
                double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

                if (is_first[layer])
                {
                    // printf("layer: %d; is first: %d", layer, is_first[layer]);
                    yaw_fp[layer]=yaw_angle;
                    is_first[layer]=false;
                    added_pt.curvature = 0.0;
                    yaw_last[layer]=yaw_angle;
                    time_last[layer]=added_pt.curvature;
                    continue;
                }

                // compute offset time
                if (yaw_angle <= yaw_fp[layer])
                {
                    added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
                }
                else
                {
                    added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
                }

                if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

                yaw_last[layer] = yaw_angle;
                time_last[layer]=added_pt.curvature;
            }

            if (i % point_filter_num == 0)
            {
                if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
                {
                    pl_surf.points.push_back(added_pt);
                }
            }
        }
    }
}
