// Map persistence ROS services: save/load trajectory, keyframes, GNSS, and scenes.
#include "slam_common.h"



/**
 * Get workspace root directory by resolving ROOT_DIR path
 * ROOT_DIR is src/FAST_LIO_SAM/FAST_LIO_SAM/, workspace root is 3 levels up
 */
string getWorkspaceRoot() {
    string root_dir = string(ROOT_DIR);
    // Remove trailing slash if present
    if (!root_dir.empty() && root_dir.back() == '/') {
        root_dir.pop_back();
    }
    // Go up 3 levels: src/FAST_LIO_SAM/FAST_LIO_SAM -> src/FAST_LIO_SAM -> src -> workspace root
    for (int i = 0; i < 3; ++i) {
        size_t last_slash = root_dir.find_last_of('/');
        if (last_slash != string::npos) {
            root_dir = root_dir.substr(0, last_slash);
        } else {
            break;
        }
    }
    return root_dir + "/";
}


/**
 * Resolve destination path: treat all paths starting with / as absolute, others as relative to workspace root
 */
string resolvePath(const string& destination, const string& defaultPath) {
    if (destination.empty()) {
        return getWorkspaceRoot() + defaultPath;
    }
    
    // If path starts with /, treat as absolute path and use directly
    if (destination[0] == '/') {
        return destination;
    }
    
    // Otherwise, treat as relative to workspace root
    return getWorkspaceRoot() + destination;
}


bool CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path, std::ios::out);                          // std::ios::out truncates/overwrites
    if(!ofs)
    {
        std::cout << "open csv file error " << std::endl;
        return  false;
    }
    return true;
}


/* write2txt   format  KITTI*/
void WriteText(std::ofstream& ofs, pose data){
    ofs << std::fixed  <<  data.R(0,0)  << " " << data.R(0,1)   << " "<<   data.R(0,2)  << " "  <<    data.t[0]  <<  " "
                                      <<  data.R(1,0)  << " "  << data.R(1,1)  <<" " <<   data.R(1,2)   << " "  <<   data.t[1]  <<  " "
                                      <<  data.R(2,0)  << " "  << data.R(2,1)  <<" " <<   data.R(2,2)   << " "  <<   data.t[2]  <<  std::endl;

}


bool savePoseService(fast_lio_sam::save_poseRequest& req, fast_lio_sam::save_poseResponse& res)
{
    pose pose_gnss ;
    pose pose_optimized ;
    pose pose_without_optimized ;

    std::ofstream  file_pose_gnss ;
    std::ofstream  file_pose_optimized ;
    std::ofstream  file_pose_without_optimized ;

    cout << "****************************************************" << endl;
    cout << "Saving poses to pose files ..." << endl;
    string savePoseDirectory = resolvePath(req.destination, "output");
    
    // Create directory if it doesn't exist
    string mkdir_cmd = "mkdir -p " + savePoseDirectory;
    int unused = system(mkdir_cmd.c_str());
    (void)unused; // Suppress unused variable warning
    
    cout << "Save destination: " << savePoseDirectory << endl;

    // create file 
    CreateFile(file_pose_gnss, savePoseDirectory + "/gnss_pose.txt");
    CreateFile(file_pose_optimized, savePoseDirectory + "/optimized_pose.txt");
    CreateFile(file_pose_without_optimized, savePoseDirectory + "/without_optimized_pose.txt");

    //  save optimize data
    for(int i = 0; i  < cloudKeyPoses6D->size(); i++){  
        pose_optimized.t =  Eigen::Vector3d(cloudKeyPoses6D->points[i].x, cloudKeyPoses6D->points[i].y, cloudKeyPoses6D->points[i].z  );
        pose_optimized.R = Exp(double(cloudKeyPoses6D->points[i].roll), double(cloudKeyPoses6D->points[i].pitch), double(cloudKeyPoses6D->points[i].yaw) );
        WriteText(file_pose_optimized, pose_optimized);
    }
    cout << "Sucess global optimized  poses to pose files ..." << endl;

    for(int i = 0; i  < fastlio_unoptimized_cloudKeyPoses6D->size(); i++){  
        pose_without_optimized.t =  Eigen::Vector3d(fastlio_unoptimized_cloudKeyPoses6D->points[i].x, fastlio_unoptimized_cloudKeyPoses6D->points[i].y, fastlio_unoptimized_cloudKeyPoses6D->points[i].z  );
        pose_without_optimized.R = Exp(double(fastlio_unoptimized_cloudKeyPoses6D->points[i].roll), double(fastlio_unoptimized_cloudKeyPoses6D->points[i].pitch), double(fastlio_unoptimized_cloudKeyPoses6D->points[i].yaw) );
        WriteText(file_pose_without_optimized, pose_without_optimized);
    }
    cout << "Sucess unoptimized  poses to pose files ..." << endl;

    for(int i = 0; i  < gnss_cloudKeyPoses6D->size(); i++){  
        pose_gnss.t =  Eigen::Vector3d(gnss_cloudKeyPoses6D->points[i].x, gnss_cloudKeyPoses6D->points[i].y, gnss_cloudKeyPoses6D->points[i].z  );
        pose_gnss.R = Exp(double(gnss_cloudKeyPoses6D->points[i].roll), double(gnss_cloudKeyPoses6D->points[i].pitch), double(gnss_cloudKeyPoses6D->points[i].yaw) );
        WriteText(file_pose_gnss, pose_gnss);
    }
    cout << "Sucess gnss  poses to pose files ..." << endl;

    file_pose_gnss.close();
    file_pose_optimized.close();
    file_pose_without_optimized.close();

    // Write per-frame runtime and memory report
    {
        std::lock_guard<std::mutex> lock(mtx_perf);
        size_t n = g_frame_timestamps.size();
        if (n > 0) {
            std::string csvPath = savePoseDirectory + "/per_keyframe_runtime_mem.csv";
            std::string reportPath = savePoseDirectory + "/performance_report.txt";
            FILE* fp_csv = fopen(csvPath.c_str(), "w");
            FILE* fp_report = fopen(reportPath.c_str(), "w");
            if (fp_csv) {
                fprintf(fp_csv, "keyframe_idx,timestamp_sec,total_ms,frontend_ms,backend_ms,mem_mb\n");
                for (size_t i = 0; i < n; i++) {
                    fprintf(fp_csv, "%zu,%.6f,%.3f,%.3f,%.3f,%.2f\n", i, g_frame_timestamps[i], g_frame_runtimes_ms[i], g_frame_frontend_ms[i], g_frame_backend_ms[i], g_frame_mem_mb[i]);
                }
                fclose(fp_csv);
                cout << "Saved per-keyframe runtime/memory to " << csvPath << endl;
            }
            if (fp_report) {
                double sum_rt = 0, sum_fe = 0, sum_be = 0, sum_mem = 0;
                double min_rt = 1e9, max_rt = 0, min_fe = 1e9, max_fe = 0, min_be = 1e9, max_be = 0, min_mem = 1e9, max_mem = 0;
                for (size_t i = 0; i < n; i++) {
                    sum_rt += g_frame_runtimes_ms[i];
                    sum_fe += g_frame_frontend_ms[i];
                    sum_be += g_frame_backend_ms[i];
                    sum_mem += g_frame_mem_mb[i];
                    if (g_frame_runtimes_ms[i] < min_rt) min_rt = g_frame_runtimes_ms[i];
                    if (g_frame_runtimes_ms[i] > max_rt) max_rt = g_frame_runtimes_ms[i];
                    if (g_frame_frontend_ms[i] < min_fe) min_fe = g_frame_frontend_ms[i];
                    if (g_frame_frontend_ms[i] > max_fe) max_fe = g_frame_frontend_ms[i];
                    if (g_frame_backend_ms[i] < min_be) min_be = g_frame_backend_ms[i];
                    if (g_frame_backend_ms[i] > max_be) max_be = g_frame_backend_ms[i];
                    if (g_frame_mem_mb[i] < min_mem) min_mem = g_frame_mem_mb[i];
                    if (g_frame_mem_mb[i] > max_mem) max_mem = g_frame_mem_mb[i];
                }
                fprintf(fp_report, "FAST-LIO-SAM Per-Keyframe Performance Report\n");
                fprintf(fp_report, "============================================\n");
                fprintf(fp_report, "Total keyframes: %zu\n", n);
                fprintf(fp_report, "Total runtime (ms):   avg=%.2f min=%.2f max=%.2f\n", sum_rt / n, min_rt, max_rt);
                fprintf(fp_report, "Frontend runtime (ms): avg=%.2f min=%.2f max=%.2f\n", sum_fe / n, min_fe, max_fe);
                fprintf(fp_report, "Backend runtime (ms):  avg=%.2f min=%.2f max=%.2f\n", sum_be / n, min_be, max_be);
                fprintf(fp_report, "Memory (MB):  avg=%.2f min=%.2f max=%.2f\n", sum_mem / n, min_mem, max_mem);
                fprintf(fp_report, "\nPer-keyframe data: %s\n", csvPath.c_str());
                fclose(fp_report);
                cout << "Saved performance report to " << reportPath << endl;
            }
        }
    }

    return true  ;
}


/**
 * Save the global keyframe feature point set
*/
bool saveMapService(fast_lio_sam::save_mapRequest& req, fast_lio_sam::save_mapResponse& res)
{
      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files ..." << endl;
      
      // Use savePCDDirectory as default if destination is empty and savePCDDirectory is set
      string destination = req.destination;
      if (destination.empty() && !savePCDDirectory.empty()) {
          destination = savePCDDirectory;
      }
      
      string saveMapDirectory = resolvePath(destination, "output");
      
      // Create directory if it doesn't exist
      string mkdir_cmd = "mkdir -p " + saveMapDirectory;
      int unused = system(mkdir_cmd.c_str());
      (void)unused; // Suppress unused variable warning
      
      cout << "Save destination: " << saveMapDirectory << endl;
      // NOTE: the rm/mkdir below was disabled because it wiped the destination dir
    //   int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
    //   unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());
      // save keyframe poses
      pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);                    // keyframe positions
      pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);      // keyframe poses

      // Save individual keyframe files in a subfolder (required for loop closure)
      string keyframesDir = saveMapDirectory + "/keyframes";
      string mkdir_keyframes = "mkdir -p " + keyframesDir;
      int unused_kf = system(mkdir_keyframes.c_str());
      (void)unused_kf; // Suppress unused variable warning
      
      cout << "\nSaving keyframe point clouds for loop closure..." << endl;
      for (int i = 0; i < (int)surfCloudKeyFrames.size(); i++) {
          string keyframeFile = keyframesDir + "/keyframe_" + to_string(i) + ".pcd";
          pcl::io::savePCDFileBinary(keyframeFile, *surfCloudKeyFrames[i]);
          if ((i + 1) % 100 == 0) {
              cout << "\r" << std::flush << "Saved " << (i + 1) << " keyframes ...";
          }
      }
      cout << "\nSaved " << surfCloudKeyFrames.size() << " keyframe point clouds to " << keyframesDir << endl;
      // gather corner/surface point sets from history keyframes
    //   pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
    //   pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());

      // NOTE: keyframes are in lidar frame, but cloudKeyPoses6D stores body-frame poses.
      // Convert to T_world_lidar = T_world_body * T_body_lidar, where T_body_lidar is the extrinsic.
      for (int i = 0; i < (int)cloudKeyPoses6D->size(); i++) {
            //   *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
            *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
            cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
      }

      if(req.resolution != 0)
      {
        cout << "\n\nSave resolution: " << req.resolution << endl;

        // downSizeFilterCorner.setInputCloud(globalCornerCloud);
        // downSizeFilterCorner.setLeafSize(req.resolution, req.resolution, req.resolution);
        // downSizeFilterCorner.filter(*globalCornerCloudDS);
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloudDS);
        // downsample
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(req.resolution, req.resolution, req.resolution);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
      }
      else
      {
        //   downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
         downSizeFilterSurf.setInputCloud(globalSurfCloud);
         downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
         downSizeFilterSurf.filter(*globalSurfCloudDS);
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloud);
        // pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);           // dense point cloud map
      }

      // combine into the global keyframe feature point set
    //   *globalMapCloud += *globalCornerCloud;
      *globalMapCloud += *globalSurfCloud;
      pcl::io::savePCDFileBinary(saveMapDirectory + "/filterGlobalMap.pcd", *globalSurfCloudDS);       // filtered map
      int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);       // dense map
      
      // Save loop closure correspondences
      string loopClosureFile = saveMapDirectory + "/loop_closures.txt";
      std::ofstream loopFile(loopClosureFile);
      if (loopFile.is_open()) {
          mtx.lock();
          int numEdges = 0;
          const double defaultVariance = 1e-2; // fallback when no variance stored (e.g. legacy edges)
          for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it) {
              for (int to : it->second) {
                  std::pair<int, int> canonical(std::min(it->first, to), std::max(it->first, to));
                  double var = defaultVariance;
                  auto vIt = loopClosureVariance.find(canonical);
                  if (vIt != loopClosureVariance.end() && vIt->second > 0.0)
                      var = vIt->second;
                  loopFile << it->first << " " << to << " " << std::scientific << var << endl;
                  numEdges++;
              }
          }
          mtx.unlock();
          loopFile.close();
          cout << "Saved " << numEdges << " loop closure correspondences (direct + chained) with covariance" << endl;
      }
      
      // Save keyframe GNSS positions for GPS-based map merge (WGS84 lat/lon/alt for universal UTM conversion)
      string keyframeGnssFile = saveMapDirectory + "/keyframe_gnss.txt";
      std::ofstream gnssFile(keyframeGnssFile);
      if (gnssFile.is_open()) {
          mtx.lock();
          const double maxTimeDiff = 0.5;
          for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i) {
              if (i < loadedKeyframeCount && i < loadedKeyframeGnssWgs84.size()) {
                  const Eigen::Vector3d& wgs = loadedKeyframeGnssWgs84[i];
                  if (fabs(wgs.x()) < 1e9 && fabs(wgs.y()) < 1e9 && fabs(wgs.z()) < 1e9) {
                      gnssFile << std::fixed << i << " " << wgs.x() << " " << wgs.y() << " " << wgs.z() << endl;  // lat, lon, alt from loaded map
                      continue;
                  }
              }
              double kfTime = cloudKeyPoses6D->points[i].time;
              double bestTimeDiff = 1e9;
              int bestIdx = -1;
              for (size_t j = 0; j < gnss_cloudKeyPoses6D->size(); ++j) {
                  double diff = fabs(gnss_cloudKeyPoses6D->points[j].time - kfTime);
                  if (diff < bestTimeDiff && diff < maxTimeDiff) {
                      bestTimeDiff = diff;
                      bestIdx = j;
                  }
              }
              if (bestIdx >= 0 && bestIdx < (int)gnss_raw_wgs84.size()) {
                  const Eigen::Vector3d& wgs = gnss_raw_wgs84[bestIdx];
                  gnssFile << std::fixed << i << " " << wgs.x() << " " << wgs.y() << " " << wgs.z() << endl;  // lat, lon, alt
              } else {
                  gnssFile << std::fixed << i << " " << GNSS_INVALID_MARKER << " " << GNSS_INVALID_MARKER << " " << GNSS_INVALID_MARKER << endl;
              }
          }
          mtx.unlock();
          gnssFile.close();
          cout << "Saved keyframe GNSS positions (WGS84) for map merge" << endl;
      }
      
      // Save scene index map
      mtx.lock();
      // Add current scene if there are new keyframes and it's not already recorded
      if (cloudKeyPoses6D->size() > loadedKeyframeCount) {
          bool sceneExists = false;
          int currentStart = loadedKeyframeCount;
          int currentEnd = cloudKeyPoses6D->size() - 1;
          string currentSceneName = log_subfolder.empty() ? "default_scene" : log_subfolder;
          
          // Check if current scene is already in the map
          for (const auto& scene : sceneIndexMap) {
              if (scene.start_idx == currentStart && scene.end_idx == currentEnd && scene.scene_name == currentSceneName) {
                  sceneExists = true;
                  break;
              }
          }
          
          if (!sceneExists) {
              SceneRange currentScene;
              currentScene.start_idx = currentStart;
              currentScene.end_idx = currentEnd;
              currentScene.scene_name = currentSceneName;
              sceneIndexMap.push_back(currentScene);
          }
      }
      string sceneIndexFile = saveMapDirectory + "/scene_index_map.txt";
      std::ofstream sceneFile(sceneIndexFile);
      if (sceneFile.is_open()) {
          for (const auto& scene : sceneIndexMap) {
              sceneFile << scene.start_idx << "-" << scene.end_idx << " " << scene.scene_name << endl;
          }
          sceneFile.close();
          cout << "Saved scene index map with " << sceneIndexMap.size() << " scenes" << endl;
      }
      mtx.unlock();
      
      res.success = ret == 0;

      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files completed\n" << endl;

      // visial optimize global map on viz
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time);
    string odometryFrame = "camera_init";
    publishCloud(&pubOptimizedGlobalMap, globalSurfCloudDS, timeLaserInfoStamp, odometryFrame);

      return true;
}


/**
 * Load a saved map and initialize the system
 */
bool loadMapService(fast_lio_sam::load_mapRequest& req, fast_lio_sam::load_mapResponse& res)
{
    cout << "****************************************************" << endl;
    cout << "Loading map from pcd files ..." << endl;
    if(req.source.empty()) {
        res.success = false;
        res.message = "Source directory is empty";
        return false;
    }
    
    string loadMapDirectory = resolvePath(req.source);
    cout << "Load source: " << loadMapDirectory << endl;
    
    // Check if required files exist
    string trajectoryFile = loadMapDirectory + "/trajectory.pcd";
    string transformationsFile = loadMapDirectory + "/transformations.pcd";
    string mapFile = loadMapDirectory + "/filterGlobalMap.pcd";
    
    // Try to load trajectory and transformations
    pcl::PointCloud<PointType>::Ptr loadedTrajectory(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointTypePose>::Ptr loadedTransformations(new pcl::PointCloud<PointTypePose>());
    pcl::PointCloud<PointType>::Ptr loadedMap(new pcl::PointCloud<PointType>());
    
    if (pcl::io::loadPCDFile<PointType>(trajectoryFile, *loadedTrajectory) == -1) {
        res.success = false;
        res.message = "Failed to load trajectory.pcd from " + trajectoryFile;
        cout << res.message << endl;
        return false;
    }
    
    if (pcl::io::loadPCDFile<PointTypePose>(transformationsFile, *loadedTransformations) == -1) {
        res.success = false;
        res.message = "Failed to load transformations.pcd from " + transformationsFile;
        cout << res.message << endl;
        return false;
    }
    
    if (loadedTrajectory->size() != loadedTransformations->size()) {
        res.success = false;
        res.message = "Mismatch between trajectory and transformations sizes";
        cout << res.message << endl;
        return false;
    }
    
    // Load map point cloud (try filterGlobalMap first, then GlobalMap)
    bool mapLoaded = false;
    if (pcl::io::loadPCDFile<PointType>(mapFile, *loadedMap) == -1) {
        string globalMapFile = loadMapDirectory + "/GlobalMap.pcd";
        if (pcl::io::loadPCDFile<PointType>(globalMapFile, *loadedMap) == -1) {
            cout << "Warning: Could not load map point cloud. Continuing without map initialization." << endl;
            mapLoaded = false;
        } else {
            mapLoaded = true;
        }
    } else {
        mapLoaded = true;
    }
    
    cout << "Loaded " << loadedTrajectory->size() << " keyframe poses" << endl;
    if (mapLoaded) {
        cout << "Loaded " << loadedMap->size() << " map points" << endl;
    }
    
    // Load keyframe point clouds for loop closure
    // Try loading from keyframes/ subfolder first, then fall back to root directory for backward compatibility
    string keyframesDir = loadMapDirectory + "/keyframes";
    cout << "Loading keyframe point clouds for loop closure from " << keyframesDir << "..." << endl;
    vector<pcl::PointCloud<PointType>::Ptr> loadedKeyFrames;
    loadedKeyFrames.reserve(loadedTrajectory->size());
    
    int loadedKeyframeCloudCount = 0; // Count of successfully loaded keyframe point cloud files
    for (size_t i = 0; i < loadedTrajectory->size(); ++i) {
        // Try keyframes subfolder first
        string keyframeFile = keyframesDir + "/keyframe_" + to_string(i) + ".pcd";
        pcl::PointCloud<PointType>::Ptr keyframeCloud(new pcl::PointCloud<PointType>());
        
        if (pcl::io::loadPCDFile<PointType>(keyframeFile, *keyframeCloud) == -1) {
            // Fall back to root directory for backward compatibility
            keyframeFile = loadMapDirectory + "/keyframe_" + to_string(i) + ".pcd";
            if (pcl::io::loadPCDFile<PointType>(keyframeFile, *keyframeCloud) == -1) {
                // If keyframe file doesn't exist, create empty cloud (some keyframes might be missing)
                keyframeCloud->clear();
            } else {
                loadedKeyframeCloudCount++;
            }
        } else {
            loadedKeyframeCloudCount++;
        }
        loadedKeyFrames.push_back(keyframeCloud);
        
        if ((i + 1) % 100 == 0) {
            cout << "\r" << std::flush << "Loaded " << (i + 1) << " keyframes ...";
        }
    }
    cout << "\nLoaded " << loadedKeyframeCloudCount << " keyframe point clouds (out of " << loadedTrajectory->size() << " expected)" << endl;
    
    // Load loop closure correspondences (direct + chained; optional third column = variance)
    std::map<int, std::set<int>> loadedLoopClosures;
    std::map<std::pair<int, int>, double> loadedLoopVariance; // canonical -> variance; missing or <=0 means use default
    string loopClosureFile = loadMapDirectory + "/loop_closures.txt";
    std::ifstream loopFile(loopClosureFile);
    if (loopFile.is_open()) {
        string line;
        int numEdges = 0;
        while (std::getline(loopFile, line)) {
            if (line.empty()) continue;
            std::istringstream ss(line);
            int from, to;
            double var = -1.0;
            if (!(ss >> from >> to)) continue;
            if (from < (int)loadedTrajectory->size() && to < (int)loadedTrajectory->size()) {
                loadedLoopClosures[from].insert(to);
                if (ss >> var && var > 0.0) {
                    std::pair<int, int> canonical(std::min(from, to), std::max(from, to));
                    loadedLoopVariance[canonical] = var;
                }
                numEdges++;
            }
        }
        loopFile.close();
        cout << "Loaded " << numEdges << " loop closure correspondences (direct + chained), " << loadedLoopVariance.size() << " with covariance" << endl;
    }
    
    // Load scene index map
    vector<SceneRange> loadedSceneIndexMap;
    string sceneIndexFile = loadMapDirectory + "/scene_index_map.txt";
    std::ifstream sceneFile(sceneIndexFile);
    if (sceneFile.is_open()) {
        string line;
        while (std::getline(sceneFile, line)) {
            if (line.empty()) continue;
            size_t dashPos = line.find('-');
            size_t spacePos = line.find(' ', dashPos);
            if (dashPos != string::npos && spacePos != string::npos) {
                SceneRange scene;
                scene.start_idx = std::stoi(line.substr(0, dashPos));
                scene.end_idx = std::stoi(line.substr(dashPos + 1, spacePos - dashPos - 1));
                scene.scene_name = line.substr(spacePos + 1);
                loadedSceneIndexMap.push_back(scene);
            }
        }
        sceneFile.close();
        cout << "Loaded scene index map with " << loadedSceneIndexMap.size() << " scenes" << endl;
    } else {
        // Backward compatibility: if no scene index map exists, create default entry for loaded keyframes
        if (loadedTrajectory->size() > 0) {
            SceneRange defaultScene;
            defaultScene.start_idx = 0;
            defaultScene.end_idx = loadedTrajectory->size() - 1;
            defaultScene.scene_name = "loaded_map";
            loadedSceneIndexMap.push_back(defaultScene);
            cout << "No scene index map found, created default entry for loaded keyframes (0-" << (loadedTrajectory->size() - 1) << ")" << endl;
        }
    }
    
    // Lock to prevent concurrent access during loading (acquire lock BEFORE GNSS loading
    // to avoid race: loop closure thread could run during GNSS load and leave initialEstimate
    // with keys, causing "key already exists" when we later insert)
    mtx.lock();
    
    // Load keyframe GNSS: WGS84 (lat,lon,alt) -> UTM for universal comparison. Keep WGS84 for re-saving loaded keyframes.
    loadedKeyframeGnssPositions.assign(loadedTrajectory->size(), Eigen::Vector3d(GNSS_INVALID_MARKER, GNSS_INVALID_MARKER, GNSS_INVALID_MARKER));
    loadedKeyframeGnssWgs84.assign(loadedTrajectory->size(), Eigen::Vector3d(GNSS_INVALID_MARKER, GNSS_INVALID_MARKER, GNSS_INVALID_MARKER));
    string keyframeGnssFile = loadMapDirectory + "/keyframe_gnss.txt";
    std::ifstream gnssInFile(keyframeGnssFile);
    if (gnssInFile.is_open()) {
        std::string line;
        int validCount = 0;
        while (std::getline(gnssInFile, line)) {
            std::istringstream iss(line);
            size_t idx;
            double v0, v1, v2;
            std::string v2Str;
            if (!(iss >> idx >> v0 >> v1 >> v2Str)) continue;
            if (idx >= loadedKeyframeGnssPositions.size()) continue;
            // Parse altitude: "nan" or invalid -> 0.0
            v2 = 0.0;
            if (v2Str != "nan" && v2Str != "NaN" && v2Str != "NAN") {
                std::istringstream altStream(v2Str);
                if (altStream >> v2 && fabs(v2) < 1e9) { /* use v2 */ } else { v2 = 0.0; }
            }
            // Require valid lat/lon (WGS84)
            if (fabs(v0) < 1e9 && fabs(v1) < 1e9 && fabs(v0) <= 90 && fabs(v1) <= 180) {
                loadedKeyframeGnssWgs84[idx] = Eigen::Vector3d(v0, v1, v2);
                double easting, northing, altOut;
                Wgs84ToUtm(v0, v1, v2, easting, northing, altOut);
                loadedKeyframeGnssPositions[idx] = Eigen::Vector3d(easting, northing, altOut);
                validCount++;
            }
        }
        gnssInFile.close();
        cout << "Loaded keyframe GNSS: " << validCount << " valid of " << loadedKeyframeGnssPositions.size() << " keyframes" << endl;
        // Build KD-tree for fast radius search (same pattern as pose-based kdtreeHistoryKeyPoses)
        loadedGnssCloud->clear();
        loadedGnssCloud->resize(loadedKeyframeGnssPositions.size());
        for (size_t i = 0; i < loadedKeyframeGnssPositions.size(); ++i) {
            loadedGnssCloud->points[i].x = loadedKeyframeGnssPositions[i].x();
            loadedGnssCloud->points[i].y = loadedKeyframeGnssPositions[i].y();
            loadedGnssCloud->points[i].z = loadedKeyframeGnssPositions[i].z();
            loadedGnssCloud->points[i].intensity = i;  // store keyframe index
        }
        if (loadedGnssCloud->size() >= 2)
            kdtreeLoadedGnss->setInputCloud(loadedGnssCloud);
    } else {
        cout << "No keyframe_gnss.txt found, GPS map merge disabled for this map" << endl;
        loadedGnssCloud->clear();
    }
    
    // IMPORTANT: Loaded map is ONLY for loop closure reference, NOT for initializing current run
    // The current rosbag should start fresh from its own IMU/reference odometry initialization
    // Loaded keyframes are added to ISAM2 so loop closure can work, but current state is NOT set from them
    
    // Store the count of loaded keyframes before appending
    loadedKeyframeCount = loadedTrajectory->size();
    mapMergeSuccessful = false;
    gpsMapMergeRebuildDone = false;
    lastGpsMergeAttemptedKeyframe = -1;
    
    // Store loaded scene index map
    sceneIndexMap = loadedSceneIndexMap;
    
    // Store loaded loop closures for chain tracing (direct + chained); merge variances for later save
    const double defaultLoadedVariance = 1e-2;
    for (auto it = loadedLoopClosures.begin(); it != loadedLoopClosures.end(); ++it) {
        for (int to : it->second) {
            loopIndexContainer[it->first].insert(to);
            std::pair<int, int> canonical(std::min(it->first, to), std::max(it->first, to));
            auto vIt = loadedLoopVariance.find(canonical);
            loopClosureVariance[canonical] = (vIt != loadedLoopVariance.end() && vIt->second > 0.0) ? vIt->second : defaultLoadedVariance;
        }
    }
    
    // Store loaded keyframes for loop closure
    // Append loaded poses to existing (or start fresh if empty)
    for (size_t i = 0; i < loadedTrajectory->size(); ++i) {
        cloudKeyPoses3D->push_back(loadedTrajectory->points[i]);
        cloudKeyPoses6D->push_back(loadedTransformations->points[i]);
    }
    
    // Append loaded keyframe point clouds (already saved at keyframeStorageLeafSize, no re-downsampling)
    for (size_t i = 0; i < loadedKeyFrames.size(); ++i) {
        surfCloudKeyFrames.push_back(loadedKeyFrames[i]);
    }
    
    // Update copy clouds for loop closure thread
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    
    // Add loaded keyframes to ISAM2 so loop closure can work directly
    // But DO NOT connect them to new keyframes - new keyframes will start as a separate disconnected component
    if (!cloudKeyPoses6D->empty() && isam != nullptr) {
        // Get current size of ISAM2 (if any keyframes already exist)
        size_t existingKeyframeCount = isamCurrentEstimate.size();
        
        // If ISAM2 is empty, add loaded keyframes with their poses
        if (existingKeyframeCount == 0) {
            // Clear any stale state (main thread may have added to initialEstimate before we got the lock)
            gtSAMgraph.resize(0);
            initialEstimate.clear();
            
            gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            
            // Add keyframes scene by scene - each scene starts with a prior, only connect within scenes
            // Track max key added to avoid "key already exists" from overlapping scene ranges
            int maxKeyAdded = -1;
            for (size_t sceneIdx = 0; sceneIdx < loadedSceneIndexMap.size(); ++sceneIdx) {
                const SceneRange& scene = loadedSceneIndexMap[sceneIdx];
                int sceneStart = scene.start_idx;
                int sceneEnd = scene.end_idx;
                
                // Add prior factor at the start of each scene (skip if key already added from overlapping scene)
                if (sceneStart > maxKeyAdded) {
                    gtsam::Pose3 firstPoseInScene = pclPointTogtsamPose3(cloudKeyPoses6D->points[sceneStart]);
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(sceneStart, firstPoseInScene, priorNoise));
                    initialEstimate.insert(sceneStart, firstPoseInScene);
                    maxKeyAdded = sceneStart;
                }
                
                // Add between factors only for consecutive keyframes within the same scene
                for (int i = sceneStart + 1; i <= sceneEnd && i < (int)cloudKeyPoses6D->size(); ++i) {
                    gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points[i-1]);
                    gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[i]);
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(i-1, i, poseFrom.between(poseTo), odometryNoise));
                    if (i > maxKeyAdded) {
                        initialEstimate.insert(i, poseTo);
                        maxKeyAdded = i;
                    }
                }
                if (sceneEnd > maxKeyAdded) maxKeyAdded = sceneEnd;
            }
            
            // Add loaded loop closure factors to ISAM2 (clear set since we're rebuilding from loaded data)
            // Use per-edge variance when available (from file), else fixed default
            loopClosurePairsAdded.clear();
            gtsam::noiseModel::Diagonal::shared_ptr defaultLoopNoise = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
            for (auto it = loadedLoopClosures.begin(); it != loadedLoopClosures.end(); ++it) {
                int from = it->first;
                for (int to : it->second) {
                    if (from < (int)cloudKeyPoses6D->size() && to < (int)cloudKeyPoses6D->size()) {
                        std::pair<int, int> canonicalPair(std::min(from, to), std::max(from, to));
                        if (loopClosurePairsAdded.find(canonicalPair) == loopClosurePairsAdded.end()) {
                            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points[from]);
                            gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[to]);
                            gtsam::noiseModel::Diagonal::shared_ptr noise = defaultLoopNoise;
                            auto vIt = loadedLoopVariance.find(canonicalPair);
                            if (vIt != loadedLoopVariance.end() && vIt->second > 0.0) {
                                double v = vIt->second;
                                noise = gtsam::noiseModel::Diagonal::Variances(
                                    (gtsam::Vector(6) << v, v, v, v, v, v).finished());
                            }
                            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(from, to, poseFrom.between(poseTo), noise));
                            loopClosurePairsAdded.insert(canonicalPair);
                        }
                    }
                }
            }
            
            // Update ISAM2 with loaded keyframes and loop closures
            isam->update(gtSAMgraph, initialEstimate);
            isam->update();
            
            // Clear after update
            initialEstimate.clear();
            gtSAMgraph.resize(0);
            
            // Get optimized estimates for loaded keyframes
            isamCurrentEstimate = isam->calculateBestEstimate();
            
            // Update poses with optimized values
            for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i) {
                gtsam::Pose3 optimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(i);
                cloudKeyPoses3D->points[i].x = optimizedPose.translation().x();
                cloudKeyPoses3D->points[i].y = optimizedPose.translation().y();
                cloudKeyPoses3D->points[i].z = optimizedPose.translation().z();
                cloudKeyPoses6D->points[i].x = optimizedPose.translation().x();
                cloudKeyPoses6D->points[i].y = optimizedPose.translation().y();
                cloudKeyPoses6D->points[i].z = optimizedPose.translation().z();
                cloudKeyPoses6D->points[i].roll = optimizedPose.rotation().roll();
                cloudKeyPoses6D->points[i].pitch = optimizedPose.rotation().pitch();
                cloudKeyPoses6D->points[i].yaw = optimizedPose.rotation().yaw();
            }
            
            // Update copy clouds
            *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
            *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        }
    }
    
    // DO NOT set current state from loaded map - let IMU/reference odometry initialize it
    // DO NOT prevent reference initialization - let it work normally
    // The new rosbag will start fresh, and loop closure will align coordinate frames when detected
    
    // Never add preloaded map to ikd-tree. Clear it and let the first scan of the new run initialize it.
    // This avoids slow ICP from millions of preloaded points. Loop closure (loaded keyframes) still works.
    ikdtree.Root_Node = nullptr;
    ikdtree.PCL_Storage.clear();
    Localmap_Initialized = false;
    if (mapLoaded) {
        cout << "Map point cloud (" << loadedMap->size() << " points) loaded for reference only. ikd-tree will be initialized from first scan." << endl;
    } else {
        cout << "ikd-tree will be initialized from new scans." << endl;
    }
    
    mtx.unlock();
    
    // Create publishers for each scene's path
    ros::NodeHandle nh;
    pubLoadedScenePaths.clear();
    loadedScenePaths.clear();
    for (size_t i = 0; i < sceneIndexMap.size(); ++i) {
        string topicName = "fast_lio_sam/loaded_path_scene_" + to_string(i);
        pubLoadedScenePaths.push_back(nh.advertise<nav_msgs::Path>(topicName, 100000));
        loadedScenePaths.push_back(nav_msgs::Path());
    }
    
    // Build and publish initial paths for each scene immediately
    for (size_t sceneIdx = 0; sceneIdx < sceneIndexMap.size() && sceneIdx < pubLoadedScenePaths.size(); ++sceneIdx) {
        const SceneRange& scene = sceneIndexMap[sceneIdx];
        int sceneStart = scene.start_idx;
        int sceneEnd = scene.end_idx;
        
        if (sceneStart >= (int)loadedKeyframeCount) continue;
        if (sceneEnd >= (int)loadedKeyframeCount) sceneEnd = loadedKeyframeCount - 1;
        
        loadedScenePaths[sceneIdx].poses.clear();
        for (int i = sceneStart; i <= sceneEnd && i < (int)cloudKeyPoses6D->size(); ++i) {
            updatePath(cloudKeyPoses6D->points[i], loadedScenePaths[sceneIdx]);
        }
        
        loadedScenePaths[sceneIdx].header.stamp = ros::Time().now();
        loadedScenePaths[sceneIdx].header.frame_id = "camera_init";
        
        // Publish immediately so RViz can see it (publish multiple times to ensure RViz receives it)
        for (int republish = 0; republish < 3; ++republish) {
            pubLoadedScenePaths[sceneIdx].publish(loadedScenePaths[sceneIdx]);
            ros::Duration(0.1).sleep(); // Small delay between publishes
        }
    }
    
    // Clear optimized path - it will be populated with all keyframes (loaded + new) when optimized
    globalPath.poses.clear();
    
    cout << "****************************************************" << endl;
    cout << "Map loading completed successfully" << endl;
    cout << "Loaded " << loadedTrajectory->size() << " keyframes for loop closure reference" << endl;
    if (mapLoaded) {
        cout << "Map point cloud (" << loadedMap->size() << " points) loaded but not used for ikd-tree" << endl;
    }
    cout << "Current rosbag will start fresh from IMU/reference odometry initialization" << endl;
    cout << "Loop closure will align coordinate frames when revisiting areas" << endl;
    cout << "****************************************************" << endl;
    
    res.success = true;
    res.message = "Map loaded successfully. " + to_string(loadedTrajectory->size()) + " keyframes loaded for loop closure reference.";
    if (mapLoaded) {
        res.message += " Map point cloud loaded for reference only (not added to local map).";
    }
    res.message += " " + to_string(loadedKeyframeCloudCount) + " keyframe point clouds loaded. Current run will start fresh.";
    
    return true;
}
