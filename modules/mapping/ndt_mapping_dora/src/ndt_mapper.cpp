#include "ndt_mapping/ndt_mapper.hpp"
#include <cmath>

namespace ndt_mapping{
NDTMapper::NDTMapper(const std::string &config_path){
  // Setup all components of the panoptic mapper.
  setConfig(config_path);
  getTF();
}

void NDTMapper::setTF(){
  // set tranform from map to baselink
  // To DO

}

void NDTMapper::getTF(){
  float baselink2lidar_config[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // [x, y, z, roll, pitch, yaw] from baselink to lidar in the config file
  b2l_tf.insert(b2l_tf.begin(), baselink2lidar_config, baselink2lidar_config+6);

  double tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw;
  tf_x = b2l_tf[0];
  tf_y = b2l_tf[1];
  tf_z = b2l_tf[2];
  tf_roll = b2l_tf[3];
  tf_pitch = b2l_tf[4];
  tf_yaw = b2l_tf[5];

  Eigen::Translation3f tl_btol(tf_x, tf_y, tf_z); // tl: translation
  Eigen::AngleAxisf rot_x_btol(tf_roll,
                              Eigen::Vector3f::UnitX()); // rot: rotation
  Eigen::AngleAxisf rot_y_btol(tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(tf_yaw, Eigen::Vector3f::UnitZ());

  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
  tf_ltob = tf_btol.inverse();

}

void NDTMapper::points_callback(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input)
{
  current_scan_time = input->header.stamp;

  // 0. filter by min and max range
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr = filterPoints(input);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());

  // 1. Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(config_.voxel_leaf_size,
                                config_.voxel_leaf_size,
                                config_.voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity()),
      t_base_link(Eigen::Matrix4f::Identity());

  // 2. Using pcl NDT to calculate translation matrix
  t_localizer =
      calTranslation(scan_ptr, transformed_scan_ptr, filtered_scan_ptr);

  // 3. Pub all result message
  Eigen::Matrix3f mat_l, mat_b; //source_code: tf2::Matrix3x3 mat_l, mat_b;
  t_base_link = t_localizer * tf_ltob; // from lidar to base_link fix matrix
  mat_l = setValue(t_localizer);
  mat_b = setValue(t_base_link);

  guess_lidar_pose.setPose(t_localizer, mat_l);
  guess_base_pose.setPose(t_base_link, mat_b);
  current_pose = guess_base_pose; 

  double shift = (current_pose - added_pose).calDistance();
  has_new_scan = false;
  if (shift >= config_.min_add_scan_shift) {
    // TRE;
    added_pose = current_pose;

    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);
    map += *transformed_scan_ptr;

    // 记录本帧增量块（全局坐标系），供节点只发送增量而非整张地图
    last_increment = *transformed_scan_ptr;
    has_new_scan = true;

    if (_incremental_voxel_update == true && config_.save_frame_point == -1)
      ndt_.updateVoxelGrid(transformed_scan_ptr); // update voxel directly
    else
      ndt_.setInputTarget(map.makeShared()); // set for next

    if (config_.save_frame_point != -1) {
      int one_scan_points_num = (*scan_ptr).size();
      int saved_points_num = one_scan_points_num * config_.save_frame_point;

      if (map.size() > saved_points_num) {

        auto oldest = map.begin();
        auto end_num = map.begin() + (map.size() - saved_points_num);
        map.erase(oldest, end_num);
      }
    }
  }
  diff_pose = current_pose - previous_pose;
  previous_pose = current_pose;

  if (_debug_print) {
    std::cout
        << "-----------------------------------------------------------------"
        << std::endl;
    std::cout << "Number of scan points: " << scan_ptr->size() << " points."
              << std::endl;
    std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size()
              << " points." << std::endl;
    if (transformed_scan_ptr->points.size() != 0)
      std::cout << "transformed_scan_ptr: "
                << transformed_scan_ptr->points.size() << " points."
                << std::endl;
    std::cout << "map: " << map.points.size() << " points." << std::endl;
    std::cout << "NDT has converged: " << has_converged << std::endl;
    std::cout << "Fitness score: " << fitness_score << std::endl;
    std::cout << "Number of iteration: " << final_num_iteration << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
    std::cout << "(" << current_pose.x << ", " << current_pose.y << ", "
              << current_pose.z << ", " << current_pose.roll << ", "
              << current_pose.pitch << ", " << current_pose.yaw << ")"
              << std::endl;
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << t_localizer << std::endl;
    std::cout << "shift: " << shift << std::endl;
    std::cout
        << "-----------------------------------------------------------------"
        << std::endl;
  }
}

Eigen::Matrix4f NDTMapper::Pose2Matrix(const Pose &p) {
  Eigen::AngleAxisf rotation_x(p.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rotation_y(p.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rotation_z(p.yaw, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f translation(p.x, p.y, p.z);

  Eigen::Matrix4f poseMatrix =
      (translation * rotation_z * rotation_y * rotation_x).matrix();
  return poseMatrix;
}

void NDTMapper::setConfig(const std::string &config_path) {
  YAML::Node config = YAML::LoadFile(config_path);
  _incremental_voxel_update = config["incremental_voxel_update"].as<bool>();
  _debug_print = config["debug_print"].as<bool>();

  config_.ndt_res = config["resolution"].as<float>();
  config_.step_size = config["step_size"].as<float>();
  config_.trans_eps = config["trans_epsilon"].as<float>();
  config_.max_iter = config["max_iterations"].as<float>();
  config_.voxel_leaf_size = config["leaf_size"].as<float>();
  config_.min_scan_range = config["min_scan_range"].as<float>();
  config_.max_scan_range = config["max_scan_range"].as<float>();
  config_.min_add_scan_shift = config["min_add_scan_shift"].as<float>();
  config_.save_frame_point = config["save_frame_point"].as<int>();

  _save_map_path = config["map_save_path"].as<std::string>();

  std::cout << "======================================> PARAM" << std::endl;
  std::cout << "config file: " << config_path << std::endl;
  std::cout << "resolution: " << config_.ndt_res << std::endl;
  std::cout << "leaf_size: " << config_.voxel_leaf_size << std::endl;
  std::cout << "min_add_scan_shift: " << config_.min_add_scan_shift << std::endl;
  std::cout << "incremental_voxel_update: " << _incremental_voxel_update
            << std::endl;
  std::cout << "save_frame_point: " << config_.save_frame_point << std::endl;
  std::cout << "map_save_path: " << _save_map_path << std::endl;
  std::cout << "PARAM <====================================== " << std::endl;
}

// pcl::PointCloud<pcl::PointXYZI>::Ptr
// NDTMapper::filterPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input) {
//   pcl::PointCloud<pcl::PointXYZI> tmp;
//   pcl::PointCloud<pcl::PointXYZI> scan;
//   pcl::PointXYZI p;
//   double p_radius;
//   tmp = *input; // pcl::fromROSMsg(*input, tmp);

// #pragma omp parallel for num_threads(4)
//   for(pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin();
//   item != tmp.end(); item++) {
//     p.x = (double)item->x;
//     p.y = (double)item->y;
//     p.z = (double)item->z;
//     p.intensity = (double)item->intensity;

//     p_radius = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
//     if (config_.min_scan_range < p_radius && p_radius < config_.max_scan_range)
//       scan.push_back(p);
//   }
//   pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(
//       new pcl::PointCloud<pcl::PointXYZI>(scan));
//   return scan_ptr;
// }

pcl::PointCloud<pcl::PointXYZI>::Ptr
NDTMapper::filterPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input) {
  // 1. 检查输入是否为空
  if (!input || input->empty()) {
    // 如果输入为空，返回一个空的点云指针
    return pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  }

  // 2. 创建一个局部点云对象 scan，用于存储过滤后的结果
  pcl::PointCloud<pcl::PointXYZI> scan;
  // 预分配内存可以显著提高性能，避免在循环中多次重新分配内存
  scan.reserve(input->size());

  // 3. 使用标准的 for 循环遍历输入点云
  // 这里我们使用基于范围的 for 循环 (C++11)，代码更简洁
  for (const auto& item : *input) {
    // 4. 计算点到原点的水平距离
    double p_radius = sqrt(pow(item.x, 2.0) + pow(item.y, 2.0));

    // 5. 根据距离进行过滤
    if (config_.min_scan_range < p_radius && p_radius < config_.max_scan_range) {
      // 如果点在有效范围内，则将其添加到结果点云中
      scan.push_back(item);
    }
  }

  // 6. (可选) 压缩点云的内部存储，释放未使用的预留内存
  // 如果你的环境支持 C++11 且 PCL 版本较新，可以加上这一句
  // scan.shrink_to_fit();

  // 7. 创建一个新的智能指针，管理过滤后的点云数据，并返回
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  return scan_ptr;
}



Eigen::Matrix4f NDTMapper::calTranslation(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr) {
  // ===========================> NDT CALCULATE <===========================
  static int initial_scan_loaded = 0;
  // Add initial point cloud to velodyne_map
  if (initial_scan_loaded == 0) {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
    map += *transformed_scan_ptr;
    ndt_.setTransformationEpsilon(config_.trans_eps);
    ndt_.setStepSize(config_.step_size);
    ndt_.setResolution(config_.ndt_res);
    ndt_.setMaximumIterations(config_.max_iter);
    pcl::PointCloud<pcl::PointXYZI>::Ptr inital_map_ptr(
        new pcl::PointCloud<pcl::PointXYZI>(map));
    ndt_.setInputTarget(inital_map_ptr); // after first it will use the last one
    initial_scan_loaded = 1;
  }

  guess_lidar_pose = previous_pose + diff_pose;
  // TODO odom imu and filter TODO
  Pose guess_pose = guess_lidar_pose;

  Eigen::Matrix4f init_guess = Pose2Matrix(guess_pose) * tf_btol;

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  // IMPORTANT STEP HERE
  ndt_.setInputSource(filtered_scan_ptr);
  ndt_.align(*output_cloud, init_guess);

  fitness_score = ndt_.getFitnessScore();
  Eigen::Matrix4f t_localizer = ndt_.getFinalTransformation();
  has_converged = ndt_.hasConverged();
  final_num_iteration = ndt_.getFinalNumIteration();
  return t_localizer;
}

void NDTMapper::saveMap() {
  std::cout
      << "-----------------------------------------------------------------"
      << std::endl;
  
  double save_map_filter_res = 0.5;
  std::cout << " ==========> ATTENTION! START SAVING MAP" << std::endl;
  std::cout << "filter resolution: " << save_map_filter_res << std::endl;
  std::cout << "save map path:     " << _save_map_path << std::endl;

  // lockPointCloud.lock();
  // Writing Point Cloud data to PCD file
  if (map.empty()) {
    std::cout << " ==========> MAP IS EMPTY, NOTHING TO SAVE" << std::endl;
    return;
  }
  auto map_ptr = map.makeShared();
  if (save_map_filter_res == 0.0) {
    std::cout << "Begin to save map!" <<std::endl;
    pcl::io::savePCDFileBinary(_save_map_path, map);
    std::cout << "Saved " << map.points.size() << " data points to "
              << _save_map_path << "." << std::endl;
  } else {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(save_map_filter_res, save_map_filter_res,
                                  save_map_filter_res);
    voxel_grid_filter.setInputCloud(map_ptr);
    voxel_grid_filter.filter(*map_filtered);
    std::cout << "Begin to save map!" << std::endl;
    pcl::io::savePCDFileBinary(_save_map_path, *map_filtered);
    std::cout << "Original: " << map.points.size() << " points." << std::endl;
    std::cout << "Filtered: " << map_filtered->points.size() << " points."
              << std::endl;
  }
  // lockPointCloud.unlock();
  std::cout << " ===========> SAVE MAP SUCCESS, PLEASE CHECK:" << _save_map_path
            << std::endl;
  std::cout
      << "-----------------------------------------------------------------"
      << std::endl;
}

};
