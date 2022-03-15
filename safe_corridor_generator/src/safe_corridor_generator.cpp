#include "safe_corridor_generator/safe_corridor_generator.h"

using namespace std;
using namespace safe_corridor_generator;

/* initialize() method //{ */

void SafeCorridorGenerator::initialize(std::string pcd_file_path, std::string world_frame, double robot_radius, double segment_margin,
                                       std::vector<double> local_bbox, double jps_size_x, double jps_size_y, double jps_size_z, double jps_max_z,
                                       double jps_min_z, std::vector<float> map_frame_coordinates, double inflation_jps, double map_resolution,
                                       double max_sampling_distance, int max_jps_expansions) {

  ROS_INFO("[SafeCorridorGenerator]: Initializing");

  pcd_file_path_         = pcd_file_path;
  world_frame_           = world_frame;
  robot_radius_          = robot_radius;
  segment_margin_        = segment_margin;
  max_sampling_distance_ = max_sampling_distance;
  local_bbox_            = Vec3f(local_bbox.at(0), local_bbox.at(1), local_bbox.at(2));

  // initialize decompose wrapper
  decompose_wrapper_ = std::make_unique<safe_corridor_generator::DecomposeWrapper>();
  decompose_wrapper_->initialize();
  got_pcl_map_ = decompose_wrapper_->loadPCLMap(pcd_file_path_);
  decompose_wrapper_->setRobotRadius(robot_radius_);
  decompose_wrapper_->setSegmentMargin(segment_margin_);
  decompose_wrapper_->setLocalBbox(local_bbox_);

  // initialize JPS planner
  jps_manager_ = std::make_unique<safe_corridor_generator::JPSManager>();

  ROS_INFO_STREAM("Loaded " << decompose_wrapper_->getPclCloud()->width * decompose_wrapper_->getPclCloud()->height << " data points with width "
                            << decompose_wrapper_->getPclCloud()->width);

  jps_manager_->initialize(decompose_wrapper_->getPclCloud(), jps_size_x, jps_size_y, jps_size_z, jps_max_z, jps_min_z, map_frame_coordinates, robot_radius_,
                           inflation_jps, map_resolution, max_jps_expansions);

  n_pcl_initial_points_ = decompose_wrapper_->getPclCloud()->width * decompose_wrapper_->getPclCloud()->height;

  n_pcl_added_points_ = 0;

  setCameraAngles(0.35, 0.25);  // TODO: remove after testing

  initialized_ = true;
  ROS_INFO("[SafeCorridorGenerator]: Initialized");
}

//}

/* getCollisionFreePath() //{ */

vec_Vec3f SafeCorridorGenerator::getCollisionFreePath(const nav_msgs::PathConstPtr &initial_path, bool equal_length_required) {

  vec_Vec3f cf_path;
  vec_Vec3f empty_path;

  if (initial_path->poses.size() < 2) {
    ROS_WARN("[SafeCorridorGenerator]: Initial path consist of less than 2 points. Returning empty path.");
    return cf_path;
  }

  if (isPointOccupied(positionToVec3f(initial_path->poses[0].pose.position))) {
    ROS_WARN("[SafeCorridorGenerator]: Start is occupied, freeing start point");
    Vec3f start_pose(initial_path->poses[0].pose.position.x, initial_path->poses[0].pose.position.y, initial_path->poses[0].pose.position.z);
    jps_manager_->freeStartPoint(start_pose);
  }

  if (isPointOccupied(positionToVec3f(initial_path->poses.back().pose.position))) {
    ROS_WARN("[SafeCorridorGenerator]: Final goal is occupied. TODO: need to search for closest point.");
    /* return cf_path; */
  }

  bool   cf_segment         = true;
  int    n_collision_points = 0;
  double dist               = 0;

  for (size_t k = 0; k < initial_path->poses.size(); k++) {
    if (!isPointOccupied(positionToVec3f(initial_path->poses[k].pose.position))) {
      if (cf_segment) {
        n_collision_points = 0;

        if (k > 0) {  // exclude first point
          dist = sqrt(pow(initial_path->poses[k].pose.position.x - initial_path->poses[k - 1].pose.position.x, 2) +
                      pow(initial_path->poses[k].pose.position.y - initial_path->poses[k - 1].pose.position.y, 2) +
                      pow(initial_path->poses[k].pose.position.z - initial_path->poses[k - 1].pose.position.z, 2));
        }

        if (dist > 2.5) {  // too long segment - need to be checked for collision and segmented

          bool      solved;
          vec_Vec3f path_jps = jps_manager_->solveJPS3D(cf_path.back(), positionToVec3f(initial_path->poses[k].pose.position), solved, false);

          if (solved) {
            bool      skip_first   = true;  // should be true if output of JPS is used (path_jps)
            vec_Vec3f path_sampled = getSampledPath(path_jps, n_collision_points, max_sampling_distance_);
            for (auto &waypoint : path_sampled) {
              if (skip_first) {
                skip_first = false;
                continue;
              }
              cf_path.push_back(waypoint);
            }
          } else {
            ROS_ERROR("[SafeCorridorGenerator]: collision free path not found,  ");
            cf_segment = false;
          }
        } else {
          cf_path.push_back(positionToVec3f(initial_path->poses[k].pose.position));
        }

      } else {
        bool      solved;
        vec_Vec3f path_jps;
        if ( k == initial_path->poses.size() - 1) { // if last point then plan to nearest
          path_jps = jps_manager_->solveJPS3D(cf_path.back(), positionToVec3f(initial_path->poses[k].pose.position), solved, true);
        } else {
          path_jps = jps_manager_->solveJPS3D(cf_path.back(), positionToVec3f(initial_path->poses[k].pose.position), solved, false);
        }

        if (solved) {
          bool      skip_first   = true;  // should be true if output of JPS is used (path_jps)
          vec_Vec3f path_sampled = getSampledPath(path_jps, n_collision_points, max_sampling_distance_);
          for (auto &waypoint : path_sampled) {
            if (skip_first) {
              skip_first = false;
              continue;
            }
            cf_path.push_back(waypoint);
          }
          cf_segment = true;
        } else {
          ROS_WARN("[SafeCorridorGenerator]: path connecting two collision free nodes was not found. Proceeding to last point.");
          n_collision_points = initial_path->poses.size() - k + n_collision_points;
          k = initial_path->poses.size() - 1;
        }
      }
    } else {
      cf_segment = false;
      n_collision_points++;
    }
  }

  if (!cf_segment) {
    ROS_WARN("[SafeCorridorGenerator]: path does not end with a collision free node, returning empty plan - should never happen");
    /* return empty_path; // not required anymore */
  }

  vec_Vec3f sampled_path;

  if (equal_length_required) {
    ROS_WARN("[SafeCorridorGenerator]: Path with equal length required");
    if (cf_path.size() > initial_path->poses.size()) {
      sampled_path = {cf_path.begin(), cf_path.begin() + initial_path->poses.size()};  // subvector creation
    } else if (cf_path.size() < initial_path->poses.size()) {
      ROS_WARN("[SafeCorridorGenerator]: Collision free path is shorter than initial path. Should not happen.");
      sampled_path     = cf_path;
      Vec3f last_point = sampled_path.back();
      for (int k = 0; k < initial_path->poses.size() - cf_path.size(); k++) {
        sampled_path.push_back(last_point);
      }
    } else {
      ROS_WARN("[SafeCorridorGenerator]: Collision free path is not longer than initial path");
      sampled_path = cf_path;
    }
  } else {
    sampled_path = cf_path;
  }

  ROS_WARN_COND(initial_path->poses.size() != sampled_path.size(),
                "[SafeCorridorGenerator]: Sampled path has different size than the initial one (%lu != %lu). Should not happen if sampling is used.",
                sampled_path.size(), initial_path->poses.size());
  ROS_WARN_COND(initial_path->poses.size() == sampled_path.size(), "[SafeCorridorGenerator]: Sampled path has similar size as the initial one (%lu = %lu).",
                sampled_path.size(), initial_path->poses.size());

  return sampled_path;
}

//}

/* getSampledPath() //{ */

vec_Vec3f SafeCorridorGenerator::getSampledPath(vec_Vec3f initial_path, int n_waypoints, double max_sampling_distance) {
  vec_Vec3f sampled_path;

  if (initial_path.size() == n_waypoints) {
    ROS_WARN("[SafeCorridorGenerator]: Sampling: Initial path already has required number of waypoints.");
    sampled_path = initial_path;
    return sampled_path;
  }

  /* for (int k = 0; k < initial_path.size(); k++) { */
  /*   ROS_WARN("[SafeCorridorGenerator - sampling]: Initial path %d: [%.2f, %.2f, %.2f].", k, initial_path[k](0), initial_path[k](1), initial_path[k](2)); */
  /* } */

  double              total_dist, sampling_dist;
  std::vector<double> dists;
  for (int k = 0; k < initial_path.size() - 1; k++) {
    dists.push_back(sqrt(pow(initial_path[k](0) - initial_path[k + 1](0), 2) + pow(initial_path[k](1) - initial_path[k + 1](1), 2) +
                         pow(initial_path[k](2) - initial_path[k + 1](2), 2)));
    total_dist += dists.back();
  }

  sampling_dist        = fmin(total_dist / n_waypoints, max_sampling_distance);
  double traveled_dist = 0.0;
  int    np;
  Vec3f  point;
  for (int k = 0; k < dists.size(); k++) {
    traveled_dist += dists[k];
    if (traveled_dist >= sampling_dist) {
      np        = floor(traveled_dist / sampling_dist);
      Vec3f dir = (initial_path[k + 1] - initial_path[k]) / np;
      for (int n = 0; n < np; n++) {
        point = initial_path[k] + n * dir;
        sampled_path.push_back(point);
      }
      traveled_dist = fmod(traveled_dist, sampling_dist);
    }
  }

  /* for (int k = 0; k < sampled_path.size(); k++) { */
  /*   ROS_WARN("[SafeCorridorGenerator - sampling]: Sampled path %d: [%.2f, %.2f, %.2f].", k, sampled_path[k](0), sampled_path[k](1), sampled_path[k](2)); */
  /* } */

  return sampled_path;
}

//}

/* vec3fPathToNavMsgsPath() //{ */

nav_msgs::PathPtr SafeCorridorGenerator::vec3fPathToNavMsgsPath(vec_Vec3f jps_path) {
  nav_msgs::PathPtr                       nav_path = boost::make_shared<nav_msgs::Path>();
  geometry_msgs::Point                    p;
  geometry_msgs::PoseStamped              ps;
  std::vector<geometry_msgs::PoseStamped> ps_vector;

  for (auto &waypoint : jps_path) {
    p.x              = waypoint(0);
    p.y              = waypoint(1);
    p.z              = waypoint(2);
    ps.pose.position = p;
    ps_vector.push_back(ps);
    // ROS_INFO("[SafeCorridorgenerator]: path waypoint = [%.2f, %.2f, %.2f] ", p.x, p.y, p.z);
  }

  nav_path->poses = ps_vector;
  return nav_path;
}

//}

/* getSafeCorridorPolyhedronVector() //{ */

vec_E<Polyhedron<3>> SafeCorridorGenerator::getSafeCorridorPolyhedronVector(const nav_msgs::PathConstPtr &initial_path) {

  ROS_INFO("[SafeCorridorGenerator]: Get safe corridors polyhedron.");

  if (!initialized_ || !got_pcl_map_) {
    ROS_WARN_COND(initialized_, "[DecomposeNode]: Map not received so far, cannot publish it.");
    ROS_WARN_COND(!initialized_, "[DecomposeNode]: Initialization was not completed, cannot publish a map.");
  }

  vec_E<Polyhedron<3>> pol_vector;

  if (initial_path->poses.size() < 1) {
    ROS_WARN("[SafeCorridorGenerator]: Empty path received. Safety corridors cannot be generated.");
    return pol_vector;
  }

  vec_Vec3f         path_cf  = getCollisionFreePath(initial_path, true);
  nav_msgs::PathPtr path_msg = vec3fPathToNavMsgsPath(path_cf);
  last_path_                 = path_msg;
  got_path_                  = true;
  pol_vector                 = decompose_wrapper_->getCorridorsPolyhedronVector(path_msg);

  last_polyhedrons_ = pol_vector;

  return pol_vector;
}

//}

/* getSafeCorridorPolyhedrons() //{ */

decomp_ros_msgs::PolyhedronArrayPtr SafeCorridorGenerator::getSafeCorridorPolyhedrons(const nav_msgs::PathConstPtr &initial_path) {

  ROS_INFO("[SafeCorridorGenerator]: Get safe corridors polyhedron.");

  if (!initialized_ || !got_pcl_map_) {
    ROS_WARN_COND(initialized_, "[DecomposeNode]: Map not received so far, cannot publish it.");
    ROS_WARN_COND(!initialized_, "[DecomposeNode]: Initialization was not completed, cannot publish a map.");
  }

  if (initial_path->poses.size() < 1) {
    ROS_WARN("[SafeCorridorGenerator]: Empty path received. Safety corridors cannot be generated.");
    return boost::make_shared<decomp_ros_msgs::PolyhedronArray>();
  }

  geometry_msgs::Point pose;
  pose.x = 0.0;
  pose.y = 5.0;
  pose.z = 4.0;
  addPositionOfRobotToPclMap(pose, 2.0, 100);
  pose.z = 9.0;
  addPyramidToPclMap(pose, 0.0, 0.0, 8.0, 0.5);
  updateMaps();

  vec_Vec3f         path_cf                               = getCollisionFreePath(initial_path, true);
  nav_msgs::PathPtr path_msg                              = vec3fPathToNavMsgsPath(path_cf);
  last_path_                                              = path_msg;
  got_path_                                               = true;
  decomp_ros_msgs::PolyhedronArray polyhedron_msg         = decompose_wrapper_->getCorridorsPolyhedron(path_msg);
  polyhedron_msg.header.frame_id                          = world_frame_;
  decomp_ros_msgs::PolyhedronArray::Ptr polyhedron_msg_sp = boost::make_shared<decomp_ros_msgs::PolyhedronArray>(polyhedron_msg);

  /* removeAddedPointsFromPclMap(); */

  return polyhedron_msg_sp;
}

//}

/* getSafeCorridorEllipsoids() //{ */

decomp_ros_msgs::EllipsoidArrayPtr SafeCorridorGenerator::getSafeCorridorEllipsoids(const nav_msgs::PathConstPtr &initial_path) {

  ROS_INFO("[SafeCorridorGenerator]: Get safe corridors ellipsoids.");

  if (!initialized_ || !got_pcl_map_) {
    ROS_WARN_COND(initialized_, "[DecomposeNode]: Map not received so far, cannot publish it.");
    ROS_WARN_COND(!initialized_, "[DecomposeNode]: Initialization was not completed, cannot publish a map.");
  }

  if (initial_path->poses.size() < 1) {
    ROS_WARN("[SafeCorridorGenerator]: Empty path received. Safety corridors cannot be generated.");
    return boost::make_shared<decomp_ros_msgs::EllipsoidArray>();
  }

  vec_Vec3f         path_cf                             = getCollisionFreePath(initial_path, true);
  nav_msgs::PathPtr path_msg                            = vec3fPathToNavMsgsPath(path_cf);
  last_path_                                            = path_msg;
  got_path_                                             = true;
  decomp_ros_msgs::EllipsoidArray ellipsoid_msg         = decompose_wrapper_->getCorridorsEllipsoid(path_msg);
  ellipsoid_msg.header.frame_id                         = world_frame_;
  decomp_ros_msgs::EllipsoidArray::Ptr ellipsoid_msg_sp = boost::make_shared<decomp_ros_msgs::EllipsoidArray>(ellipsoid_msg);

  return ellipsoid_msg_sp;
}

//}

/* getLastPath() //{ */

nav_msgs::PathPtr SafeCorridorGenerator::getLastPath() {

  if (!got_path_) {
    ROS_WARN("[SafeCorridorGenerator]: Path not got yet, returning empty path.");
  }

  return last_path_;
}

//}

/* FUNCTIONS FOR PUBLISHING //{ */

/* publishCorridor() //{ */

void SafeCorridorGenerator::publishCorridor(decomp_ros_msgs::EllipsoidArrayPtr ell_array, ros::Publisher pub_ell) {
  try {
    pub_ell.publish(ell_array);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_ell.getTopic().c_str());
  }
};

void SafeCorridorGenerator::publishCorridor(decomp_ros_msgs::PolyhedronArrayPtr ell_array, ros::Publisher pub_ell) {
  try {
    pub_ell.publish(ell_array);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_ell.getTopic().c_str());
  }
};

void SafeCorridorGenerator::publishCorridor(const vec_E<Polyhedron<3>> &pol_array, ros::Publisher &pub_pol) {
  decomp_ros_msgs::PolyhedronArray pol_array_ros = DecompROS::polyhedron_array_to_ros(pol_array);
  pol_array_ros.header.frame_id                  = world_frame_;
  try {
    pub_pol.publish(pol_array_ros);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_pol.getTopic().c_str());
  }
}

//}

/* publishCorridor() //{ */

void SafeCorridorGenerator::publishCorridor(ros::Publisher &pub_pol) {
  decomp_ros_msgs::PolyhedronArray pol_array_ros = DecompROS::polyhedron_array_to_ros(last_polyhedrons_);
  pol_array_ros.header.frame_id                  = world_frame_;
  try {
    pub_pol.publish(pol_array_ros);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_pol.getTopic().c_str());
  }
}

//}

/* publishPath() //{ */

void SafeCorridorGenerator::publishLastPath(ros::Publisher pub_path) {

  if (!initialized_ || !got_path_) {
    ROS_WARN("[SafeCorridorGenerator]: Path not received so far, cannot publish it.");
  }

  last_path_->header.frame_id = world_frame_;

  try {
    pub_path.publish(last_path_);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_path.getTopic().c_str());
  }
}

/* //} */

/* publishCloud() //{ */

void SafeCorridorGenerator::publishCloud(ros::Publisher pub_cloud) {

  if (!initialized_ || !got_pcl_map_) {
    ROS_WARN_COND(initialized_, "[SafeCorridorGenerator]: Map not received so far, cannot publish it.");
    ROS_WARN_COND(!initialized_, "[SafeCorridorGenerator]: Initialization was not completed, cannot publish a map.");
  }

  sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>(decompose_wrapper_->getPCLMap());
  msg->header.frame_id              = world_frame_;

  try {
    ROS_INFO("[SafeCorridorGenerator]: Publishing cloud map of size %lu", msg->data.size());
    pub_cloud.publish(msg);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_cloud.getTopic().c_str());
  }
}

//}

//}

/* MAP UPDATES //{ */

/* addPositionOfRobotsToPclMap() //{ */

void SafeCorridorGenerator::addPositionsOfRobotsToPclMap(geometry_msgs::PoseArray poses, double radius, int n_points) {

  ROS_INFO("[SafeCorridorGenerator]: Including positions of %lu robots into PCL map.", poses.poses.size());

  for (auto &p : poses.poses) {
    addPositionOfRobotToPclMap(p.position, radius, n_points);
  }
}

//}

/* addPositionOfRobotToPclMap() //{ */

void SafeCorridorGenerator::addPositionOfRobotToPclMap(geometry_msgs::Point pose, double radius, int n_points) {

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_ptr    = decompose_wrapper_->getPclCloud();
  std::vector<pcl::PointXYZ>                        pcl_sphere = generatePclSphere(n_points, radius, pose);
  n_pcl_added_points_                                          = n_pcl_added_points_ + pcl_sphere.size();
  for (auto &p : pcl_sphere) {
    pcl_ptr->push_back(p);
  }
}

//}

/* addPyramidToPclMap() //{ */

void SafeCorridorGenerator::addPyramidToPclMap(geometry_msgs::Point pose, double heading, double pitch, double fov_length, double sampling_dist) {

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_ptr     = decompose_wrapper_->getPclCloud();
  std::vector<pcl::PointXYZ>                        pcl_pyramid = generatePclPyramid(pose, heading, pitch, fov_length, sampling_dist);
  ROS_INFO("[SafeCorridorGenerator]: Generated pcl pyramid has %lu points", pcl_pyramid.size());
  n_pcl_added_points_ = n_pcl_added_points_ + pcl_pyramid.size();
  for (auto &p : pcl_pyramid) {
    pcl_ptr->push_back(p);
  }
}

//}

/* removeAddedPointsFromPclMap() //{ */

void SafeCorridorGenerator::removeAddedPointsFromPclMap() {

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_ptr = decompose_wrapper_->getPclCloud();

  if (pcl_ptr->size() > n_pcl_initial_points_) {
    ROS_INFO("[SafeCorridorGenerator]: removing points ");
    pcl_ptr->erase(pcl_ptr->begin() + n_pcl_initial_points_, pcl_ptr->end());
    n_pcl_added_points_ = 0;
    decompose_wrapper_->updateMap();
    jps_manager_->updateMap();
  } else {
    ROS_ERROR(
        "[SafeCorridorGenerator]: Cannot reset the pcl map to original state. Number of points in current map is not higher than the initial size of the "
        "map.");
  }
}

//}

/* updateMaps() //{ */

void SafeCorridorGenerator::updateMaps() {
  decompose_wrapper_->updateMap();
  jps_manager_->updateMap();
  ROS_INFO("[SafeCorridorGenerator]: Maps updated. New size of the pointcloud = %d",
           decompose_wrapper_->getPclCloud()->height * decompose_wrapper_->getPclCloud()->width);
}


void SafeCorridorGenerator::updateMaps(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &_pcd_input, const std::vector<float> &_pose) {
  decompose_wrapper_->updateMap(_pcd_input);
  jps_manager_->updateMap(_pcd_input);
  ROS_INFO("[SafeCorridorGenerator]: Maps updated. New size of the pointcloud = %d",
           decompose_wrapper_->getPclCloud()->height * decompose_wrapper_->getPclCloud()->width);
}

//}

/* generatePclSphere() //{ */

std::vector<pcl::PointXYZ> SafeCorridorGenerator::generatePclSphere(int n_points, double radius, geometry_msgs::Point center) {
  double                     phi, theta, x, y, z;
  std::vector<pcl::PointXYZ> sunflower_sphere;
  for (int k = 0; k < n_points; k++) {
    phi   = acos(1 - 2 * (k + 0.5) / n_points);
    theta = M_PI * (1 + pow(5, 0.5)) * (k + 0.5);
    x     = center.x + radius * cos(theta) * sin(phi);
    y     = center.y + radius * sin(theta) * sin(phi);
    z     = center.z + radius * cos(phi);
    sunflower_sphere.push_back(pcl::PointXYZ(x, y, z));
  }
  ROS_INFO("[SafeCorridorGenerator]: Sunflower sphere size = %lu", sunflower_sphere.size());
  return sunflower_sphere;
}

//}

/* generatePclPyramid() //{ */

std::vector<pcl::PointXYZ> SafeCorridorGenerator::generatePclPyramid(geometry_msgs::Point center, double heading, double pitch, double fov_length,
                                                                     double sampling_dist) {
  std::vector<pcl::PointXYZ> pyramid;

  // find four vectors
  geometry_msgs::Point              p_ul, p_ur, p_bl, p_br, tmp_dir;
  std::vector<geometry_msgs::Point> pv;
  double                            base = fov_length / cos(camera_aov_h_);

  p_ul.x = base * cos(heading + camera_aov_h_) * cos(pitch + camera_aov_v_);
  p_ul.y = base * sin(heading + camera_aov_h_) * cos(pitch + camera_aov_v_);
  p_ul.z = base * sin(pitch + camera_aov_v_);

  p_ur.x = base * cos(heading - camera_aov_h_) * cos(pitch + camera_aov_v_);
  p_ur.y = base * sin(heading - camera_aov_h_) * cos(pitch + camera_aov_v_);
  p_ur.z = base * sin(pitch + camera_aov_v_);

  p_bl.x = base * cos(heading + camera_aov_h_) * cos(pitch - camera_aov_v_);
  p_bl.y = base * sin(heading + camera_aov_h_) * cos(pitch - camera_aov_v_);
  p_bl.z = base * sin(pitch - camera_aov_v_);

  p_br.x = base * cos(heading - camera_aov_h_) * cos(pitch - camera_aov_v_);
  p_br.y = base * sin(heading - camera_aov_h_) * cos(pitch - camera_aov_v_);
  p_br.z = base * sin(pitch - camera_aov_v_);

  pv.push_back(p_ul);
  pv.push_back(p_ur);
  pv.push_back(p_br);
  pv.push_back(p_bl);

  int    n_points = ceil(base / sampling_dist);
  int    n2;
  double x, y, z;
  double p_norm;

  for (int k = 0; k < n_points; k++) {
    for (int p = 0; p < pv.size(); p++) {
      p_norm = k / (double)n_points;
      x      = center.x + p_norm * pv[p].x;
      y      = center.y + p_norm * pv[p].y;
      z      = center.z + p_norm * pv[p].z;
      pyramid.push_back(pcl::PointXYZ(x, y, z));
    }
  }

  int    pv_first_size = pyramid.size() - 1;
  double norm_size;
  for (int k = 0; k < pv_first_size; k++) {
    if ((k + 1) % 5 == 0) {
      continue;
    }

    tmp_dir.x = pyramid[k + 1].x - pyramid[k].x;
    tmp_dir.y = pyramid[k + 1].y - pyramid[k].y;
    tmp_dir.z = pyramid[k + 1].z - pyramid[k].z;

    n2 = round(sqrt(pow(tmp_dir.x, 2) + pow(tmp_dir.y, 2) + pow(tmp_dir.z, 2)));

    for (int s = 1; s < n2; s++) {
      norm_size = s / (double)n2;
      x         = pyramid[k].x + tmp_dir.x * norm_size;
      y         = pyramid[k].y + tmp_dir.y * norm_size;
      z         = pyramid[k].z + tmp_dir.z * norm_size;
      pyramid.push_back(pcl::PointXYZ(x, y, z));
    }
  }

  ROS_INFO("[SafeCorridorGenerator]: Pyramid size = %lu", pyramid.size());
  return pyramid;
}

//}

//}

/* HELPER FUNCTIONS //{ */

/* positionToVec3f() //{ */

Vec3f SafeCorridorGenerator::positionToVec3f(geometry_msgs::Point point) {
  return Vec3f(point.x, point.y, point.z);
}

//}

/* isPointOccupied() //{ */
bool SafeCorridorGenerator::isPointOccupied(Vec3f point) {
  return jps_manager_->isPointOccupied(point);
}
//}

/* setCameraAngles() //{ */

void SafeCorridorGenerator::setCameraAngles(double aov_h, double aov_v) {
  camera_aov_h_ = aov_h;
  camera_aov_v_ = aov_v;
  ROS_INFO("[SafeCorridorGenerator]: Set camera angles of view: horizontal = %.2f, vertical = %.2f", camera_aov_h_, camera_aov_v_);
};

//}

//}

