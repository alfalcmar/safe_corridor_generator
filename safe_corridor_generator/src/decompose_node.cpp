#include "safe_corridor_generator/decompose_node.h"

using namespace std;
using namespace safe_corridor_generator;

/* onInit() method //{ */

void DecomposeNode::onInit() {

  ROS_INFO("[DecomposeNode]: Initializing ------------------------------");
  nh_ = nodelet::Nodelet::getPrivateNodeHandle();
  ros::Time::waitForValid();

  /** Load parameters from ROS * //{*/
  string               node_name = ros::this_node::getName().c_str();
  // LOAD INITIAL PARAMETERS
  ROS_INFO("[DecomposeWrapper]: Loading static parameters:");
  std::vector<double> _local_bbox;
  std::vector<float>  _map_frame_coordinates;
  double              _size_x, _size_y, _size_z, _min_z, _max_z, _jps_inflation, _map_resolution, _max_sampling_dist, _max_jps_expansions;
  loadParam(nh_,"pcl_filepath", pcd_file_path_);
  loadParam(nh_,"world_frame", world_frame_);
  loadParam(nh_,"loop_rate", main_loop_rate_);
  loadParam(nh_,"decompose_inflation", robot_radius_);
  loadParam(nh_,"segment_margin", segment_margin_);
  loadParam(nh_,"local_bbox", _local_bbox);
  loadParam(nh_,"test", test_);
  loadParam(nh_,"map_width_x", _size_x);
  loadParam(nh_,"map_width_y", _size_y);
  loadParam(nh_,"map_width_z", _size_z);
  loadParam(nh_,"max_height", _max_z);
  loadParam(nh_,"z_ground", _min_z);
  loadParam(nh_,"inflation", _jps_inflation);
  loadParam(nh_,"resolution", _map_resolution);
  loadParam(nh_,"map_center", _map_frame_coordinates);
  loadParam(nh_,"max_sampling_dist", _max_sampling_dist);
  loadParam(nh_,"max_jps_expansions", _max_jps_expansions);

  //}

  /** Create publishers and subscribers //{**/
  // Initialize other subs and pubs

  pub_path_                 = nh_.advertise<nav_msgs::Path>("path_out", 1);
  pub_point_cloud_          = nh_.advertise<sensor_msgs::PointCloud2>("pcl_map_out", 1);
  pub_corridor_polyhedrons_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedrons_out", 1);
  pub_corridor_ellipsoids_  = nh_.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoids_out", 1);

  sub_path_ = nh_.subscribe("path_in", 1, &DecomposeNode::referencePathCallback, this, ros::TransportHints().tcpNoDelay());

  // initialize decompose wrapper
  safe_corridor_generator_.initialize(pcd_file_path_, world_frame_, robot_radius_, segment_margin_, _local_bbox, _size_x, _size_y, _size_z, _max_z, _min_z,
                                      _map_frame_coordinates, _jps_inflation, _map_resolution, _max_sampling_dist, _max_jps_expansions);

  main_timer_ = nh_.createTimer(ros::Rate(main_loop_rate_), &DecomposeNode::mainTimer, this);

  initialized_ = true;
  ROS_INFO("[DecomposeNode]: Initialized ------------------------------");
}


//}

/* mainTimer() //{ */
void DecomposeNode::mainTimer([[maybe_unused]] const ros::TimerEvent &event) {
  if (ros::ok()) {
    ROS_INFO("[DecomposeNode]: Main loop spinning");
    safe_corridor_generator_.publishCloud(pub_point_cloud_);
  }
}
//}

/* referencePathCallback() //{ */

void DecomposeNode::referencePathCallback(const nav_msgs::PathConstPtr &msg) {

  ROS_INFO("[DecomposeNode]: Path reference received.");

  ROS_INFO("[DecomposeNode]: Publishing corridors for path with #waypoints = %lu", msg->poses.size());

  for (auto &pose : msg->poses) {
    ROS_INFO("[debug]: Obtained path: [%.2f, %.2f, %.2f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  }

  geometry_msgs::Pose p;
  geometry_msgs::PoseArray pa;
  p.position.x = 10.0;
  p.position.y = 10.0;
  p.position.z = 5.0;
  pa.poses.push_back(p);
  p.position.x = -10.0;
  p.position.y = 10.0;
  p.position.z = 5.0;
  pa.poses.push_back(p);
  p.position.x = -10.0;
  p.position.y = -10.0;
  p.position.z = 5.0;
  pa.poses.push_back(p);
  
  safe_corridor_generator_.addPositionsOfRobotsToPclMap(pa, 3.0, 100);
  safe_corridor_generator_.updateMaps();

  decomp_ros_msgs::PolyhedronArrayPtr pol_corrs = safe_corridor_generator_.getSafeCorridorPolyhedrons(msg);
  decomp_ros_msgs::EllipsoidArrayPtr  ell_corrs = safe_corridor_generator_.getSafeCorridorEllipsoids(msg);

  vec_E<Polyhedron<3>> pol_vec = safe_corridor_generator_.getSafeCorridorPolyhedronVector(msg);

  safe_corridor_generator_.publishLastPath(pub_path_);

  ROS_INFO("[DecomposeNode]: Publishing corridors ");
  safe_corridor_generator_.publishCorridor(pol_corrs, pub_corridor_polyhedrons_);
  safe_corridor_generator_.publishCorridor(ell_corrs, pub_corridor_ellipsoids_);

  safe_corridor_generator_.removeAddedPointsFromPclMap();
}


//}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(safe_corridor_generator::DecomposeNode, nodelet::Nodelet)
