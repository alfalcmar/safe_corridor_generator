#include "safe_corridor_generator/decompose_wrapper.h"

using namespace std;
using namespace safe_corridor_generator;

namespace safe_corridor_generator
{

void DecomposeWrapper::initialize() {
  pcl_map_changed_ = true;
  got_pcl_map_     = false;
  got_corridors_   = false;
  initialized_     = true;
  ROS_INFO("[DecomposeWrapper]: Initialized.");
}

/* loadPCLMap() //{ */

bool DecomposeWrapper::loadPCLMap(const string &filepath) {

  pcl_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath.c_str(), *pcl_cloud_) == -1)  // load the file
  {
    ROS_ERROR("Couldn't read file %s\n", filepath.c_str());
    return false;
  }
  ROS_INFO_STREAM("Loaded " << pcl_cloud_->width * pcl_cloud_->height << " data points from " << filepath);

  bool success = updateMap();

  return success;
}

//}

/* updateMap() //{ */

bool DecomposeWrapper::updateMap() {
  sensor_msgs::PointCloud cloud_msg;
  pcl::toROSMsg(*pcl_cloud_.get(), pcl_sensor_message_);
  if (sensor_msgs::convertPointCloud2ToPointCloud(pcl_sensor_message_, cloud_msg)) {
    pcl_map_vector_ = DecompROS::cloud_to_vec(cloud_msg);
    got_pcl_map_     = true;
    pcl_map_changed_ = true;
    return true;
  } else {
    ROS_WARN("[DecomposeWrapper]: Conversion of PointCloud to PointCloud2 failed.");
    return false;
  }
}

bool DecomposeWrapper::updateMap(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &_pcd_input) {
  // UPDATE PCL CLOUD
  pcl_cloud_ = _pcd_input;

  sensor_msgs::PointCloud cloud_msg;
  pcl::toROSMsg(*_pcd_input.get(), pcl_sensor_message_);
  if (sensor_msgs::convertPointCloud2ToPointCloud(pcl_sensor_message_, cloud_msg)) {
    pcl_map_vector_ = DecompROS::cloud_to_vec(cloud_msg);
    got_pcl_map_     = true;
    pcl_map_changed_ = true;
    return true;
  } else {
    ROS_WARN("[DecomposeWrapper]: Conversion of PointCloud to PointCloud2 failed.");
    return false;
  }
}

//}


/* generateCorridors() //{ */

bool DecomposeWrapper::generateCorridors(vec_Vec3f reference_path) {

  if (!initialized_ || !got_pcl_map_) {
    ROS_WARN("[DecomposeWrapper]: Not ready for corridor generation. (Initialized = %d, got_pcl_map = %d)", initialized_, got_pcl_map_);
    return false;
  }

  ros::WallTime start;
  if (pcl_map_changed_) {
    start = ros::WallTime::now();
    decomp_util_.set_obs(pcl_map_vector_);
    decomp_util_.set_local_bbox(local_bbox_);  // use for generation of cuboids surrounding the path
    decomp_util_.set_inflate_distance(robot_radius_);
    ROS_INFO("[DecomposeWrapper]: Setting obstacles took %.2f ms", (ros::WallTime::now() - start).toSec() * 1000.0);
  }

  start = ros::WallTime::now();
  if (reference_path.size() != 0) {
    decomp_util_.dilate(reference_path, segment_margin_);
    ROS_INFO("[DecomposeWrapper]: Corridors generated. Dilating path took %.2f ms", (ros::WallTime::now() - start).toSec() * 1000.0);

    got_corridors_ = true;
  } else {
    ROS_WARN("[DecomposeWrapper]: Cannot dilate path with zero length");
    return false;
  }

  return true;
}

//}

decomp_ros_msgs::EllipsoidArray DecomposeWrapper::getCorridorsEllipsoid(const nav_msgs::PathConstPtr &reference_path) {

  decomp_ros_msgs::EllipsoidArray ell_array;
  if (!generateCorridors(pathToVector(reference_path))) {
    return ell_array;
  }

  return DecompROS::ellipsoid_array_to_ros(decomp_util_.get_ellipsoids());
}

decomp_ros_msgs::PolyhedronArray DecomposeWrapper::getCorridorsPolyhedron(const nav_msgs::PathConstPtr &reference_path) {

  decomp_ros_msgs::PolyhedronArray pol_array;
  if (!generateCorridors(pathToVector(reference_path))) {
    return pol_array;
  }

  return DecompROS::polyhedron_array_to_ros(decomp_util_.get_polyhedrons());
}

vec_E<Polyhedron<3>> DecomposeWrapper::getCorridorsPolyhedronVector(const nav_msgs::PathConstPtr &reference_path) {

  vec_E<Polyhedron<3>> pol_array;
  if (!generateCorridors(pathToVector(reference_path))) {
    return pol_array;
  }

  return decomp_util_.get_polyhedrons();
}

/* pathToVector() //{ */

vec_Vec3f DecomposeWrapper::pathToVector(const nav_msgs::PathConstPtr &path_ref) {
  vec_Vec3f path_vector;
  Vec3f     waypoint;
  for (const auto &it : path_ref->poses) {
    waypoint(0) = it.pose.position.x;
    waypoint(1) = it.pose.position.y;
    waypoint(2) = it.pose.position.z;
    path_vector.push_back(waypoint);
  }
  path_vector_ = path_vector;
  return path_vector;
}

//}

void DecomposeWrapper::printInequalityConstraints() {

  if (!got_corridors_) {
    return;
  }

  // Convert to inequality constraints Ax < b
  // Taken from decomp test node
  auto polys = decomp_util_.get_polyhedrons();
  for (size_t i = 0; i < path_vector_.size() - 1; i++) {
    const auto         pt_inside = (path_vector_[i] + path_vector_[i + 1]) / 2;
    LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
    printf("i: %zu\n", i);
    std::cout << "A: " << cs.A() << std::endl;
    std::cout << "b: " << cs.b() << std::endl;
    std::cout << "point: " << path_vector_[i].transpose();
    if (cs.inside(path_vector_[i]))
      std::cout << " is inside!" << std::endl;
    else
      std::cout << " is outside!" << std::endl;

    std::cout << "point: " << path_vector_[i + 1].transpose();
    if (cs.inside(path_vector_[i + 1]))
      std::cout << " is inside!" << std::endl;
    else
      std::cout << " is outside!" << std::endl;
  }
}

/* getGotCorridors() //{ */

bool DecomposeWrapper::getGotCorridors() {
  return got_corridors_;
}

//}

/* getPCLMap() //{ */

sensor_msgs::PointCloud2 DecomposeWrapper::getPCLMap() {

  if (!initialized_ || !got_pcl_map_) {
    ROS_WARN("[DecomposeWrapper]: Pointcloud is not available. Returning empty point cloud");
  }

  pcl::toROSMsg(*pcl_cloud_.get(), pcl_sensor_message_);

  return pcl_sensor_message_;
}

//}

/* setRobotRadius() //{ */

void DecomposeWrapper::setRobotRadius(double robot_radius) {
  robot_radius_ = robot_radius;
  decomp_util_.set_inflate_distance(robot_radius_);
  ROS_INFO("[DecomposeWrapper]: Robot radius set to %.2f", robot_radius_);
}

//}

/* setSegmentMargin() //{ */

void DecomposeWrapper::setSegmentMargin(double segment_margin) {
  segment_margin_ = segment_margin;
  ROS_INFO("[DecomposeWrapper]: Segment margin set to %.2f", segment_margin_);
}

//}

/* setLocalBbox() //{ */

void DecomposeWrapper::setLocalBbox(Vec3f local_bbox) {
  local_bbox_ = local_bbox;
  ROS_INFO("[DecomposeWrapper]: Local bbox set to [%.2f, %.2f, %.2f]", local_bbox_(0), local_bbox_(1), local_bbox_(2));
}

//}

/* getPclCloud() //{ */

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> DecomposeWrapper::getPclCloud() {
  ROS_INFO("[DecomposeWrapper]: size of cloud = %d ", pcl_cloud_->height * pcl_cloud_->width);
  return pcl_cloud_;
}

//}

}  // namespace safe_corridor_generator
