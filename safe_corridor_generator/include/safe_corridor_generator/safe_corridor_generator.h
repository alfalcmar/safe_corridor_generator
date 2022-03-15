#ifndef SAFE_CORRIDOR_GENERATOR_H
#define SAFE_CORRIDOR_GENERATOR_H

/* Includes //{ */
#include "decompose_wrapper.h"
#include "jps_manager.h"
#include <geometry_msgs/PoseArray.h>

//}

namespace safe_corridor_generator
{

/* //{ class SafeCorridorGenerator */

class SafeCorridorGenerator {

public:
  void initialize(std::string pcd_file_path, std::string world_frame, double robot_radius, double segment_margin, std::vector<double> local_bbx,
                  double jps_size_x, double jps_size_y, double jps_size_z, double jps_max_z, double jps_min_z, std::vector<float> map_frame_coordinates,
                  double inflation_jps, double map_resolution, double max_sampling_distance, int max_jps_expansions);

  void publishLastPath(ros::Publisher pub_path);

  void publishCorridor(decomp_ros_msgs::EllipsoidArrayPtr ell_array, ros::Publisher pub_ell);

  void publishCorridor(decomp_ros_msgs::PolyhedronArrayPtr pol_array, ros::Publisher pub_pol);

  void publishCorridor(const vec_E<Polyhedron<3>> &pol_array, ros::Publisher &pub_pol);

  void publishCorridor(ros::Publisher &pub_pol);

  void publishCloud(ros::Publisher pub_cloud);

  decomp_ros_msgs::PolyhedronArrayPtr getSafeCorridorPolyhedrons(const nav_msgs::PathConstPtr &initial_path);

  decomp_ros_msgs::EllipsoidArrayPtr getSafeCorridorEllipsoids(const nav_msgs::PathConstPtr &initial_path);

  vec_E<Polyhedron<3>> getSafeCorridorPolyhedronVector(const nav_msgs::PathConstPtr &initial_path);

  void addPositionOfRobotToPclMap(geometry_msgs::Point pose, double radius, int n_points);

  void addPositionsOfRobotsToPclMap(geometry_msgs::PoseArray poses, double radius, int n_points);

  void addPyramidToPclMap(geometry_msgs::Point pose, double heading, double pitch, double fov_length, double sampling_dist);

  void removeAddedPointsFromPclMap();

  void setCameraAngles(double aov_h, double aov_v);

  void setMaxSamplingDistance(double max_sampling_distance);

  void updateMaps(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &_pcd_input, const std::vector<float> &_pose);

  void updateMaps();

  nav_msgs::PathPtr getLastPath();

  bool isPointOccupied(Vec3f point);


private:
  // --------------------------------------------------------------
  // |                ROS-related member variables                |
  // --------------------------------------------------------------

  /* Parameters, loaded from ROS //{ */
  std::string world_frame_;
  std::string pcd_file_path_;  // path to pcd file containing representation of obstacles
  double      robot_radius_;
  double      segment_margin_;
  Vec3f       local_bbox_;

  int n_pcl_added_points_;
  int n_pcl_initial_points_;

  //}

  //}

private:
  // --------------------------------------------------------------
  // |                       Other variables                      |
  // --------------------------------------------------------------

  /* Other variables //{ */

  bool   initialized_;
  bool   got_pcl_map_;
  bool   got_path_;
  bool   test_;
  double camera_aov_h_;
  double camera_aov_v_;
  double max_sampling_distance_;

  std::unique_ptr<safe_corridor_generator::DecomposeWrapper> decompose_wrapper_;

  std::unique_ptr<safe_corridor_generator::JPSManager> jps_manager_;

  sensor_msgs::PointCloud2 pcl_sensor_message_;

  nav_msgs::PathPtr last_path_;

  vec_E<Polyhedron<3>> last_polyhedrons_;

  //}

private:
  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

  vec_Vec3f                  getCollisionFreePath(const nav_msgs::PathConstPtr &initial_path, bool equal_length_required);
  Vec3f                      positionToVec3f(geometry_msgs::Point point);
  nav_msgs::PathPtr          vec3fPathToNavMsgsPath(vec_Vec3f jps_path);
  std::vector<pcl::PointXYZ> generatePclSphere(int n_points, double radius, geometry_msgs::Point center);
  std::vector<pcl::PointXYZ> generatePclPyramid(geometry_msgs::Point center, double heading, double pitch, double fov_length, double sampling_dist);
  vec_Vec3f                  getSampledPath(vec_Vec3f initial_path, int n_waypoints, double max_sampling_distance);

  //}
};

}  // namespace safe_corridor_generator

#endif  // DECOMPOSE_NODE_H
