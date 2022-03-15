#ifndef DECOMPOSE_WRAPPER_H
#define DECOMPOSE_WRAPPER_H

/* Includes //{ */
// ROS includes
#include <ros/package.h>
#include <ros/ros.h>

// Convex Decomposition includes
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>

// Includes from this package
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
//}

namespace safe_corridor_generator
{

/* //{ class DecomposeWrapper */

class DecomposeWrapper {


public:
  void initialize();

  bool loadPCLMap(const std::string &file_path);

  decomp_ros_msgs::EllipsoidArray getCorridorsEllipsoid(const nav_msgs::PathConstPtr &reference_path);

  decomp_ros_msgs::PolyhedronArray getCorridorsPolyhedron(const nav_msgs::PathConstPtr &reference_path);

  vec_E<Polyhedron<3>> getCorridorsPolyhedronVector(const nav_msgs::PathConstPtr &reference_path);

  sensor_msgs::PointCloud2 getPCLMessage();

  bool getGotCorridors();

  sensor_msgs::PointCloud2 getPCLMap();

  void printInequalityConstraints();

  void setRobotRadius(double robot_radius);

  void setSegmentMargin(double segment_margin);

  void setLocalBbox(Vec3f local_bbox);

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getPclCloud();

  bool updateMap();

  bool updateMap(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &_pcd_input);

private:
  void mainLoop([[maybe_unused]] const ros::TimerEvent &evt);

private:
  // --------------------------------------------------------------
  // |                ROS-related member variables                |
  // --------------------------------------------------------------

  /* Parameters, loaded from ROS //{ */
  std::string world_frame_;
  //}

  /* ROS related variables (subscribers, timers etc.) //{ */
  //}

private:
  // --------------------------------------------------------------
  // |                       Other variables                      |
  // --------------------------------------------------------------

  /* Other variables //{ */
  std::string pcd_file_path_;  // path to pcd file containing representation of obstacles
  double      robot_radius_;
  double      segment_margin_;
  Vec3f       local_bbox_;
  vec_Vec3f   pcl_map_vector_;
  vec_Vec3f   path_vector_;

  EllipsoidDecomp3D decomp_util_;

  bool got_path_;
  bool got_pcl_map_;
  bool got_corridors_;
  bool initialized_;
  bool pcl_map_changed_;

  sensor_msgs::PointCloud2 pcl_sensor_message_;

  double main_loop_rate_;
  
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud_;

  //}

private:
  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

  vec_Vec3f pathToVector(const nav_msgs::PathConstPtr &path_ref);
  bool      generateCorridors(vec_Vec3f reference_path);
};

//}

}  // namespace safe_corridor_generator

#endif  // DECOMPOSE_WRAPPER_H
