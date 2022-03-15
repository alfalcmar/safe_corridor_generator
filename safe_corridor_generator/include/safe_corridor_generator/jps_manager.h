#ifndef JPS_MANAGER_H
#define JPS_MANAGER_H

#include "ros/ros.h"
#include <iostream>
#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_basis/data_utils.h>
/* #include <jps_basis/data_type.h> */
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


namespace safe_corridor_generator
{

class JPSManager {

public:
  JPSManager();

  void initialize(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double size_x, double size_y, double size_z, double max_z, double min_z,
                  std::vector<float> map_frame_coordinates, double robot_radius, double inflation_jps, double map_reoslution, int expansion_limit);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

  void        updateJPSMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr, Eigen::Vector3d& center);
  vec_Vecf<3> solveJPS3D(const Vec3f& start, const Vec3f& goal, bool& solved, bool plan_to_nearest);

  bool isPointOccupied(Vec3f point);

  void updateMap();

  void updateMap(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &_pcd_input);

  void freeStartPoint(const Vec3f& start);


private:
  std::shared_ptr<JPS::VoxelMapUtil> map_util_;
  std::unique_ptr<JPSPlanner3D>      planner_ptr_;
  Vec3f                              center_map_;
  vec_Vecf<3>                        path_;
  double                             factor_jps_, res_, inflation_jps_, z_ground_, z_max_, robot_radius_;
  double                             width_x_, width_y_, width_z_;
  int                                cells_x_, cells_y_, cells_z_;
  int                                max_jps_expansions_;

  bool initialized_;
};

}  // namespace safe_corridor_generator

#endif
