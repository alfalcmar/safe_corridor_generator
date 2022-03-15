#include <safe_corridor_generator/jps_manager.h>

using namespace safe_corridor_generator;

JPSManager::JPSManager() : map_util_(std::make_shared<JPS::VoxelMapUtil>()), planner_ptr_(std::make_unique<JPSPlanner3D>(true)) {
  // load parameters

  initialized_ = true;
}

void JPSManager::initialize(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double size_x, double size_y, double size_z, double max_z, double min_z,
                            std::vector<float> map_frame_coordinates, double robot_radius, double inflation_jps, double map_resolution, int max_jps_expansions) {
  width_x_       = size_x;
  width_y_       = size_y;
  width_z_       = size_z;
  z_max_         = max_z;
  z_ground_      = min_z;
  robot_radius_  = robot_radius;
  inflation_jps_ = inflation_jps;
  res_           = map_resolution;
  cloud_         = cloud;
  max_jps_expansions_ = max_jps_expansions;

  // conversion to cells
  cells_x_ = width_x_ / res_;
  cells_y_ = width_y_ / res_;
  cells_z_ = width_z_ / res_;

  std::cout << "cells "
            << "[" << cells_x_ << "," << cells_y_ << "," << cells_z_ << "]" << std::endl;

  // read and create the map from a pcd
  center_map_ = Vec3f(map_frame_coordinates.at(0), map_frame_coordinates.at(1), map_frame_coordinates.at(2));
  ROS_INFO("Read map");
  ros::WallTime start = ros::WallTime::now();
  map_util_->readMap(cloud_, cells_x_, cells_y_, cells_z_, res_, center_map_, z_ground_, z_max_, inflation_jps_);  // Map read
ROS_INFO("Read map lasts %.4f s ", (ros::WallTime::now() - start).toSec());
  planner_ptr_->setMapUtil(map_util_);

  initialized_ = true;
  ROS_INFO("[JPSManager]: initialized ");
}

void JPSManager::updateMap() { 
  ROS_INFO("[JPSManager]: Update map");
  ros::WallTime start = ros::WallTime::now();
  map_util_->readMap(cloud_, cells_x_, cells_y_, cells_z_, res_, center_map_, z_ground_, z_max_, inflation_jps_);  // Map read
  ROS_INFO("[JPSManager]: Map update lasts %.2f s ", (ros::WallTime::now() - start).toSec());
}

void JPSManager::updateMap(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &_pcd_input) { 
  ROS_INFO("[JPSManager]: Update map");

  // UPDATE CLOUD FROM _pcd_input
  cloud_ = _pcd_input;

  ros::WallTime start = ros::WallTime::now();
  map_util_->readMap(cloud_, cells_x_, cells_y_, cells_z_, res_, center_map_, z_ground_, z_max_, inflation_jps_);  // Map read
  ROS_INFO("[JPSManager]: Map update lasts %.2f s ", (ros::WallTime::now() - start).toSec());
}

vec_Vecf<3> JPSManager::solveJPS3D(const Vec3f& start, const Vec3f& goal, bool& solved, bool plan_to_nearest) {

  bool valid_jps = planner_ptr_->plan(start, goal, 1, true, max_jps_expansions_, plan_to_nearest);  // start and goal in coordinates (m)
                                                              // Plan from start to goal with heuristic weight=1, and
                                                              // using JPS (if false --> use A*)

  vec_Vecf<3> path;

  if (valid_jps == true)  // There is a solution
  {
    path = planner_ptr_->getPath();  // getpar_.RawPath() if you want the path with more corners (not "cleaned")
    if (path.size() > 1) {
      path[0]               = start;
      path[path.size() - 1] = goal;  // force to start and end in the start and goal (and not somewhere in the voxel)
    } else {                         // happens when start and goal are very near (--> same cell)
      vec_Vecf<3> tmp;
      tmp.push_back(start);
      tmp.push_back(goal);
      path = tmp;
    }
  } else {
    std::cout << "JPS didn't find a solution from" << start.transpose() << " to " << goal.transpose() << std::endl;
  }
  solved = valid_jps;
  return path;
}

bool JPSManager::isPointOccupied(Vec3f point) {
  return planner_ptr_->isPointOccupied(point);
}

void JPSManager::freeStartPoint(const Vec3f& start){
  Vec3i start_pose_cell =  map_util_->floatToInt(start);
   return map_util_->setFree(start_pose_cell);
}
