#ifndef DECOMPOSE_NODE_H
#define DECOMPOSE_NODE_H

/* Includes //{ */
#include "safe_corridor_generator.h"
#include <nodelet/nodelet.h>

//}

namespace safe_corridor_generator
{

/* //{ class DecomposeNode */

class DecomposeNode : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  void mainTimer(const ros::TimerEvent &evt);

private:
  // --------------------------------------------------------------
  // |                ROS-related member variables                |
  // --------------------------------------------------------------

  /* Parameters, loaded from ROS //{ */
  std::string world_frame_;
  std::string pcd_file_path_;  // path to pcd file containing representation of obstacles
  double      main_loop_rate_;
  double      robot_radius_;
  double      segment_margin_;
  Vec3f       local_bbox_;
  //}

  /* ROS related variables (subscribers, timers etc.) //{ */
  ros::NodeHandle nh_;

  ros::Subscriber sub_path_;

  ros::Publisher pub_corridor_ellipsoids_;
  ros::Publisher pub_corridor_polyhedrons_;
  ros::Publisher pub_point_cloud_;
  ros::Publisher pub_path_;

  ros::ServiceServer srvs_set_pcd_file_;

  ros::Timer main_timer_;

  //}

private:
  // --------------------------------------------------------------
  // |                       Other variables                      |
  // --------------------------------------------------------------

  /* Other variables //{ */

  bool initialized_;
  bool test_;

  SafeCorridorGenerator safe_corridor_generator_;

  //}

private:
  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

  void referencePathCallback(const nav_msgs::PathConstPtr &msg);

  template <typename T>
  bool loadParam(ros::NodeHandle& _nh, std::string const& _param_name, T& _param_value) {
    if (!_nh.getParam(_param_name, _param_value)) {
      ROS_ERROR("Failed to find parameter: %s",
                _nh.resolveName(_param_name, true).c_str());
      exit(1);
    }
    return true;
  } 
};

//}

}  // namespace safe_corridor_generator

#endif  // DECOMPOSE_NODE_H
