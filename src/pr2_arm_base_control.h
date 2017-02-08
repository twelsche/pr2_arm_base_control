#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Custom messages for GMM
// #include <pr2_arm_base_control/array1d.h>
// #include <pr2_arm_base_control/array2d.h>
// #include <pr2_arm_base_control/GMM.h>
// #include <pr2_arm_base_control/Mu.h>
// #include <pr2_arm_base_control/Priors.h>
// #include <pr2_arm_base_control/Sigma.h>


class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher cmd_arm_pub_;
  ros::Publisher cmd_torso_pub_;
  ros::Publisher DesPose_pub_;
  ros::Subscriber base_goal_sub_;
  ros::Subscriber base_scan_sub_;
  tf::TransformListener listener_; 
  moveit_msgs::GetPlanningScene scene_srv_;
  ros::ServiceClient client_get_scene_;

  // define parameters for obstacle from laserscan handling
  // std::std::vector<tf::Transform> obstaclePoints;

  
public:
  RobotDriver(ros::NodeHandle &nh);

  void followTrajectory(const geometry_msgs::PoseArray msg);

  void laserObstacleCallback(const sensor_msgs::LaserScan msg);



};

namespace validityFun
{
    bool validityCallbackFn(planning_scene::PlanningScenePtr &planning_scene,
                          // const kinematics_constraint_aware::KinematicsRequest &request,
                          // kinematics_constraint_aware::KinematicsResponse &response,
                          robot_state::RobotStatePtr kinematic_state,
                          const robot_state::JointModelGroup *joint_model_group,
                          const double *joint_group_variable_values
                          // const std::vector<double> &joint_group_variable_values
                          );

  }