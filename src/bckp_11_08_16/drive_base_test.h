#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
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
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>




class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  ros::Publisher cmd_arm_pub_;
  ros::Publisher cmd_torso_pub_;
  ros::Publisher trajectoryPublisher_;
  ros::Subscriber base_goal_sub_;
  tf::TransformListener listener_;
  //we will record transforms here
  tf::StampedTransform start_transform_;
  tf::StampedTransform current_transform_;


  ////////////  Stuff for ik    //////////////////////////////////////////////////////////
  // using moveit:
  boost::shared_ptr<move_group_interface::MoveGroup> group_;
  tf::StampedTransform transformTorso_;
  // using kdl directly


public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh);

  void BlindfollowTrajectory(const moveit_msgs::DisplayTrajectory msg);
  void moveitIKfollowTrajectory(const geometry_msgs::PoseArray msg);
  void followGoal2(const geometry_msgs::PoseArray msg);

};