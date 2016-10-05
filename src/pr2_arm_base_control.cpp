#include "pr2_arm_base_control.h"


// Constructor
RobotDriver::RobotDriver(ros::NodeHandle &nh)
{
	nh_ = nh;
	base_goal_sub_ = nh_.subscribe("/PRCommunicator/Precomp_Trajectory", 10, &RobotDriver::followTrajectory,this);
	//set up the publishers for the robot controller topics
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
	cmd_arm_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/r_arm_controller/command", 1);
	cmd_torso_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
	//wait for the listener to get the first message
	listener_.waitForTransform("odom_combined","map",  ros::Time(0), ros::Duration(10.0));
	listener_.waitForTransform("map","base_footprint",  ros::Time(0), ros::Duration(10.0));
	listener_.waitForTransform("map","torso_lift_link",  ros::Time(0), ros::Duration(10.0));
}




// main callback function for motion generation
void RobotDriver::followTrajectory(const geometry_msgs::PoseArray msg)
{	
	// trnaform pose array to vector of poses for base and gripper
	// in map-Frame (waypointsWorld) and robot-Frame (waypoints)
	unsigned int traj_length = msg.poses.size();
	ROS_INFO("Trajectory msg with length %u received",traj_length);
	std::vector<geometry_msgs::Pose> waypointsWorld;
	std::vector<geometry_msgs::Pose> torsoPoints;
	// tf::Transform offset;
	// offset.setIdentity();
	// tf::Vector3 translationOff(0.05,0.0,0.0);
	// offset.setOrigin(translationOff);
	for (unsigned int j = 0 ; j < traj_length ; j++) 
	{
		if (j % 2)
		{
			// tf::Transform torsoTransform;
			// tf::poseMsgToTF(msg.poses[j],torsoTransform);      
			// torsoTransform = torsoTransform*offset;
			// geometry_msgs::Pose poseMsg;
			// tf::poseTFToMsg(torsoTransform,poseMsg);
			// torsoPoints.push_back(poseMsg); 
			torsoPoints.push_back(msg.poses[j]);
		}      
		else
		{
			waypointsWorld.push_back(msg.poses[j]);
		}
	}


 	// set initial pose for arm joints
	trajectory_msgs::JointTrajectory newArmPose;
    	trajectory_msgs::JointTrajectoryPoint jointPoint;
	int trajectoryLength = waypointsWorld.size();//fullBodyTraj_msg.joint_trajectory.points.size();
	newArmPose.points.push_back(jointPoint);
	newArmPose.points[0].time_from_start = ros::Duration(0.1);

	// set initialt torso pose
	trajectory_msgs::JointTrajectory newTorsoPose;
	trajectory_msgs::JointTrajectoryPoint torsoPoint;	
	torsoPoint.positions.push_back(torsoPoints[0].position.z-0.75);
	newTorsoPose.joint_names.push_back("torso_lift_joint");
	newTorsoPose.points.push_back(torsoPoint);
	newTorsoPose.points[0].time_from_start = ros::Duration(0.1);


	// Set up stuff for IK
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
	int counter = 0;
	for(std::size_t i=0; i < joint_names.size(); ++i)
	{
		ROS_INFO_STREAM(joint_names[i].c_str());
		if (i != 3 && i != 6) 
		{
			newArmPose.joint_names.push_back(joint_names[counter]);			
		}
		counter++;
	}

	// Collision constraint function GroupStateValidityCallbackFn(),
	planning_scene::PlanningSceneConstPtr planning_scene;
	// robot_state::GroupStateValidityCallbackFn constraint_callback_fn = boost::bind(&RobotDriver::validityCallbackFn,this, planning_scene, kinematic_state, _1,_2);

	// variables for execute precomputed trajectory:
	double motionDuration = msg.header.stamp.toSec();
	double initialTime = ros::Time::now().toSec();    	
	int currentPoseIdx = 0;
	int Idx = 0;
	geometry_msgs::Twist base_cmd;
	tf::Transform goal_transform;
    geometry_msgs::Transform transform;
    geometry_msgs::Pose currentDesiredBasepose;

	// Loop for executing precomputed trajectory
	// desired rate for the control loop
	ros::Rate rate(50.0);
	bool done = false;
	while (!done && nh_.ok())
	{
		double currentTime = ros::Time::now().toSec();		
		currentTime = (currentTime- initialTime)/10.0;
		// find closest point to currentTime
		Idx = round(trajectoryLength*currentTime/motionDuration);
		if (Idx > trajectoryLength-1)
			Idx = trajectoryLength-1;
		if (Idx != currentPoseIdx)
		{			
			currentPoseIdx = Idx;
			ROS_INFO("currentPoseIdx: %d, trajectoryLength: %d",currentPoseIdx,trajectoryLength-1);
			currentDesiredBasepose = torsoPoints[currentPoseIdx];
			tf::poseMsgToTF(currentDesiredBasepose,goal_transform);

	    	// set and command new torso configuration
	    	trajectory_msgs::JointTrajectoryPoint newTorsoPoint;
	    	// newTorsoPoint.positions.push_back(fullBodyTraj_msg.joint_trajectory.points[currentPoseIdx].positions[nrJoints-1]);
	    	newTorsoPoint.positions.push_back(torsoPoints[currentPoseIdx].position.z-0.75);
	    	newTorsoPose.points[0] = newTorsoPoint;
	    	newTorsoPose.points[0].time_from_start = ros::Duration(0.1);
	    	cmd_torso_pub_.publish(newTorsoPose);
		}

		//get the current base pose
		tf::StampedTransform current_transform;
		try
		{
			// listener_.lookupTransform("odom_combined", "base_footprint", ros::Time(0), current_transform);
			listener_.lookupTransform("map", "base_footprint", ros::Time(0), current_transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			break;
		}		
		// calculate base velocity based on current and desired base poses
		tf::Transform relative_desired_pose = current_transform.inverse() * goal_transform;
		double roll_current,pitch_current,yaw_current;
		relative_desired_pose.getBasis().getRPY(roll_current,pitch_current,yaw_current);
		base_cmd.angular.z = yaw_current*1.0;
		base_cmd.linear.x = relative_desired_pose.getOrigin().getX()*1.0;
		base_cmd.linear.y = relative_desired_pose.getOrigin().getY()*1.0;
		// send out command for base velocity
		cmd_vel_pub_.publish(base_cmd);

		// find ik for arm and command new pose:
		tf::Transform currentGripperTransform;
		tf::poseMsgToTF(waypointsWorld[currentPoseIdx],currentGripperTransform); 
		currentGripperTransform = current_transform.inverse()*currentGripperTransform;
		geometry_msgs::Pose gripperPoseMsg;
		tf::poseTFToMsg(currentGripperTransform,gripperPoseMsg);
		gripperPoseMsg.position.z -=  newTorsoPose.points[0].positions[0] - 0.17;// 0.12; // offset don"t know why this is needed???????

		Eigen::Affine3d state;
		tf::poseMsgToEigen(gripperPoseMsg,state);
		const Eigen::Affine3d &desiredState = state;

		// TODO: Include collision checking with collision constraint function GroupStateValidityCallbackFn(),

		// Get IK solution from desired cartesian state:
		bool found_ik = kinematic_state->setFromIK(joint_model_group, desiredState, 5, 0.1);
		// If found solution create and publish arm pose command
		if (found_ik)
		{
			std::vector<double> joint_values;
			std::vector<std::string> joint_names2;
			kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
			trajectory_msgs::JointTrajectoryPoint armJointPoint;
			int counter = 0;
			for(std::size_t i=0; i < joint_names.size(); ++i)
			{
				if (i != 3 && i != 6) 
				{
					armJointPoint.positions.push_back(joint_values[counter]);
					counter++;
				}
				
			}
			newArmPose.points[0] = armJointPoint;
	    	newArmPose.points[0].time_from_start = ros::Duration(0.1);
			// publish command for the new joint pose for robot arm
	    	cmd_arm_pub_.publish(newArmPose);
		}	
		else
			ROS_INFO("IK not found");	
		
		// check if reached end of motion
		if(currentTime > motionDuration){
			done = true;
			double dist_goal = relative_desired_pose.getOrigin().length();
			ROS_INFO("Motion finished. Remaining distance for base to goal: %g m)",dist_goal);
		} 
		// try to keep the loop at constant rate:
	  	rate.sleep();
	}
	
}





int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  ros::spin();  
}



bool RobotDriver::validityCallbackFn(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                   	// const kinematics_constraint_aware::KinematicsRequest &request,
                                   	// kinematics_constraint_aware::KinematicsResponse &response,
									robot_state::RobotStatePtr kinematic_state,
                                   	robot_state::JointModelGroup *joint_model_group,
                                   	const std::vector<double> &joint_group_variable_values) const
{
  kinematic_state->setJointGroupPositions(joint_model_group,joint_group_variable_values);  
  // Now check for collisions

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;  
    // collision_request.group_name = request.group_name_;
    planning_scene->checkCollision(collision_request, collision_result, *kinematic_state);    
    if(collision_result.collision)
    {
      logDebug("IK solution is in collision");      
      // response.error_code_.val = response.error_code_.GOAL_IN_COLLISION;
      return false;      
    }    
 
  return true;  
}
