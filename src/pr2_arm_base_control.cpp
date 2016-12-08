#include "pr2_arm_base_control.h"


// Constructor
RobotDriver::RobotDriver(ros::NodeHandle &nh)
{
	nh_ = nh;
	base_goal_sub_ = nh_.subscribe("/PRCommunicator/Precomp_Trajectory", 10, &RobotDriver::followTrajectory,this);

	base_scan_sub_ = nh_.subscribe("/base_scan", 10, &RobotDriver::laserObstacleCallback,this);
	//set up the publishers for the robot controller topics
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
	cmd_arm_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/r_arm_controller/command", 1);
	cmd_torso_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
	//wait for the listener to get the first message
	listener_.waitForTransform("odom_combined","map",  ros::Time(0), ros::Duration(10.0));
	listener_.waitForTransform("map","base_footprint",  ros::Time(0), ros::Duration(10.0));
	listener_.waitForTransform("map","torso_lift_link",  ros::Time(0), ros::Duration(10.0));
	// Start planning scene client:
	client_get_scene_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
}




// main callback function for motion generation, Following precomputed trajectory, TODO: Online motion generation from Model
void RobotDriver::followTrajectory(const geometry_msgs::PoseArray msg)
{	
	// trnaform pose array to vector of poses for base and gripper
	// in map-Frame (waypointsWorld) and robot-Frame (waypoints)
	unsigned int traj_length = msg.poses.size();
	ROS_INFO("Trajectory msg with length %u received",traj_length);
	std::vector<geometry_msgs::Pose> waypointsWorld;
	std::vector<geometry_msgs::Pose> torsoPoints;
	tf::Transform offset;
	offset.setIdentity();
	tf::Vector3 translationOff(0.05,0.0,0.0);
	offset.setOrigin(translationOff);
	for (unsigned int j = 0 ; j < traj_length ; j++) 
	{
		if (j % 2)
		{
			tf::Transform torsoTransform;
			tf::poseMsgToTF(msg.poses[j],torsoTransform);      
			torsoTransform = torsoTransform*offset;
			geometry_msgs::Pose poseMsg;
			tf::poseTFToMsg(torsoTransform,poseMsg);
			torsoPoints.push_back(poseMsg); 
			// torsoPoints.push_back(msg.poses[j]);
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
	pr2_arm_base_control::array1d d;
	d.a.push_back(4.5);
	d.a.push_back(4.3);
	pr2_arm_base_control::GMM gmm_msg;
	geometry_msgs::Pose test;
	gmm_msg.mu.gripperPoses.push_back(test);
	gmm_msg.mu.basePoses.push_back(test);
	// Collision constraint function GroupStateValidityCallbackFn(),
	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
	robot_state::GroupStateValidityCallbackFn constraint_callback_fn = boost::bind(&validityFun::validityCallbackFn, planning_scene, kinematic_state,_2,_3);
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

		// TODO: calculate desired pose based on model, currentPose and time
		// 
		//
		// 

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
		// Transform to /base_footprint frame because planning scene operates here 
		currentGripperTransform = current_transform.inverse()*currentGripperTransform;
		geometry_msgs::Pose gripperPoseMsg;
		tf::poseTFToMsg(currentGripperTransform,gripperPoseMsg);
		gripperPoseMsg.position.z -=  newTorsoPose.points[0].positions[0] - 0.17;// 0.12; // offset don"t know why this is needed???????
		Eigen::Affine3d state;
		tf::poseMsgToEigen(gripperPoseMsg,state);
		const Eigen::Affine3d &desiredState = state;

		// TODO: Include collision checking with collision constraint function GroupStateValidityCallbackFn()
		// update planning scene
		moveit_msgs::PlanningScene currentScene;
		moveit_msgs::GetPlanningScene scene_srv;
		// scene_srv.request.components.components = scene_srv.request.components.WORLD_OBJECT_GEOMETRY;
		scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX + 
												  scene_srv.request.components.WORLD_OBJECT_GEOMETRY +
												  scene_srv.request.components.OCTOMAP + 
												  scene_srv.request.components.SCENE_SETTINGS;
		if(!client_get_scene_.call(scene_srv))
		{
			ROS_WARN("Failed to call service /get_planning_scene");
		}
		else
		{ 
			currentScene = scene_srv.response.scene;
		}
		planning_scene->setPlanningSceneMsg(currentScene);


		// moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;    
	    // ROS_INFO("acm size: %lu",currentACM.entry_names.size());
	    // ROS_INFO("acm name: %s",currentACM.entry_names[0].c_str());
    

		// Get IK solution from desired cartesian state:
		bool found_ik = kinematic_state->setFromIK(joint_model_group, desiredState, 5, 0.1,constraint_callback_fn);
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
			double dist_goal = sqrt(relative_desired_pose.getOrigin().x()*relative_desired_pose.getOrigin().x()+relative_desired_pose.getOrigin().y()*relative_desired_pose.getOrigin().y());
			ROS_INFO("Motion finished. Remaining distance for base to goal: %g m)",dist_goal);
		} 
		// try to keep the loop at constant rate:
	  	rate.sleep();
	}
	
}


void RobotDriver::laserObstacleCallback(const sensor_msgs::LaserScan msg)
{
	// TODO: calculate modulation for velocity from obstacles in laserscan
	// ROS_INFO("LaserScan received");
}





int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  ros::spin();  
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace validityFun
{

	bool validityCallbackFn(planning_scene::PlanningScenePtr &planning_scene,
	                        // const kinematics_constraint_aware::KinematicsRequest &request,
	                        // kinematics_constraint_aware::KinematicsResponse &response,
							robot_state::RobotStatePtr kinematic_state,
	                        const robot_state::JointModelGroup *joint_model_group,
	                        const double *joint_group_variable_values
	                        // const std::vector<double> &joint_group_variable_values
	                        ) 
	{
	  	kinematic_state->setJointGroupPositions(joint_model_group,joint_group_variable_values);  
	  	// Now check for collisions	  	
	    collision_detection::CollisionRequest collision_request;
	    // collision_request.group_name = "right_arm";
	    collision_detection::CollisionResult collision_result; 
	    // ROS_INFO("Planning frame: %s",planning_scene->getPlanningFrame().c_str());



	    collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
	    // moveit_msgs::AllowedCollisionMatrix acm_msg;
	    // acm.getMessage(acm_msg);
	    // ROS_INFO("acm_msg size: %lu",acm_msg.entry_names.size());
	    // ROS_INFO("acm_msg name: %s",acm_msg.entry_names[0].c_str());
	    planning_scene->checkCollision(collision_request, collision_result, planning_scene->getCurrentState());  
	    // planning_scene->checkCollision(collision_request, collision_result, planning_scene->getCurrentState(),acm);    
	    // planning_scene->checkCollision(collision_request, collision_result, *kinematic_state,acm);    
	    if(collision_result.collision)
	    {
	      logDebug("IK solution is in collision");      
	      // response.error_code_.val = response.error_code_.GOAL_IN_COLLISION;
	      return false;      
	    }    
	  return true;  
	}
}