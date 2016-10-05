#include "drive_base_test.h"


RobotDriver::RobotDriver(ros::NodeHandle &nh)
{
nh_ = nh;
//base_goal_sub_ = nh_.subscribe("RobotTrajectory", 10, &RobotDriver::followGoal,this);
base_goal_sub_ = nh_.subscribe("/PRCommunicator/Precomp_Trajectory", 10, &RobotDriver::followGoal2,this);
//set up the publishers for the robot controller topics
cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
cmd_arm_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/r_arm_controller/command", 1);
cmd_torso_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
// Publisher for robotTrajectory (for visualization)
trajectoryPublisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("RobotTrajectory",1,false);
//wait for the listener to get the first message
listener_.waitForTransform("odom_combined","map",  ros::Time(0), ros::Duration(10.0));
listener_.waitForTransform("map","base_footprint",  ros::Time(0), ros::Duration(10.0));
//record the starting transform from the map to the base frame
try
  {
    // listener_.lookupTransform("odom_combined", "base_footprint", ros::Time(0), start_transform_);
    listener_.lookupTransform("map","base_footprint", ros::Time(0), start_transform_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
current_transform_ = start_transform_;
try
{
	listener_.lookupTransform("odom_combined", "map",ros::Time(0), transformTorso2_);
}
catch (tf::TransformException ex)
{
	ROS_ERROR("%s",ex.what());
}

try
{
        listener_.lookupTransform("map", "torso_lift_link",ros::Time(0), transformTorso_);
}
catch (tf::TransformException ex)
{
        ROS_ERROR("%s",ex.what());
}


// start a move group for right arm to generate seed for ik-solver
group_.reset(new moveit::planning_interface::MoveGroup ("right_arm"));

}




// main callback function for motion generation
void RobotDriver::followGoal2(const geometry_msgs::PoseArray msg)
{
	// get the current pose of the robot
	try
	{
		listener_.lookupTransform("map", "torso_lift_link",ros::Time(0), transformTorso_);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}

    try
    {
            listener_.lookupTransform("odom_combined", "torso_lift_link",ros::Time(0), transformTorso2_);
    }
    catch (tf::TransformException ex)
    {
            ROS_ERROR("%s",ex.what());
    }
	
	// trnaform pose array to vector of poses for base and gripper
	// in map-Frame (waypointsWorld) and robot-Frame (waypoints)
	unsigned int traj_length = msg.poses.size();
	ROS_INFO("Trajectory msg with length %u received",traj_length);
	std::vector<geometry_msgs::Pose> waypoints;
	std::vector<geometry_msgs::Pose> waypointsWorld;
	std::vector<geometry_msgs::Pose> torsoPoints;
	tf::Transform offset;
	offset.setIdentity();
	// tf::Vector3 translationOff(0.05,0.0,0.0);
	tf::Vector3 translationOff(0.0,0.0,0.0);
	offset.setOrigin(translationOff);
	// tf::Transform gripperOffset;
	// gripperOffset.setIdentity();
	// tf::Vector3 gripperTranslationOff(-0.18,0.0,0.0);
	// gripperOffset.setOrigin(gripperTranslationOff);
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
		}      
		else
		{
			tf::Transform gripperTransform;
			tf::poseMsgToTF(msg.poses[j],gripperTransform);
			tf::Transform torsoTransform;
			tf::poseMsgToTF(msg.poses[j+1],torsoTransform);
			torsoTransform = torsoTransform*offset;
			torsoTransform = transformTorso2_.inverse()*torsoTransform;  
			// gripperTransform = torsoTransform.inverse()*gripperTransform*transformTorso2_;
			gripperTransform = torsoTransform.inverse()*gripperTransform;
			geometry_msgs::Pose poseMsg;
			tf::poseTFToMsg(gripperTransform,poseMsg);
			waypoints.push_back(poseMsg);
			waypointsWorld.push_back(msg.poses[j]);
		}
	}
	// stop any running motion before staring the new
	group_->stop(); 
	group_->setPlanningTime(30);
	//compute joint trajectory
	moveit_msgs::RobotTrajectory trajectory_msg;
	moveit_msgs::RobotTrajectory fullBodyTraj_msg;
	double fraction = group_->computeCartesianPath(waypoints,50.0, 0.0, trajectory_msg,false); 
	ROS_INFO("Achieved fraction %g of total path",fraction*100);
	// if (fraction < 0.5){
	// 	ROS_INFO("Planning motion failed aborting");
	// 	return;
	// }
	// add torso poses to Trajectory
	fullBodyTraj_msg.joint_trajectory.joint_names = trajectory_msg.joint_trajectory.joint_names;
	fullBodyTraj_msg.joint_trajectory.joint_names.push_back("torso_lift_joint");
	fullBodyTraj_msg.joint_trajectory.header = msg.header;
	// fullBodyTraj_msg.multi_dof_joint_trajectory.header.frame_id = "odom_combined";
	fullBodyTraj_msg.multi_dof_joint_trajectory.header.frame_id = "map";
	fullBodyTraj_msg.multi_dof_joint_trajectory.joint_names.push_back("world_joint");
	unsigned int moveitArmtraj_length = trajectory_msg.joint_trajectory.points.size();

	for (int i = 0; i < traj_length/2;i++)
	{
		// Torso_lift_link trajectory
		trajectory_msgs::JointTrajectoryPoint jointPoint;
		if (i<moveitArmtraj_length)
             		jointPoint = trajectory_msg.joint_trajectory.points[i];
		else
			jointPoint = trajectory_msg.joint_trajectory.points[moveitArmtraj_length-1];
		int closestRelative;
		closestRelative = i;
		// add torso position
		jointPoint.positions.push_back(torsoPoints[closestRelative].position.z-0.75);
		fullBodyTraj_msg.joint_trajectory.points.push_back(jointPoint);
		// Base Trajectory
		trajectory_msgs::MultiDOFJointTrajectoryPoint basePoint;
		geometry_msgs::Transform transform;
		transform.translation.x = torsoPoints[closestRelative].position.x;
		transform.translation.y = torsoPoints[closestRelative].position.y;
		transform.translation.z = 0; 
		transform.rotation = torsoPoints[closestRelative].orientation; 
		basePoint.transforms.push_back(transform);
		fullBodyTraj_msg.multi_dof_joint_trajectory.points.push_back(basePoint); 
	}
	ROS_INFO("Done building fullBody Trajectory");
	// Build Display Trajectory for visualization purposes only
	moveit_msgs::DisplayTrajectory displayTrajectory;
	displayTrajectory.model_id = "pr2";
	displayTrajectory.trajectory.push_back(fullBodyTraj_msg);
	// Set startstate
	moveit_msgs::RobotState start_state;
	int m_num_joints = 8;
	start_state.joint_state.name.resize(m_num_joints);
	start_state.joint_state.position.resize(m_num_joints);
	start_state.joint_state.velocity.resize(m_num_joints); 
	start_state.joint_state.name = fullBodyTraj_msg.joint_trajectory.joint_names;
	for (int j = 0 ; j < m_num_joints; j++)
	{
		start_state.joint_state.position[j] = fullBodyTraj_msg.joint_trajectory.points[0].positions[j];  
	}
	// start_state.multi_dof_joint_state.header.frame_id = "odom_combined";
	start_state.multi_dof_joint_state.header.frame_id = "map";
	start_state.multi_dof_joint_state.joint_names.push_back("world_joint");
	geometry_msgs::Transform startTransform;
	startTransform.translation.x = 0;
	startTransform.translation.y = 0;
	startTransform.translation.z = 0;
	startTransform.rotation.x = 0;
	startTransform.rotation.y = 0;
	startTransform.rotation.z = 0;
	startTransform.rotation.w = 1;
	start_state.multi_dof_joint_state.transforms.push_back(startTransform);
	displayTrajectory.trajectory_start = start_state; //set robot start state
	ROS_INFO_STREAM("Publish fullBodyTraj_msg:");
	trajectoryPublisher_.publish(displayTrajectory);

	moveit_msgs::RobotTrajectory robotTraj = fullBodyTraj_msg;


 	// set initial pose for arm joints
	trajectory_msgs::JointTrajectory initialArmPose;
    	trajectory_msgs::JointTrajectoryPoint jointPoint;
	int trajectoryLength = robotTraj.joint_trajectory.points.size();
    	int nrJoints = robotTraj.joint_trajectory.joint_names.size();
    	for (int i = 0; i< nrJoints; i++)
    	{
    		initialArmPose.joint_names.push_back(robotTraj.joint_trajectory.joint_names[i]);
    		jointPoint.positions.push_back(robotTraj.joint_trajectory.points[0].positions[i]);
    	}
    	initialArmPose.points.push_back(jointPoint);
    	initialArmPose.points[0].time_from_start = ros::Duration(0.1);
    	// cmd_arm_pub_.publish(initialArmPose);

    	// set initialt torso pose
	trajectory_msgs::JointTrajectory initialTorsoPose;
    	trajectory_msgs::JointTrajectoryPoint torsoPoint;
    	initialTorsoPose.joint_names.push_back("torso_lift_joint");
    	torsoPoint.positions.push_back(robotTraj.joint_trajectory.points[0].positions[nrJoints-1]);
    	initialTorsoPose.points.push_back(torsoPoint);
    	initialTorsoPose.points[0].time_from_start = ros::Duration(0.1);
    	// cmd_torso_pub_.publish(initialTorsoPose);

	// set initial base pose
	trajectory_msgs::MultiDOFJointTrajectoryPoint basePoint = robotTraj.multi_dof_joint_trajectory.points[0];

	// Set up stuff for IK
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
	// robot_state::JointStateGroup joint_state_group = kinematic_state->getJointStateGroup("right_arm") ;

	// variables for execute precomputed trajectory:
	double motionDuration = robotTraj.joint_trajectory.header.stamp.toSec();
	double initialTime = ros::Time::now().toSec();
    	bool done = false;
	int currentPoseIdx = 0;
	int Idx = 0;
	geometry_msgs::Twist base_cmd;
	trajectory_msgs::JointTrajectory newArmPose;
	newArmPose = initialArmPose;
	newArmPose.points[0].time_from_start = ros::Duration(0.1);
	trajectory_msgs::JointTrajectory newTorsoPose;
	newTorsoPose = initialTorsoPose;
	newTorsoPose.points[0].time_from_start = ros::Duration(0.1);  
	tf::Transform goal_transform;
        geometry_msgs::Transform transform;
        geometry_msgs::Pose pose;
	// desired rate for the control loop
	ros::Rate rate(50.0);



	// Collision constrain function GroupStateValidityCallbackFn(),
	planning_scene::PlanningSceneConstPtr planning_scene;
	// robot_state::GroupStateValidityCallbackFn constraint_callback_fn = boost::bind(&RobotDriver::validityCallbackFn,this, planning_scene, kinematic_state, _1,_2);
	// execute precomputed trajectory
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
			// set new desired base position
			basePoint = robotTraj.multi_dof_joint_trajectory.points[currentPoseIdx];
			transform = basePoint.transforms[0];
			pose.position.x = transform.translation.x;
			pose.position.y = transform.translation.y;
			pose.position.z = transform.translation.z;
			pose.orientation = transform.rotation;
			tf::poseMsgToTF(pose,goal_transform);


		    	// set and command new torso configuration
		    	trajectory_msgs::JointTrajectoryPoint torsoPoint;
		    	torsoPoint.positions.push_back(robotTraj.joint_trajectory.points[currentPoseIdx].positions[nrJoints-1]);
		    	newTorsoPose.points[0] = torsoPoint;
		    	newTorsoPose.points[0].time_from_start = ros::Duration(0.1);
		    	cmd_torso_pub_.publish(newTorsoPose);
		}

		//get the current base pose
		try
		{
			// listener_.lookupTransform("odom_combined", "base_footprint", ros::Time(0), current_transform_);
			listener_.lookupTransform("map", "base_footprint", ros::Time(0), current_transform_);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			break;
		}		
		// calculate base velocity based on current and desired base poses
		tf::Transform relative_desired_pose = current_transform_.inverse() * goal_transform;
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
		currentGripperTransform = current_transform_.inverse()*currentGripperTransform;
		geometry_msgs::Pose gripperPoseMsg;
		tf::poseTFToMsg(currentGripperTransform,gripperPoseMsg);
		gripperPoseMsg.position.z -=  newTorsoPose.points[0].positions[0] - 0.17;// 0.12; // offset don"t know why this is needed???????

		Eigen::Affine3d state;
		tf::poseMsgToEigen(gripperPoseMsg,state);
		const Eigen::Affine3d &desiredState = state;
		// TODO: Include waypoint[currentPoseIdx] as seed for ik

		// Torso_lift_link trajectory
		trajectory_msgs::JointTrajectoryPoint jointPoint;
		if (currentPoseIdx<moveitArmtraj_length)
		{
			std::vector< double > gstate;
			int counter = 0;
			for(std::size_t i=0; i < joint_names.size(); ++i)
			{
				if (i != 3 && i != 6) 
				{
					counter++;
				}
				gstate.push_back(fullBodyTraj_msg.joint_trajectory.points[currentPoseIdx].positions[counter]);
			}
			kinematic_state->setJointGroupPositions(joint_model_group,gstate);
			// ROS_INFO("Seeded IK");
		}  



		bool found_ik = kinematic_state->setFromIK(joint_model_group, desiredState, 5, 0.1);
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
			ROS_INFO("Motion finished. Remaining distance to goal: %g)",dist_goal);
		} 
		// try to keep the loop at sconstant rate:
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
