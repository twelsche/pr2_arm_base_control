#include "drive_base_test.h"


RobotDriver::RobotDriver(ros::NodeHandle &nh)
{
nh_ = nh;
// base_goal_sub_ = nh_.subscribe("RobotTrajectory", 10, &RobotDriver::BlindfollowTrajectory,this);
base_goal_sub_ = nh_.subscribe("/PRCommunicator/Precomp_Trajectory", 10, &RobotDriver::followGoal2,this);
//set up the publisher for the cmd_vel topic
cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
cmd_arm_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/r_arm_controller/command", 1);
cmd_torso_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
// Publisher for robotTrajectory
trajectoryPublisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("RobotTrajectory",1,false);
//wait for the listener to get the first message
listener_.waitForTransform("odom_combined","base_footprint",  
                           ros::Time(0), ros::Duration(10.0));
//record the starting transform from the odometry to the base frame
try
  {
    listener_.lookupTransform("odom_combined", "base_footprint", 
                              ros::Time(0), start_transform_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }
current_transform_ = start_transform_;
try
{
	listener_.lookupTransform("odom_combined", "torso_lift_link",
                          ros::Time(0), transformTorso_);
}
catch (tf::TransformException ex)
{
	ROS_ERROR("%s",ex.what());
}

// new stuff ik driver
group_.reset(new moveit::planning_interface::MoveGroup ("right_arm"));

}
















void RobotDriver::followGoal2(const geometry_msgs::PoseArray msg)
{
	try
	{
		listener_.lookupTransform("odom_combined", "torso_lift_link",
	                          ros::Time(0), transformTorso_);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	unsigned int traj_length = msg.poses.size();
	ROS_INFO("Trajectory msg with length %u received",traj_length);
	std::vector<geometry_msgs::Pose> waypoints;
	std::vector<geometry_msgs::Pose> waypointsWorld;
	std::vector<geometry_msgs::Pose> torsoPoints;
	tf::Transform offset;
	offset.setIdentity();
	tf::Vector3 translationOff(0.05,0.0,0.0);
	offset.setOrigin(translationOff);
	tf::Transform gripperOffset;
	gripperOffset.setIdentity();
	tf::Vector3 gripperTranslationOff(-0.18,0.0,0.0);
	gripperOffset.setOrigin(gripperTranslationOff);
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
			torsoTransform = transformTorso_.inverse()*torsoTransform;  
			gripperTransform = torsoTransform.inverse()*gripperTransform;
			geometry_msgs::Pose poseMsg;
			tf::poseTFToMsg(gripperTransform,poseMsg);
			waypoints.push_back(poseMsg);
			waypointsWorld.push_back(msg.poses[j]);
		}
	}
	ROS_INFO_STREAM("Startpoint after Transform:");
	// stop any running motion before staring the new
	group_->stop(); 
	group_->setPlanningTime(30);
	//compute joint trajectory
	moveit_msgs::RobotTrajectory trajectory_msg;
	moveit_msgs::RobotTrajectory fullBodyTraj_msg;
	// double fraction = group_->computeCartesianPath(waypoints,1.9, 15.5, trajectory_msg,true); // jump_threshold  // eef_step
	double fraction = group_->computeCartesianPath(waypoints,20.0, 0.0, trajectory_msg,false); 
	ROS_INFO("Achieved fraction %g of total path",fraction);
	// add torso poses to Trajectory
	fullBodyTraj_msg.joint_trajectory.joint_names = trajectory_msg.joint_trajectory.joint_names;
	fullBodyTraj_msg.joint_trajectory.joint_names.push_back("torso_lift_joint");
	fullBodyTraj_msg.joint_trajectory.header = msg.header;
	fullBodyTraj_msg.multi_dof_joint_trajectory.header.frame_id = "odom_combined";
	fullBodyTraj_msg.multi_dof_joint_trajectory.joint_names.push_back("world_joint");
	traj_length = trajectory_msg.joint_trajectory.points.size()-1;

	for (int i = 0; i < traj_length;i++)
	{
		// Torso_lift_link trajectory
		trajectory_msgs::JointTrajectoryPoint jointPoint;
		jointPoint = trajectory_msg.joint_trajectory.points[i];
		int closestRelative;
		closestRelative = i;
		// add torso position
		jointPoint.positions.push_back(torsoPoints[closestRelative].position.z-0.8);
		fullBodyTraj_msg.joint_trajectory.points.push_back(jointPoint);
		// Base Trajectory
		trajectory_msgs::MultiDOFJointTrajectoryPoint basePoint;
		geometry_msgs::Transform transform;
		transform.translation.x = torsoPoints[closestRelative].position.x;//+ offset.getOrigin().getX()
		transform.translation.y = torsoPoints[closestRelative].position.y;//+ offset.getOrigin().getY()
		transform.translation.z = 0; 
		transform.rotation = torsoPoints[closestRelative].orientation; 
		basePoint.transforms.push_back(transform);
		fullBodyTraj_msg.multi_dof_joint_trajectory.points.push_back(basePoint); 
	}
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
	start_state.multi_dof_joint_state.header.frame_id = "odom_combined";
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
 	// command arm to initial pose
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
    initialArmPose.points[0].time_from_start = ros::Duration(0.01);
    cmd_arm_pub_.publish(initialArmPose);

    // command torso to initial pose
    trajectory_msgs::JointTrajectory initialTorsoPose;
    trajectory_msgs::JointTrajectoryPoint torsoPoint;
    initialTorsoPose.joint_names.push_back("torso_lift_joint");
    torsoPoint.positions.push_back(robotTraj.joint_trajectory.points[0].positions[nrJoints-1]);
    ROS_INFO("TorsoEndPose: %g",robotTraj.joint_trajectory.points[trajectoryLength-2].positions[nrJoints-1]);
    ROS_INFO("TorsoEndPose(1): %g",robotTraj.joint_trajectory.points[trajectoryLength-1].positions[nrJoints-1]);
    initialTorsoPose.points.push_back(torsoPoint);
    initialTorsoPose.points[0].time_from_start = ros::Duration(0.01);
    cmd_torso_pub_.publish(initialTorsoPose);

	// Drive to to start pose
	trajectory_msgs::MultiDOFJointTrajectoryPoint basePoint = robotTraj.multi_dof_joint_trajectory.points[0];
	geometry_msgs::Transform transform = basePoint.transforms[0];
	geometry_msgs::Pose pose;
	tf::Transform goal_transform;
	pose.position.x = transform.translation.x;pose.position.y = transform.translation.y;pose.position.z = transform.translation.z;
	pose.orientation = transform.rotation;

	// Set up stuff for IK
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

	// Drive to start point
	tf::poseMsgToTF(pose,goal_transform);
    double roll_start,pitch_start,yaw_start;
	goal_transform.getBasis().getRPY(roll_start,pitch_start,yaw_start);	
	geometry_msgs::Twist base_cmd;
	ros::Rate rate(50.0);
	bool done = false;
	while (!done && nh_.ok())
	{		
		//get the current transform
		try
		{
		listener_.lookupTransform("odom_combined", "base_footprint",
		                          ros::Time(0), current_transform_);
		}
		catch (tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
		break;
		}		
		// get goal in current frame to calculate desired velocity
		tf::Transform relative_desired_pose = current_transform_.inverse() * goal_transform;
		double roll_current,pitch_current,yaw_current;
		relative_desired_pose.getBasis().getRPY(roll_current,pitch_current,yaw_current);
		base_cmd.angular.z = yaw_current;
		base_cmd.linear.x = relative_desired_pose.getOrigin().getX();
		base_cmd.linear.y = relative_desired_pose.getOrigin().getY();		
		//send the drive command to base
		cmd_vel_pub_.publish(base_cmd);
		rate.sleep();
		double dist_goal = relative_desired_pose.getOrigin().length();
		if(dist_goal < 0.01 && yaw_current < 0.01){
			done = true;
	    	ROS_INFO("dist_goal to Goal: %g",dist_goal);    	
		}
	  		
	}

	ROS_INFO("Waiting");
	ros::Duration(2.0).sleep();
	ROS_INFO("Reached StartPosition");




	// execute the precomputed trajectory:
	double motionDuration = robotTraj.joint_trajectory.header.stamp.toSec();
	double initialTime = ros::Time::now().toSec();
	done = false;
	int currentPoseIdx = 0;
	int Idx = 0;
	trajectory_msgs::JointTrajectory newArmPose;
	newArmPose = initialArmPose;
	newArmPose.points[0].time_from_start = ros::Duration(0.02);
	trajectory_msgs::JointTrajectory newTorsoPose;
	newTorsoPose = initialTorsoPose;
	newTorsoPose.points[0].time_from_start = ros::Duration(0.02);  
	sensor_msgs::JointState stateMsg;
	stateMsg.name = trajectory_msg. joint_trajectory.joint_names;





	//
	while (!done && nh_.ok())
	{
		double currentTime = ros::Time::now().toSec();		
		currentTime = (currentTime- initialTime)/5.0;
		// find closest point to currentTime
		Idx = round(trajectoryLength*currentTime/motionDuration);
		if (Idx > trajectoryLength-1)
			Idx = trajectoryLength-1;
		if (Idx != currentPoseIdx)
		{
			currentPoseIdx = Idx;
			ROS_INFO("currentPoseIdx: %d, trajectoryLength: %d",currentPoseIdx,trajectoryLength-1);
			// set new base position
			basePoint = robotTraj.multi_dof_joint_trajectory.points[currentPoseIdx];
			transform = basePoint.transforms[0];
			pose.position.x = transform.translation.x;
			pose.position.y = transform.translation.y;
			pose.position.z = transform.translation.z;
			pose.orientation = transform.rotation;
			tf::poseMsgToTF(pose,goal_transform);


		    // set new torso configuration
		    trajectory_msgs::JointTrajectoryPoint torsoPoint;
		    torsoPoint.positions.push_back(robotTraj.joint_trajectory.points[currentPoseIdx].positions[nrJoints-1]);
		    newTorsoPose.points[0] = torsoPoint;
		    newTorsoPose.points[0].time_from_start = ros::Duration(0.01);
		    // ROS_INFO("torsoJoint: %g",robotTraj.joint_trajectory.points[currentPoseIdx].positions[nrJoints-1]);
		    // ROS_INFO("currentPoseIdx: %d, trajectoryLength: %d",currentPoseIdx,trajectoryLength-1);
		    cmd_torso_pub_.publish(newTorsoPose);
		}

		//get the current transform
		try
		{
			listener_.lookupTransform("odom_combined", "base_footprint",
		                          ros::Time(0), current_transform_);
		}
		catch (tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
			break;
		}		
		//see how far we've traveled
		tf::Transform relative_desired_pose = current_transform_.inverse() * goal_transform;
		double roll_current,pitch_current,yaw_current;
		relative_desired_pose.getBasis().getRPY(roll_current,pitch_current,yaw_current);
		base_cmd.angular.z = yaw_current;
		base_cmd.linear.x = relative_desired_pose.getOrigin().getX()*5.0;
		base_cmd.linear.y = relative_desired_pose.getOrigin().getY()*5.0;
		cmd_vel_pub_.publish(base_cmd);

		// find ik for arm and command new pose:
		tf::Transform currentGripperTransform;
		tf::poseMsgToTF(waypointsWorld[currentPoseIdx],currentGripperTransform); 
		currentGripperTransform = current_transform_.inverse()*currentGripperTransform;
		geometry_msgs::Pose gripperPoseMsg;
		tf::poseTFToMsg(currentGripperTransform,gripperPoseMsg);
		gripperPoseMsg.position.z += 0.148;


		Eigen::Affine3d state;
		tf::poseMsgToEigen(gripperPoseMsg,state);
		const Eigen::Affine3d &desiredState = state;
		stateMsg.position = trajectory_msg. joint_trajectory.points[currentPoseIdx].positions;
		kinematic_state->setVariableValues(stateMsg);
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
					// joint_names2.push_back(joint_names[counter]);
					counter++;
				}
				// ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
			}
			// newArmPose.joint_names = joint_names2;
			newArmPose.points[0] = armJointPoint;
		    newArmPose.points[0].time_from_start = ros::Duration(0.01);
		    cmd_arm_pub_.publish(newArmPose);
		}	
		else
			ROS_INFO("IK not found");	
		
		if(currentTime > motionDuration){
			done = true;
			double dist_goal = relative_desired_pose.getOrigin().length();
			ROS_INFO("Motion finished");
			if (dist_goal < 0.001 && yaw_current < 0.01)
				ROS_INFO("Reached Goal");
			else
				ROS_INFO("Failed to reached Goal (remaining Distance %g)",dist_goal);
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






void RobotDriver::BlindfollowTrajectory(const moveit_msgs::DisplayTrajectory msg)
{
	ROS_INFO("received msg");
	moveit_msgs::RobotTrajectory robotTraj = msg.trajectory[0];
 	// command arm to initial pose
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
    initialArmPose.points[0].time_from_start = ros::Duration(0.01);
    cmd_arm_pub_.publish(initialArmPose);

    // command torso to initial pose
    trajectory_msgs::JointTrajectory initialTorsoPose;
    trajectory_msgs::JointTrajectoryPoint torsoPoint;
    initialTorsoPose.joint_names.push_back("torso_lift_joint");
    torsoPoint.positions.push_back(robotTraj.joint_trajectory.points[0].positions[nrJoints-1]);
    ROS_INFO("TorsoEndPose: %g",robotTraj.joint_trajectory.points[trajectoryLength-2].positions[nrJoints-1]);
    ROS_INFO("TorsoEndPose(1): %g",robotTraj.joint_trajectory.points[trajectoryLength-1].positions[nrJoints-1]);
    initialTorsoPose.points.push_back(torsoPoint);
    initialTorsoPose.points[0].time_from_start = ros::Duration(0.01);
    cmd_torso_pub_.publish(initialTorsoPose);

	// Drive to to start pose
	trajectory_msgs::MultiDOFJointTrajectoryPoint basePoint = robotTraj.multi_dof_joint_trajectory.points[0];
	geometry_msgs::Transform transform = basePoint.transforms[0];
	geometry_msgs::Pose pose;
	tf::Transform goal_transform;
	pose.position.x = transform.translation.x;pose.position.y = transform.translation.y;pose.position.z = transform.translation.z;
	pose.orientation = transform.rotation;

	// Drive to start point
	tf::poseMsgToTF(pose,goal_transform);
    double roll_start,pitch_start,yaw_start;
	goal_transform.getBasis().getRPY(roll_start,pitch_start,yaw_start);	
	geometry_msgs::Twist base_cmd;
	ros::Rate rate(50.0);
	bool done = false;
	while (!done && nh_.ok())
	{		
		//get the current transform
		try
		{
		listener_.lookupTransform("odom_combined", "base_footprint",
		                          ros::Time(0), current_transform_);
		}
		catch (tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
		break;
		}		
		// get goal in current frame to calculate desired velocity
		tf::Transform relative_desired_pose = current_transform_.inverse() * goal_transform;
		double roll_current,pitch_current,yaw_current;
		relative_desired_pose.getBasis().getRPY(roll_current,pitch_current,yaw_current);
		base_cmd.angular.z = yaw_current;
		base_cmd.linear.x = relative_desired_pose.getOrigin().getX();
		base_cmd.linear.y = relative_desired_pose.getOrigin().getY();		
		//send the drive command to base
		cmd_vel_pub_.publish(base_cmd);
		rate.sleep();
		double dist_goal = relative_desired_pose.getOrigin().length();
		if(dist_goal < 0.01 && yaw_current < 0.01){
			done = true;
	    	ROS_INFO("dist_goal to Goal: %g",dist_goal);    	
		}
	  		
	}

	ROS_INFO("Waiting");
	ros::Duration(2.0).sleep();
	ROS_INFO("Reached StartPosition");




	// execute the precomputed trajectory:
	double motionDuration = robotTraj.joint_trajectory.header.stamp.toSec();
	double initialTime = ros::Time::now().toSec();
	done = false;
	int currentPoseIdx = 0;
	int Idx = 0;
	trajectory_msgs::JointTrajectory newArmPose;
	newArmPose = initialArmPose;
	newArmPose.points[0].time_from_start = ros::Duration(0.02);
	trajectory_msgs::JointTrajectory newTorsoPose;
	newTorsoPose = initialTorsoPose;
	newTorsoPose.points[0].time_from_start = ros::Duration(0.02);  
	while (!done && nh_.ok())
	{
		double currentTime = ros::Time::now().toSec();
		currentTime = (currentTime- initialTime)/2.0;
		// find closest point to currentTime
		Idx = round(trajectoryLength*currentTime/motionDuration);
		if (Idx > trajectoryLength-1)
			Idx = trajectoryLength-1;
		if (Idx != currentPoseIdx)
		{
			currentPoseIdx = Idx;
			// set new base position
			basePoint = robotTraj.multi_dof_joint_trajectory.points[currentPoseIdx];
			transform = basePoint.transforms[0];
			pose.position.x = transform.translation.x;
			pose.position.y = transform.translation.y;
			pose.position.z = transform.translation.z;
			pose.orientation = transform.rotation;
			tf::poseMsgToTF(pose,goal_transform);

			// set new arm configuration
		    newArmPose.points[0] = robotTraj.joint_trajectory.points[currentPoseIdx]; 
		    trajectory_msgs::JointTrajectoryPoint armJointPoint;
		    for (int i = 0; i< nrJoints-1; i++)
		    {
		    	armJointPoint.positions.push_back(robotTraj.joint_trajectory.points[currentPoseIdx].positions[i]);
		    }
		    newArmPose.points[0] = armJointPoint;
		    newArmPose.points[0].time_from_start = ros::Duration(0.01);
		    cmd_arm_pub_.publish(newArmPose);

		    // set new torso configuration
		    trajectory_msgs::JointTrajectoryPoint torsoPoint;
		    torsoPoint.positions.push_back(robotTraj.joint_trajectory.points[currentPoseIdx].positions[nrJoints-1]);
		    newTorsoPose.points[0] = torsoPoint;
		    newTorsoPose.points[0].time_from_start = ros::Duration(0.01);
		    // ROS_INFO("torsoJoint: %g",robotTraj.joint_trajectory.points[currentPoseIdx].positions[nrJoints-1]);
		    // ROS_INFO("currentPoseIdx: %d, trajectoryLength: %d",currentPoseIdx,trajectoryLength-1);
		    cmd_torso_pub_.publish(newTorsoPose);
		}

		//get the current transform
		try
		{
		listener_.lookupTransform("odom_combined", "base_footprint",
		                          ros::Time(0), current_transform_);
		}
		catch (tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
		break;
		}		
		//see how far we've traveled
		tf::Transform relative_desired_pose = current_transform_.inverse() * goal_transform;
		//the command will be to go forward at 0.25 m/s
		double roll_current,pitch_current,yaw_current;
		relative_desired_pose.getBasis().getRPY(roll_current,pitch_current,yaw_current);
		base_cmd.angular.z = yaw_current;
		base_cmd.linear.x = relative_desired_pose.getOrigin().getX()*5.0;
		base_cmd.linear.y = relative_desired_pose.getOrigin().getY()*5.0;
		cmd_vel_pub_.publish(base_cmd);
		rate.sleep();
		
		
		
		if(currentTime > motionDuration){
			done = true;
			double dist_goal = relative_desired_pose.getOrigin().length();
			ROS_INFO("Motion finished");
			if (dist_goal < 0.001 && yaw_current < 0.01)
				ROS_INFO("Reached Goal");
			else
				ROS_INFO("Failed to reached Goal (remaining Distance %g)",dist_goal);
		} 

	  	
	}
	
}



void RobotDriver::moveitIKfollowTrajectory(const geometry_msgs::PoseArray msg)
{
	try
	{
		listener_.lookupTransform("odom_combined", "torso_lift_link",
	                          ros::Time(0), transformTorso_);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
	}
	unsigned int traj_length = msg.poses.size();
	ROS_INFO("Trajectory msg with length %u received",traj_length);
	std::vector<geometry_msgs::Pose> waypoints;
	std::vector<geometry_msgs::Pose> torsoPoints;
	tf::Transform offset;
	offset.setIdentity();
	tf::Vector3 translationOff(0.05,0.0,0.0);
	offset.setOrigin(translationOff);
	tf::Transform gripperOffset;
	gripperOffset.setIdentity();
	tf::Vector3 gripperTranslationOff(-0.18,0.0,0.0);
	gripperOffset.setOrigin(gripperTranslationOff);
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
			// tf::Transform torsoTransform;
			// tf::poseMsgToTF(msg.poses[j+1],torsoTransform);
			// torsoTransform = transformTorso_.inverse()*torsoTransform;  
			// gripperTransform = torsoTransform.inverse()*gripperTransform;
			geometry_msgs::Pose poseMsg;
			tf::poseTFToMsg(gripperTransform,poseMsg);
			waypoints.push_back(poseMsg);
		}
	}
	ROS_INFO_STREAM("Startpoint after Transform:");
	// stop any running motion before staring the new
	group_->stop(); 
	group_->setPlanningTime(30);
	//compute joint trajectory
	moveit_msgs::RobotTrajectory trajectory_msg;
	moveit_msgs::RobotTrajectory fullBodyTraj_msg;
	// double fraction = group_->computeCartesianPath(waypoints,1.9, 15.5, trajectory_msg,true); // jump_threshold  // eef_step
	double fraction = group_->computeCartesianPath(waypoints,20.0, 0.0, trajectory_msg,false); 
	ROS_INFO("Achieved fraction %g of total path",fraction);
	// add torso poses to Trajectory
	fullBodyTraj_msg.joint_trajectory.joint_names = trajectory_msg.joint_trajectory.joint_names;
	fullBodyTraj_msg.joint_trajectory.joint_names.push_back("torso_lift_joint");
	fullBodyTraj_msg.joint_trajectory.header = msg.header;
	fullBodyTraj_msg.multi_dof_joint_trajectory.header.frame_id = "odom_combined";
	fullBodyTraj_msg.multi_dof_joint_trajectory.joint_names.push_back("world_joint");
	traj_length = trajectory_msg.joint_trajectory.points.size()-1;

	for (int i = 0; i < traj_length;i++)
	{
		// Torso_lift_link trajectory
		trajectory_msgs::JointTrajectoryPoint jointPoint;
		jointPoint = trajectory_msg.joint_trajectory.points[i];
		int closestRelative;
		closestRelative = i;
		// add torso position
		jointPoint.positions.push_back(torsoPoints[closestRelative].position.z-0.8);
		fullBodyTraj_msg.joint_trajectory.points.push_back(jointPoint);
		// Base Trajectory
		trajectory_msgs::MultiDOFJointTrajectoryPoint basePoint;
		geometry_msgs::Transform transform;
		transform.translation.x = torsoPoints[closestRelative].position.x;//+ offset.getOrigin().getX()
		transform.translation.y = torsoPoints[closestRelative].position.y;//+ offset.getOrigin().getY()
		transform.translation.z = 0; 
		transform.rotation = torsoPoints[closestRelative].orientation; 
		basePoint.transforms.push_back(transform);
		fullBodyTraj_msg.multi_dof_joint_trajectory.points.push_back(basePoint); 
	}
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
	start_state.multi_dof_joint_state.header.frame_id = "odom_combined";
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
 	// command arm to initial pose
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
    initialArmPose.points[0].time_from_start = ros::Duration(0.01);
    cmd_arm_pub_.publish(initialArmPose);

    // command torso to initial pose
    trajectory_msgs::JointTrajectory initialTorsoPose;
    trajectory_msgs::JointTrajectoryPoint torsoPoint;
    initialTorsoPose.joint_names.push_back("torso_lift_joint");
    torsoPoint.positions.push_back(robotTraj.joint_trajectory.points[0].positions[nrJoints-1]);
    ROS_INFO("TorsoEndPose: %g",robotTraj.joint_trajectory.points[trajectoryLength-2].positions[nrJoints-1]);
    ROS_INFO("TorsoEndPose(1): %g",robotTraj.joint_trajectory.points[trajectoryLength-1].positions[nrJoints-1]);
    initialTorsoPose.points.push_back(torsoPoint);
    initialTorsoPose.points[0].time_from_start = ros::Duration(0.01);
    cmd_torso_pub_.publish(initialTorsoPose);

	// Drive to to start pose
	trajectory_msgs::MultiDOFJointTrajectoryPoint basePoint = robotTraj.multi_dof_joint_trajectory.points[0];
	geometry_msgs::Transform transform = basePoint.transforms[0];
	geometry_msgs::Pose pose;
	tf::Transform goal_transform;
	pose.position.x = transform.translation.x;pose.position.y = transform.translation.y;pose.position.z = transform.translation.z;
	pose.orientation = transform.rotation;

	// Drive to start point
	tf::poseMsgToTF(pose,goal_transform);
    double roll_start,pitch_start,yaw_start;
	goal_transform.getBasis().getRPY(roll_start,pitch_start,yaw_start);	
	geometry_msgs::Twist base_cmd;
	ros::Rate rate(10.0);
	bool done = false;
	while (!done && nh_.ok())
	{		
		//get the current transform
		try
		{
		listener_.lookupTransform("odom_combined", "base_footprint",
		                          ros::Time(0), current_transform_);
		}
		catch (tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
		break;
		}		
		// get goal in current frame to calculate desired velocity
		tf::Transform relative_desired_pose = current_transform_.inverse() * goal_transform;
		double roll_current,pitch_current,yaw_current;
		relative_desired_pose.getBasis().getRPY(roll_current,pitch_current,yaw_current);
		base_cmd.angular.z = yaw_current;
		base_cmd.linear.x = relative_desired_pose.getOrigin().getX();
		base_cmd.linear.y = relative_desired_pose.getOrigin().getY();		
		//send the drive command to base
		cmd_vel_pub_.publish(base_cmd);
		rate.sleep();
		double dist_goal = relative_desired_pose.getOrigin().length();
		if(dist_goal < 0.01 && yaw_current < 0.01){
			done = true;
	    	ROS_INFO("dist_goal to Goal: %g",dist_goal);    	
		}
	  		
	}

	ROS_INFO("Waiting");
	ros::Duration(2.0).sleep();
	ROS_INFO("Reached StartPosition");




	// execute the precomputed trajectory:
	double motionDuration = robotTraj.joint_trajectory.header.stamp.toSec();
	double initialTime = ros::Time::now().toSec();
	done = false;
	int currentPoseIdx = 0;
	int Idx = 0;
	trajectory_msgs::JointTrajectory newArmPose;
	newArmPose = initialArmPose;
	newArmPose.points[0].time_from_start = ros::Duration(0.02);
	trajectory_msgs::JointTrajectory newTorsoPose;
	newTorsoPose = initialTorsoPose;
	newTorsoPose.points[0].time_from_start = ros::Duration(0.02);  
	while (!done && nh_.ok())
	{
		double currentTime = ros::Time::now().toSec();
		ROS_INFO("currentPoseIdx: %d, trajectoryLength: %d",currentPoseIdx,trajectoryLength-1);
		currentTime = (currentTime- initialTime)/10.0;
		// find closest point to currentTime
		Idx = round(trajectoryLength*currentTime/motionDuration);
		if (Idx > trajectoryLength-1)
			Idx = trajectoryLength-1;
		if (Idx != currentPoseIdx)
		{
			currentPoseIdx = Idx;
			// set new base position
			basePoint = robotTraj.multi_dof_joint_trajectory.points[currentPoseIdx];
			transform = basePoint.transforms[0];
			pose.position.x = transform.translation.x;
			pose.position.y = transform.translation.y;
			pose.position.z = transform.translation.z;
			pose.orientation = transform.rotation;
			tf::poseMsgToTF(pose,goal_transform);

			// set new arm configuration
		    newArmPose.points[0] = robotTraj.joint_trajectory.points[currentPoseIdx]; 
		    trajectory_msgs::JointTrajectoryPoint armJointPoint;
		    for (int i = 0; i< nrJoints-1; i++)
		    {
		    	armJointPoint.positions.push_back(robotTraj.joint_trajectory.points[currentPoseIdx].positions[i]);
		    }
		    newArmPose.points[0] = armJointPoint;
		    newArmPose.points[0].time_from_start = ros::Duration(0.01);
		    // cmd_arm_pub_.publish(newArmPose);

		    // set new torso configuration
		    trajectory_msgs::JointTrajectoryPoint torsoPoint;
		    torsoPoint.positions.push_back(robotTraj.joint_trajectory.points[currentPoseIdx].positions[nrJoints-1]);
		    newTorsoPose.points[0] = torsoPoint;
		    newTorsoPose.points[0].time_from_start = ros::Duration(0.01);
		    // ROS_INFO("torsoJoint: %g",robotTraj.joint_trajectory.points[currentPoseIdx].positions[nrJoints-1]);
		    // ROS_INFO("currentPoseIdx: %d, trajectoryLength: %d",currentPoseIdx,trajectoryLength-1);
		    cmd_torso_pub_.publish(newTorsoPose);
		}

		//get the current transform
		try
		{
		listener_.lookupTransform("odom_combined", "base_footprint",
		                          ros::Time(0), current_transform_);
		}
		catch (tf::TransformException ex)
		{
		ROS_ERROR("%s",ex.what());
		break;
		}		
		//see how far we've traveled
		tf::Transform relative_desired_pose = current_transform_.inverse() * goal_transform;
		double roll_current,pitch_current,yaw_current;
		relative_desired_pose.getBasis().getRPY(roll_current,pitch_current,yaw_current);
		base_cmd.angular.z = yaw_current;
		base_cmd.linear.x = relative_desired_pose.getOrigin().getX()*5.0;
		base_cmd.linear.y = relative_desired_pose.getOrigin().getY()*5.0;
		cmd_vel_pub_.publish(base_cmd);

		// find ik for arm and command new pose:
		group_->setPoseTarget(waypoints[currentPoseIdx]);
		group_->stop();
		group_->asyncMove();		
		
		
		if(currentTime > motionDuration){
			done = true;
			double dist_goal = relative_desired_pose.getOrigin().length();
			ROS_INFO("Motion finished");
			if (dist_goal < 0.001 && yaw_current < 0.01)
				ROS_INFO("Reached Goal");
			else
				ROS_INFO("Failed to reached Goal (remaining Distance %g)",dist_goal);
		} 
		// try to keep the loop at sconstant rate:
	  	rate.sleep();
	}
	
}