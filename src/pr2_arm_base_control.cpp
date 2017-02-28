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
	DesPose_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/Controller/DesiredPose", 1, true);
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
	// pr2_arm_base_control::array1d d;
	// d.a.push_back(4.5);
	// d.a.push_back(4.3);
	// pr2_arm_base_control::GMM gmm_msg;
	// geometry_msgs::Pose test;
	// gmm_msg.mu.gripperPoses.push_back(test);
	// gmm_msg.mu.basePoses.push_back(test);
	// Collision constraint function GroupStateValidityCallbackFn(),
	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));
	robot_state::GroupStateValidityCallbackFn constraint_callback_fn = boost::bind(&validityFun::validityCallbackFn, planning_scene, kinematic_state,_2,_3);
	// variables for execute precomputed trajectory:
	double motionDuration = msg.header.stamp.toSec();
	double initialTime = ros::Time::now().toSec();    	
	int currentPoseIdx = 0;
	int currentHandIdx = 0;
	int Idx = 0;
	geometry_msgs::Twist base_cmd;
	tf::Transform goal_transform;
	geometry_msgs::Transform transform;
    	geometry_msgs::Pose currentDesiredBasepose;

	// Loop for executing precomputed trajectory
	// desired rate for the control loop
	ros::Rate rate(50.0);
	bool done = false;
	double slowFactor = 15.0;
	double lastTime = ros::Time::now().toSec()- initialTime;
	double runningTime = 0;
	double lastVel = 0;
	double lastAngVel = 0;
	bool hand_good = true;
	while (!done && nh_.ok())
	{
		double currentTime = ros::Time::now().toSec();	

		currentTime = (currentTime- initialTime);// /slowFactor;

		runningTime = runningTime + (currentTime-lastTime)/slowFactor;
		lastTime = currentTime;
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
		Idx = round(trajectoryLength*runningTime/motionDuration);
		if (Idx > trajectoryLength-1)
			Idx = trajectoryLength-1;
		if (Idx > currentPoseIdx)
		{			
			currentPoseIdx++;// = Idx;
			// ROS_INFO("currentPoseIdx: %d, trajectoryLength: %d, slowFactor: %g",currentPoseIdx,trajectoryLength-1,slowFactor);
			currentDesiredBasepose = torsoPoints[currentPoseIdx];
			tf::poseMsgToTF(currentDesiredBasepose,goal_transform);

	    	// set and command new torso configuration
	    	trajectory_msgs::JointTrajectoryPoint newTorsoPoint;
	    	// newTorsoPoint.positions.push_back(fullBodyTraj_msg.joint_trajectory.points[currentPoseIdx].positions[nrJoints-1]);
	    	if (torsoPoints[currentPoseIdx].position.z-0.747 > 0.3135)
	    		newTorsoPoint.positions.push_back(0.3135);
	    	else
	    		newTorsoPoint.positions.push_back(torsoPoints[currentPoseIdx].position.z-0.7467);
	    	newTorsoPose.points[0] = newTorsoPoint;
	    	newTorsoPose.points[0].time_from_start = ros::Duration(0.1);
	    	cmd_torso_pub_.publish(newTorsoPose);

	    	// Publsh desired Pose as marker for rviz
	    	visualization_msgs::Marker mark;
	        mark.header.frame_id = "/map";
	        mark.ns = "Base";
	        mark.id = 1;
		double marker_size = 0.08;
	        mark.type = visualization_msgs::Marker::CUBE;
		    mark.action = visualization_msgs::Marker::ADD;
		    mark.pose.orientation.w = 1.0;
		    mark.scale.x = marker_size;
		    mark.scale.y = marker_size;
		    mark.scale.z = marker_size;
		    mark.color.g = 1.0;
		    mark.color.a = 1.;


		    tf::Transform torsoPose;
		    tf::poseMsgToTF(currentDesiredBasepose,torsoPose); 
		    tf::Transform offsetT;
			offsetT.setIdentity();
			tf::Vector3 translationOffT(-0.05,0.0,0.0);     
			offsetT.setOrigin(translationOffT);
			torsoPose = torsoPose*offsetT;
			geometry_msgs::Pose poseMsgT;
			tf::poseTFToMsg(torsoPose,poseMsgT);
		    mark.pose = poseMsgT;
		    // mark.pose = currentDesiredBasepose;
		    // mark.pose.position.x = torsoPoints[currentPoseIdx].position.x;
		    // mark.pose.position.y = torsoPoints[currentPoseIdx].position.y;
		    // mark.pose.position.z = torsoPoints[currentPoseIdx].position.z; 

		    visualization_msgs::Marker mark_gripper;
	        mark_gripper.header.frame_id = "/map";
	        mark_gripper.ns = "Gripper";
	        mark_gripper.id = 2;
	        mark_gripper.type = visualization_msgs::Marker::CUBE;
		    mark_gripper.action = visualization_msgs::Marker::ADD;
		    mark_gripper.pose.orientation.w = 1.0;
		    mark_gripper.scale.x = marker_size;
		    mark_gripper.scale.y = marker_size;
		    mark_gripper.scale.z = marker_size;
		    mark_gripper.color.b = 1.0;
		    mark_gripper.color.r = 1.0;
		    mark_gripper.color.a = 1.;
		    tf::Transform gripperPose;
		    tf::poseMsgToTF(waypointsWorld[currentPoseIdx],gripperPose); 
		    tf::Transform offsetGr;
			offsetGr.setIdentity();
			tf::Vector3 translationOffG(0.18,0.0,0.0);
			offsetGr.setOrigin(translationOffG);     
			gripperPose = gripperPose*offsetGr;
			geometry_msgs::Pose poseMsg;
			tf::poseTFToMsg(gripperPose,poseMsg);
		    mark_gripper.pose = poseMsg;

		    visualization_msgs::MarkerArray ma;
		    ma.markers.push_back(mark);
		    ma.markers.push_back(mark_gripper);
		    DesPose_pub_.publish(ma);
		}

				
		// calculate base velocity based on current and desired base poses
		tf::Transform relative_desired_pose = current_transform.inverse() * goal_transform;
		double roll_current,pitch_current,yaw_current;
		relative_desired_pose.getBasis().getRPY(roll_current,pitch_current,yaw_current);
		base_cmd.angular.z = yaw_current*1.0;
		base_cmd.linear.x = relative_desired_pose.getOrigin().getX()*1.0;
		base_cmd.linear.y = relative_desired_pose.getOrigin().getY()*1.0;
		//if(abs(base_cmd.linear.x) > 1.5)
			//base_cmd.linear.x = 1.5;
		//if(abs(base_cmd.linear.y) > 1.5)
                        //base_cmd.linear.y = 1.5;
		// send out command for base velocity
		if(sqrt(base_cmd.linear.x*base_cmd.linear.x+base_cmd.linear.y*base_cmd.linear.y)<0.015 && lastVel<0.01)   
		{
			base_cmd.linear.x = 0.0;
                        base_cmd.linear.y = 0.0;
			if(abs(base_cmd.angular.z)>0.02) // only react if angle is more then 1 degree off 
			{
				cmd_vel_pub_.publish(base_cmd);
			}
			else
				base_cmd.angular.z = 0.0;
		}
		else
		{
			if(abs(base_cmd.angular.z)<0.02) // only react if angle is more then 1 degree off
                        {
				base_cmd.angular.z = 0.0;
			}
			cmd_vel_pub_.publish(base_cmd);
		}
		double dist2desired = sqrt(relative_desired_pose.getOrigin().getX()*relative_desired_pose.getOrigin().getX() + 
								   relative_desired_pose.getOrigin().getY()*relative_desired_pose.getOrigin().getY());
		if (dist2desired > 0.02)
			slowFactor += 0.1;
		else if (dist2desired < 0.02)
			slowFactor -= 0.1;
		if (slowFactor > 40)
			slowFactor = 40;
		else if (slowFactor < 2)
			slowFactor = 2;
		// ROS_INFO("base_cmd (%g,%g)",sqrt(base_cmd.linear.x*base_cmd.linear.x+base_cmd.linear.y*base_cmd.linear.y),base_cmd.angular.z);
		lastVel = sqrt(base_cmd.linear.x*base_cmd.linear.x+base_cmd.linear.y*base_cmd.linear.y);
		lastAngVel = base_cmd.angular.z;
		// find ik for arm and command new pose:
		tf::Transform currentGripperTransform;
		tf::poseMsgToTF(waypointsWorld[currentPoseIdx],currentGripperTransform); 
		// Transform to /base_footprint frame because planning scene operates here 
		currentGripperTransform = current_transform.inverse()*currentGripperTransform;
		geometry_msgs::Pose gripperPoseMsg;
		tf::poseTFToMsg(currentGripperTransform,gripperPoseMsg);
		// gripperPoseMsg.position.z -= 0.17; //  newTorsoPose.points[0].positions[0] - 0.17
		gripperPoseMsg.position.z -= newTorsoPose.points[0].positions[0] - 0.17;//  offset don"t know why this is needed??????? 
		Eigen::Affine3d state;
		tf::poseMsgToEigen(gripperPoseMsg,state);
		const Eigen::Affine3d &desiredState = state;

    

		// Get IK solution from desired cartesian state:
		// bool found_ik = kinematic_state->setFromIK(joint_model_group, desiredState, 5, 0.1,constraint_callback_fn);
		bool found_ik = false;
		if (hand_good)
		{

			tf::poseMsgToTF(waypointsWorld[currentPoseIdx],currentGripperTransform);
                	// Transform to /base_footprint frame because planning scene operates here 
                	currentGripperTransform = current_transform.inverse()*currentGripperTransform;
                	geometry_msgs::Pose gripperPoseMsg;
                	tf::poseTFToMsg(currentGripperTransform,gripperPoseMsg);
                	// gripperPoseMsg.position.z -= 0.17; //  newTorsoPose.points[0].positions[0] - 0.17
                	gripperPoseMsg.position.z -= newTorsoPose.points[0].positions[0] - 0.17;//  offset don"t know why this is needed??????? 
                	Eigen::Affine3d state;
        	        tf::poseMsgToEigen(gripperPoseMsg,state);
	                const Eigen::Affine3d &desiredState = state;


			found_ik = kinematic_state->setFromIK(joint_model_group, desiredState, 5, 0.1);			
		}
		// If found solution create and publish arm pose command
		if (hand_good && found_ik)
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
			currentHandIdx = currentPoseIdx;
		}	
		else if(hand_good && !found_ik)//////////////////////////////////////////////////////////////////////////////////////////////////////////////
		{

			std::vector<int> ik_try;
			ik_try.push_back(-1);
			ik_try.push_back(-10);
			ik_try.push_back(10);
			ik_try.push_back(-30);
			ik_try.push_back(30);
			ik_try.push_back(-60);
			ik_try.push_back(60);
			ik_try.push_back(-100);
			ik_try.push_back(100);
			ik_try.push_back(-150);
			int hand_off = 0;
			for (int i =1;i< ik_try.size(); i++)
			{
				tf::poseMsgToTF(waypointsWorld[currentPoseIdx+ik_try[i]],currentGripperTransform);
                        	// Transform to /base_footprint frame because planning scene operates here 
                        	currentGripperTransform = current_transform.inverse()*currentGripperTransform;
                        	geometry_msgs::Pose gripperPoseMsg;
                       	 	tf::poseTFToMsg(currentGripperTransform,gripperPoseMsg);
                        	// gripperPoseMsg.position.z -= 0.17; //  newTorsoPose.points[0].positions[0] - 0.17
                        	gripperPoseMsg.position.z -= newTorsoPose.points[0].positions[0] - 0.17;//  offset don"t know why this is needed??????? 
                        	Eigen::Affine3d state;
                	        tf::poseMsgToEigen(gripperPoseMsg,state);
        	                const Eigen::Affine3d &desiredState = state;

	
	                        found_ik = kinematic_state->setFromIK(joint_model_group, desiredState, 5, 0.1);
				if (found_ik)
				{
					hand_off = ik_try[i];
					break;
				}
			}
			
			hand_good = false;
			ROS_INFO("IK not found at currentPoseIdx: %d", currentPoseIdx);
			if(found_ik)
			{
				currentHandIdx = currentPoseIdx + hand_off;
				ROS_INFO("Found IK for hand at Idx: %d instead", currentHandIdx); 
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
			{
				currentHandIdx = currentPoseIdx;
				hand_good = true;
				ROS_INFO("No IK found at all!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			}
			//ROS_INFO("base_cmd (%g,%g)",sqrt(base_cmd.linear.x*base_cmd.linear.x+base_cmd.linear.y*base_cmd.linear.y),base_cmd.angular.z);
		}
		else //if !hand_good //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		{
			int count_diff = currentPoseIdx - currentHandIdx;
			std::vector<int> ik_try;
			if (abs(count_diff) < 20)
			{	
				if (count_diff < 0)
				{
					ik_try.push_back(count_diff);
                                        ik_try.push_back(-1);
                                        ik_try.push_back(0);
                                        ik_try.push_back(1);
                                        ik_try.push_back(10);
                                        ik_try.push_back(50);
				}
				else
				{
					ik_try.push_back(count_diff);
					ik_try.push_back(1);
	                        	ik_try.push_back(10);
	                        	ik_try.push_back(0);
        	                	ik_try.push_back(-1);
                	        	ik_try.push_back(-10);
				}
			}
			else
			{
				if(count_diff <0)
				{
					ik_try.push_back(-10);
                                        ik_try.push_back(-5);
                                        ik_try.push_back(-1);
                                        ik_try.push_back(0);
                                        ik_try.push_back(1);
                                        ik_try.push_back(10);
				}
				else
				{
					ik_try.push_back(10);
                                        ik_try.push_back(5);
                                        ik_try.push_back(1);
                                        ik_try.push_back(0);
                                        ik_try.push_back(-1);
                                        ik_try.push_back(-5);
                                        ik_try.push_back(-10);
                                        ik_try.push_back(-20);
				}
			}	
                        int hand_off = 0;
                        for (int i =1;i< ik_try.size(); i++)
                        {
                               	tf::poseMsgToTF(waypointsWorld[currentHandIdx+ik_try[i]],currentGripperTransform);
                               	// Transform to /base_footprint frame because planning scene operates here 
                               	currentGripperTransform = current_transform.inverse()*currentGripperTransform;
                               	geometry_msgs::Pose gripperPoseMsg;
                               	tf::poseTFToMsg(currentGripperTransform,gripperPoseMsg);
                               	// gripperPoseMsg.position.z -= 0.17; //  newTorsoPose.points[0].positions[0] - 0.17
                               	gripperPoseMsg.position.z -= newTorsoPose.points[0].positions[0] - 0.17;//  offset don"t know why this is needed??????? 
                               	Eigen::Affine3d state;
                               	tf::poseMsgToEigen(gripperPoseMsg,state);
                               	const Eigen::Affine3d &desiredState = state;

                               	found_ik = kinematic_state->setFromIK(joint_model_group, desiredState, 5, 0.1);
                               	if (found_ik)
                               	{	
                              		hand_off = ik_try[i];
	                                break;
                	        }
	      	        }


			if(found_ik)
                        {
                                currentHandIdx = currentPoseIdx + hand_off-count_diff;
                                ROS_INFO("Hand moving with Idx offset of: %d ", hand_off-count_diff);
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
                        {
                                currentHandIdx = currentPoseIdx;
                                ROS_INFO("No IK found at all!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                        }

	
			if (currentHandIdx == currentPoseIdx)
				hand_good = true;

		}













		// check if reached end of motion
		if(currentPoseIdx > trajectoryLength-5){
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
