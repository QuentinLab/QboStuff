#ifndef _QBO_MULTIMODAL_ORGANIZER_H
#define _QBO_MULTIMODAL_ORGANIZER_H

//Classic includes
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h> // From detection parts
#include <geometry_msgs/PoseStamped.h> // In case of face detection
#include <geometry_msgs/Pose.h> 
#include <move_base_msgs/MoveBaseAction.h> // For sendging goals to the robot
#include <actionlib/client/simple_action_client.h> // For creating the action client
#include <actionlib_msgs/GoalID.h> // Enabling to keep track of a goal

//Including stuff to make plans
#include <voronoi/MakeNavPlan.h> // Request for a path to goal
#include <voronoi/FindClosestInPoseArray.h> //Find Closest point
#include <qbo_soundstream/soundAcquisition.h> //For listening a number of second

//Definitions and namespaces
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

class QboMultiOrganizer
{
private:
	// ROS elements
	ros::NodeHandle private_nh_;
	ros::Subscriber face_position_sub_;
	geometry_msgs::PoseStamped face_position_;
	geometry_msgs::PoseArray semantic_places_;

	// Other parameters
	MoveBaseClient client_; // Client for actions
	ros::ServiceClient plan_client_; //Client for making plans
	ros::ServiceClient listen_client_; // Client for listening
	ros::ServiceClient find_closest_client_; //Client to chose next destination
	voronoi::MakeNavPlan plan_maker_; //Request for plan
	voronoi::FindClosestInPoseArray closest_asker_; // Request for closest pose
	qbo_soundstream::soundAcquisition sound_asker_ ;//Request for sound delays 


	int robot_mode_; // Mode : 1 = Patrol; 2 = Audio; 3 = Video
	geometry_msgs::PoseStamped next_move_;
	move_base_msgs::MoveBaseGoal goal_;
	//unsigned char* semantic_visited_;
	std::vector<unsigned char> semantic_visited_;


	//Functions
	void OnInit();
	void faceCallback(const geometry_msgs::PoseStamped& face_pos);
	void semanticMap();

	

public:
	// Constructor & Destructor
	QboMultiOrganizer(); // Wrong constructor, the code will break if it is used
	QboMultiOrganizer(std::string name, bool istrue); // Right constructor
	virtual ~QboMultiOrganizer();























};

#endif
