#ifndef _QBO_MULTIMODAL_ORGANIZER_H
#define _QBO_MULTIMODAL_ORGANIZER_H

//Classic includes
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h> // From detection parts
#include <geometry_msgs/PoseStamped.h> // In case of face detection
#include <geometry_msgs/Pose.h> 
#include <move_base_msgs/MoveBaseAction.h> // For sendging goals to the robot
#include <actionlib/client/simple_action_client.h> // For creating the action client
#include <actionlib_msgs/GoalID.h> // Enabling to keep track of a goal

//Include the transform library

#include <tf/tf.h>
#include <tf/transform_listener.h>

//Including stuff for clients
#include <voronoi/MakeNavPlan.h> // Request for a path to goal
#include <voronoi/FindClosestInPoseArray.h> //Find Closest point
#include <voronoi/Feasability.h> //Client for feasability
#include <qbo_soundstream/soundAcquisition.h> //For listening a number of second

//Definitions and namespaces
#define PI 3.14159265
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

class QboMultiOrganizer
{
private:
	// ROS elements
	ros::NodeHandle private_nh_;
	ros::Subscriber face_position_sub_;
	geometry_msgs::PoseStamped face_position_;
	geometry_msgs::PoseArray semantic_places_; //List of semantic position
	geometry_msgs::PoseArray audio_places_; // List of audio places

	// Other parameters
	MoveBaseClient client_; // Client for actions
	ros::ServiceClient plan_client_; //Client for making plans
	ros::ServiceClient listen_client_; // Client for listening
	ros::ServiceClient find_closest_client_; //Client to chose next destination
	ros::ServiceClient feasability_client_; //Client to check if all audio position are available
	voronoi::MakeNavPlan plan_maker_; //Request for plan
	voronoi::FindClosestInPoseArray closest_asker_; // Request for closest pose
	voronoi::Feasability feasability_checker_;
	qbo_soundstream::soundAcquisition sound_asker_ ;//Request for sound delays 


	int robot_mode_; // Mode : 1 = Patrol; 2 = Audio; 3 = Video
	geometry_msgs::PoseStamped next_move_;
	geometry_msgs::PoseStamped current_goal_;
	move_base_msgs::MoveBaseGoal goal_;

	double previous_check_; //To compute Time between two successive audio checkings
	double current_check_; //To compute Time between two successive audio checkings
	//unsigned char* semantic_visited_;
	std::vector<unsigned char> semantic_visited_;
	std::vector<unsigned char> audio_visited_;
	std::vector<unsigned char> face_visited_;

	int count_faces_; //count callbacks where we see a face
	int count_callbacks_; //count callbacks


	//Functions
	void OnInit();
	void faceCallback(const geometry_msgs::PoseStamped& face_pos);
	void semanticMap();
	geometry_msgs::PoseStamped computeNextPosition(geometry_msgs::Pose realPose, geometry_msgs::Pose robotPose);
	bool listening_routine(std::vector<int> delays); 
	

public:
	// Constructor & Destructor
	QboMultiOrganizer(); // Wrong constructor, the code will break if it is used
	QboMultiOrganizer(std::string name, bool istrue); // Right constructor
	virtual ~QboMultiOrganizer();























};

#endif
