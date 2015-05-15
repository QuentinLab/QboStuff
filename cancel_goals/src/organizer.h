#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Organizer
{
private:
	// ROS ELEMENTS

	ros::NodeHandle private_nh_;
	ros::Subscriber face_found_sub_;

	// Callbacks

	void faceFoundCallback(const std_msgs::UInt8 &face_found);
	
	

	// Parameters

	MoveBaseClient client_;
	// Functions

	void onInit();

public:
	Organizer(std::string name, bool istrue);
	virtual  ~Organizer();
};
