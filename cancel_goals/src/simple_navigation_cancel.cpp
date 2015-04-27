#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_navigation_cancel");

	MoveBaseClient client("move_base",true);

	while (!client.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting...");
	}

	client.cancelGoalsAtAndBeforeTime(ros::Time::now());
	//tell the action client that we want to spin a thread by default
	
	return 0;
}
