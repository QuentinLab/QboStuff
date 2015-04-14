#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main (int argc,char** argv)
{
	ros::init(argc,argv, "navivi");

	MoveBaseClient ac("move_base",true);

	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("ON ATTEND LE SERVER");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 1.0;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("J'ENVOIE");
	ac.sendGoal(goal);
	ac.waitForResult();
	
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("YES");
	else
		ROS_INFO("NO");

	return 0;
}
