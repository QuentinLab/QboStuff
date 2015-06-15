#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


int main(int argc,char** argv)
{

	ros::init(argc,argv,"turnaround");
	ros::NodeHandle private_nh;
	ros::Publisher twistpub = private_nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
	int i;
	geometry_msgs::Twist datwist;

	datwist.linear.x = 0;
	datwist.linear.y = 0;
	datwist.linear.z = 0;
	datwist.angular.x = 0;
	datwist.angular.y = 0;
	datwist.angular.z = 0;

	twistpub.publish(datwist);	

	for (i = 0; i< 4; i++)
	{
		ROS_INFO("COMMANDE");
		ros::Duration(3).sleep();
		datwist.linear.x = 0;
		datwist.linear.y = 0;
		datwist.linear.z = 0;

		datwist.angular.x = 0;
		datwist.angular.y = 0;
		datwist.angular.z = 1.70;

		twistpub.publish(datwist);
	}
}
		
