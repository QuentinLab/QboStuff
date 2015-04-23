#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <voronoi/MakeNavPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "requestplan");
	
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<voronoi::MakeNavPlan>("make_plan");
	voronoi::MakeNavPlan srv;
	srv.request.start.header.stamp = ros::Time::now();
	srv.request.start.header.frame_id = "map";
	srv.request.goal.header.stamp = ros::Time::now();
	srv.request.goal.header.frame_id = "map";

	// Goal position 

	srv.request.goal.pose.position.x = 1.404;
	srv.request.goal.pose.position.y = 2.869;
	srv.request.goal.pose.position.z = 0.000;

	// Goal orientation
	srv.request.goal.pose.orientation.x = 0.0;
	srv.request.goal.pose.orientation.y = 0.0;
	srv.request.goal.pose.orientation.z = 0.0;
	srv.request.goal.pose.orientation.w = 1.0;

	// Start position
	
	srv.request.start.pose.position.x = 1.200;
	srv.request.start.pose.position.y = -1.372;
	srv.request.start.pose.position.z = 0.000;

	// Start orientation

	srv.request.goal.pose.orientation.x = 0.000;
	srv.request.goal.pose.orientation.y = 0.000;
	srv.request.goal.pose.orientation.z = 0.000;
	srv.request.goal.pose.orientation.w = 1.000;

	if (client.call(srv))
	{
		printf("Plan_found = %d\n", srv.response.plan_found);
	}
	return 0;
}
