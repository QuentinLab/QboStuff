#include <qbo_soundstream/soundAcquisition.h>
#include <std_msgs/UInt8.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include "math.h"


int main(int argc, char** argv)
{
	ros::init(argc,argv,"requestsound");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<qbo_soundstream::soundAcquisition>("request_sound");
	qbo_soundstream::soundAcquisition srv;
	srv.request.go = 1;

	client.call(srv);
	return 0;
}
	
