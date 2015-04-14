#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <ros/ros.h>

using namespace std;
class XtionTest
{
private:
	ros::NodeHandle private_nh_;
	ros::Subscriber sub_;
	void OnInit();	
	void mycallback(const sensor_msgs::Image & image);

public:
	XtionTest();
	virtual ~XtionTest();
};
