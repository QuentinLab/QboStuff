#include "xtiontest.h"

XtionTest::XtionTest()
{
	ROS_INFO("GO");
	OnInit();
}

XtionTest::~XtionTest()
{
	ROS_INFO("DONE");
}

void XtionTest::OnInit()
{
	ROS_INFO("Subscribing");
	sub_ = private_nh_.subscribe("/camera/depth/image_raw",1,&XtionTest::mycallback,this);
}

void XtionTest::mycallback(const sensor_msgs::Image & image)
{
	ROS_INFO("Image received");

	cout <<		"width =" << image.width << endl;
	cout << 	"height=" << image.height << endl;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"xtiontest");
	XtionTest myTest;
	ros::spin();
	return 0;
} 
