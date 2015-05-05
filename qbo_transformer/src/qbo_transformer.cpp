#include "qbo_transformer.h"

QboTransformer::~QboTransformer()
{
	ROS_INFO("Done");
}

QboTransformer::QboTransformer()
{
	ROS_INFO("Initialization\n");
	OnInit();
}
void QboTransformer::OnInit()
{
	points_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("/depth_in_map",1);
	points_sub_= private_nh_.subscribe("/camera/depth/points",10,&QboTransformer::pointcloudCallback,this);
}

void QboTransformer::pointcloudCallback(const sensor_msgs::PointCloud2 &pc)
{
	ROS_INFO("Receiving point cloud");
	//cout <<		"frame_id =" << pc.header.frame_id << endl;
	//cout <<	 	"width =" << pc.width << endl;
	//cout <<		"height=" << pc.height << endl;
	//cout <<		"rowstep =" << 	pc.row_step << endl;
	//cout << 	"point step=" << pc.point_step << endl;
	//cout << 	"is dense=" << pc.is_dense << endl;
	printf("Is dense =  %d\n",pc.is_dense);
	/*try
	{
		listener_.lookupTransform("/map","/camera_depth_optical_frame",ros::Time(0),transform_);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	try
	{
		listener_.transformPointCloud("/map",pc,pcout_);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep;
	}*/
	listener_.waitForTransform(pc.header.frame_id,"map",pc.header.stamp,ros::Duration(1.0));
	falsepcin_ = pc;
	falsepcin_.header.frame_id = "/camera_depth_frame_notilt";
	pcl_ros::transformPointCloud("map",pc, pcout_, listener_);
	points_pub_.publish(pcout_);
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"qbo_transform");
	QboTransformer qbo_transform;
	ros::spin();
	return 0;
}
