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
	_pose_array.header.frame_id = "map";
	points_pub_ = private_nh_.advertise<geometry_msgs::PoseArray>("/pointcloud",10);
	points_sub_ = private_nh_.subscribe("/camera/depth/points",10,&QboTransformer::pointcloudCallback,this);
}


void QboTransformer::pointcloudCallback(const sensor_msgs::PointCloud2 &pc)
{
	ROS_INFO("Receiving point cloud");
	listener_.waitForTransform(pc.header.frame_id,"map",pc.header.stamp,ros::Duration(1.0));
	pcl_ros::transformPointCloud("map",pc, pcout_, listener_);
	pcl::fromROSMsg(pcout_,cloud_converted_);
	for (int i=0;i<cloud_converted_.width;i++)
	{
		for (int j=0;j<cloud_converted_.height;j++)
		{
			if(!isnan(cloud_converted_.at(i,j).x) && !isnan(cloud_converted_.at(i,j).y) && !isnan(cloud_converted_.at(i,j).z))
			{
				_pose.position.x = cloud_converted_.at(i,j).x;
				_pose.position.y = cloud_converted_.at(i,j).y;
				_pose.position.z = cloud_converted_.at(i,j).z;
				_pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
				_pose_array.poses.push_back(_pose);
			}
		}
	}
	_pose_array.header.stamp = ros::Time::now();
	points_pub_.publish(_pose_array);
	cout << "Size of cloud = " << _pose_array.poses.size();
	_pose_array.poses.clear();
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"mytransformer");
	QboTransformer mytransformer;
	ros::spin();
	return 0;
}
