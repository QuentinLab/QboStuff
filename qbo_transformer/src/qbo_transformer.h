#include <iostream>
#include <tf/transform_listener.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/filters/filter.h>
using namespace std;

class QboTransformer
{
private:
	void OnInit();	
	void pointcloudCallback(const sensor_msgs::PointCloud2 & pc);
	/*
	ROS ELEMENTS
	*/
	
	ros::NodeHandle private_nh_;
	ros::Subscriber points_sub_;
	ros::Publisher points_pub_;
	tf::TransformListener listener_;
	tf::StampedTransform transform_;
	sensor_msgs::PointCloud2 pcout_;
	sensor_msgs::PointCloud2 falsepcin_;


	pcl::PointCloud<pcl::PointXYZ> cloud_Nans_;
	pcl::PointCloud<pcl::PointXYZ> cloud_converted_;
	geometry_msgs::Pose _pose;
	geometry_msgs::PoseArray _pose_array;
	std::vector<int> vector_;
public:
	QboTransformer();
	virtual ~QboTransformer();
};
