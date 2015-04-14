#ifndef _QBO_PERCEPTOR_H_
#define _QBO_PERCEPTOR_H_

#include <iostream>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <qbo_face_xtion/FacesPos.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
using namespace std;

class QboPerceptor
{
private:
	/*
	ROS ELEMENTS
	*/
	ros::NodeHandle private_nh_;
	ros::Subscriber points_sub_;
	ros::Subscriber faces_sub_;
	ros::Publisher  _possible_poses_pub;
	geometry_msgs::Pose _pose_face;
	geometry_msgs::PoseArray _possible_poses;
	geometry_msgs::PoseStamped _robot_pose;	
	/*
	Parameters
	*/
	string cloud_topic_;
	string faces_topic_;
	double _resolution;
	pcl::PointCloud<pcl::PointXYZ> cloud_converted_;
	/*
	METHODS
	*/
	void OnInit();
	void pointsCallback(const sensor_msgs::PointCloud2 & points);
	void facesCallback(const qbo_face_xtion::FacesPos & faces);
	void conversion(sensor_msgs::PointCloud2 & points);
	
public:
	QboPerceptor();
	virtual ~QboPerceptor();		
};

#endif
