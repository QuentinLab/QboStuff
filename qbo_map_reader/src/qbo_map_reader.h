#ifndef _QBO_MAP_READER_
#define _QBO_PERCEPTOR_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int8.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <cmath>


#define PI 3.14
using namespace std;
class QboMapReader{
private:

	/* ROS ELEMENTS*/
	ros::NodeHandle _private_nh;
	ros::Publisher _fixed_pose_pub;
	ros::Publisher _robot_pose_pub;
	ros::Publisher _origin_world_pub;
	ros::Publisher _possible_poses_pub;
	ros::Publisher _poses_audio_pub;
	ros::Subscriber _map_data_sub;

	double _xr;
	void OnInit();
	void Circling();
	double _robot_yaw;

	/* Callback */

	void metadataCallback(const nav_msgs::MapMetaData & data);

	/* Messages */
	geometry_msgs::PoseStamped _fixed_pose;
	geometry_msgs::PoseStamped _robot_pose;
	geometry_msgs::PoseStamped _origin_world;	
	geometry_msgs::PoseArray _poses_audio;
	geometry_msgs::PoseStamped _pose_heard;
	geometry_msgs::PoseArray _possible_poses;

	/* Parameters */
	std_msgs::Float64 _origin_x;
	std_msgs::Float64 _origin_y;
	std_msgs::Float32 _resolution;

public:
	QboMapReader();
	virtual ~QboMapReader();
};

#endif
