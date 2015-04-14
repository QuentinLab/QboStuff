#ifndef _QBO_SOUND_TRANSFORMATION_H_
#define _QBO_SOUND_TRANSFORMATION_H_

#include <iostream>
#include <string>
#include <cmath>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#define PI 3.14159265


using namespace std;

class QboSoundTransformation
{
private:
	/* ROS ELEMENTS */
	ros::NodeHandle _private_nh;
	ros::Subscriber _angle_sub;
	ros::Publisher _audio_poses_pub;

	geometry_msgs::PoseStamped _pose_to_add;
	geometry_msgs::PoseStamped _pose_to_add_map;
	geometry_msgs::PoseArray _audio_poses;

	
	/* Methods */
	void OnInit();
	void angleCallback(const std_msgs::Float64 & angle_found);
	
	/* Elements for transform */

	tf::TransformListener _tf_listener;

	/* Parameters */

	double _angle_cone;
	double _resolution;
	double _uncertainty_rad;
	int _distance_observed;
	int _angular_observed;
public:
	QboSoundTransformation();
	virtual ~QboSoundTransformation();
};

#endif
