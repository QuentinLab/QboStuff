#include "qbo_soundtransformation.h"

QboSoundTransformation::QboSoundTransformation()
{
	ROS_INFO("Beginning Transformation :");
	OnInit();
}

QboSoundTransformation::~QboSoundTransformation()
{
	ROS_INFO("Ending Transformation :");
}

void QboSoundTransformation::OnInit()
{
	_pose_to_add.header.frame_id = "base_link";
	_audio_poses.header.frame_id = "map";
	
	_resolution = 0.05;
	_uncertainty_rad = 7*PI/180; /* Cone of 7 degrees ucnertainty */
	_distance_observed = 5; // We have a 5meter long cone
	_angular_observed = 15; // 15 degree large cone


	_audio_poses_pub = _private_nh.advertise<geometry_msgs::PoseArray>("/audio_poses",100);
	_angle_sub = _private_nh.subscribe("/audio_angle",10,&QboSoundTransformation::angleCallback,this);
}



void QboSoundTransformation::angleCallback(const std_msgs::Float64 & angle_found)
{
	_pose_to_add.header.stamp = ros::Time::now();
	_audio_poses.header.stamp = ros::Time::now();
	_tf_listener.waitForTransform("base_link","map",_pose_to_add.header.stamp,ros::Duration(1.0));


	int r = 0;
	int theta = 0;
	for (r = 0; r<(int)(_distance_observed/_resolution);r++)
	{
		_angle_cone = angle_found.data - _uncertainty_rad; 
		for (theta = 0; theta < _angular_observed; theta ++)
		{
			_angle_cone += PI/180;
			_pose_to_add.pose.position.x =r*_resolution*cos(_angle_cone);
			_pose_to_add.pose.position.y =r*_resolution*sin(_angle_cone);
			_pose_to_add.pose.orientation = tf::createQuaternionMsgFromYaw(_angle_cone);
			_tf_listener.transformPose("map",_pose_to_add,_pose_to_add_map);
			_pose_to_add_map.pose.position.z = 0;
			_audio_poses.poses.push_back(_pose_to_add_map.pose);
		}
	}
	_audio_poses_pub.publish(_audio_poses);
	_audio_poses.poses.clear();
}



int main(int argc, char ** argv)
{
	ros::init(argc,argv,"qbo_soundtransformation");
	QboSoundTransformation soundTransformer;
	ros::spin();
	return 0;
}
