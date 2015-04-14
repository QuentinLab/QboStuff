#include "qbo_map_reader.h"

QboMapReader::QboMapReader()
{	
	ROS_INFO("Beginning");
	OnInit();
}

QboMapReader::~QboMapReader()
{	
	ROS_INFO("Released");
}

void QboMapReader::OnInit()
{
	_fixed_pose_pub = _private_nh.advertise<geometry_msgs::PoseStamped>("/fixed_pose",100);
	_robot_pose_pub = _private_nh.advertise<geometry_msgs::PoseStamped>("/robot_pose",100);
	_origin_world_pub = _private_nh.advertise<geometry_msgs::PoseStamped>("/origin",100);
	_possible_poses_pub = _private_nh.advertise<geometry_msgs::PoseArray>("/possible_poses",100);
	_poses_audio_pub = _private_nh.advertise<geometry_msgs::PoseArray>("/audio_poses",100);
	_fixed_pose.pose.position.x  = 0.000;
	_fixed_pose.pose.position.y  = 0.000;
	_fixed_pose.pose.position.z  = 0.000;

	_fixed_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.000);

	/*_fixed_pose.pose.orientation.x = 0.000;
	_fixed_pose.pose.orientation.y = 0.000;
	_fixed_pose.pose.orientation.z = -0.806; 
	_fixed_pose.pose.orientation.w = 0.592;*/
	_fixed_pose.header.frame_id = "map";
	_robot_pose.header.frame_id = "map";
	_origin_world.header.frame_id = "map";
	_possible_poses.header.frame_id = "map";
	_poses_audio.header.frame_id = "map";
	_pose_heard.header.frame_id = "map";
	_map_data_sub = _private_nh.subscribe("/map_metadata",1,&QboMapReader::metadataCallback,this);


}

void QboMapReader::metadataCallback(const nav_msgs::MapMetaData & data)
{

	/*_origin_x.data = data.origin.position.x;
	_origin_y.data = data.origin.position.y;*/
	_origin_x.data = 0.0;
	_origin_y.data = 0.0;
	_resolution.data = data.resolution;

	_origin_world.pose.position.x = 0.0;
	_origin_world.pose.position.y = 0.0;
	_origin_world.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
	Circling();	
	


}
void QboMapReader::Circling()
{
	_robot_yaw = tf::getYaw(_fixed_pose.pose.orientation);
	cout << "robot yaw initial : "<< _robot_yaw << endl;
	_robot_yaw += PI/4-PI/16;
	_pose_heard.pose.orientation = tf::createQuaternionMsgFromYaw(_robot_yaw);
	_pose_heard.pose.position = _fixed_pose.pose.position;

	ros::Rate loop_rate(1);
	int i,j;
	while (ros::ok())
	{

		_origin_world.header.stamp = ros::Time::now();
		_fixed_pose.header.stamp = ros::Time::now();
		_fixed_pose_pub.publish(_fixed_pose);
		_robot_pose.pose = _fixed_pose.pose;
		_robot_pose.header.stamp = ros::Time::now();
		_possible_poses.header.stamp = ros::Time::now();
		_pose_heard.header.stamp = ros::Time::now();

		int r = 0;
		int theta = 0;
		for (r = 0; r<(int)(5/_resolution.data);r++)
		{
			_robot_yaw = PI/4 - PI/16;
			for (theta = 0; theta < 15; theta ++)
			{
				_robot_yaw += PI/180;
				_pose_heard.pose.position.x = _fixed_pose.pose.position.x + r*0.05*cos(_robot_yaw);
				_pose_heard.pose.position.y =_fixed_pose.pose.position.y + r*0.05*sin(_robot_yaw);
				_pose_heard.pose.orientation = tf::createQuaternionMsgFromYaw(_robot_yaw);
				_poses_audio.poses.push_back(_pose_heard.pose);
			}
		}
		_poses_audio_pub.publish(_poses_audio);
		_poses_audio.poses.clear();
	
	
/*		for (i=1;i<(int)1./_resolution.data/2;i++)
		{
			cout << "i = " << i << endl;
			_robot_pose.pose.position.x = _fixed_pose.pose.position.x - 1 + i*_resolution.data*4;
			for (j=1;j<(int)1./_resolution.data/2;j++)
			{
				_robot_pose.pose.position.y = _fixed_pose.pose.position.y- 1 + j*_resolution.data*4;
				double scalar = (_fixed_pose.pose.position.x - _robot_pose.pose.position.x)/sqrt(pow(_fixed_pose.pose.position.x - _robot_pose.pose.position.x,2)+pow(_fixed_pose.pose.position.y - _robot_pose.pose.position.y,2));
				double angle = acos(scalar);

				if (_fixed_pose.pose.position.y - _robot_pose.pose.position.y < 0)
				{
					_robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-angle);	
				}
				else
				{
					_robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
				}
				if (i*_resolution.data*4 != 1 && j*_resolution.data*4 !=0)
				{
					_possible_poses.poses.push_back(_robot_pose.pose);
				}	
			}
		}
		_possible_poses_pub.publish(_possible_poses);
		_possible_poses.poses.clear();*/


		loop_rate.sleep();
	}
}

int main (int argc, char ** argv)
{
	ros::init(argc,argv,"qbo_map_reader");
	QboMapReader Reader;
	ros::spin();
	return 0;
}
