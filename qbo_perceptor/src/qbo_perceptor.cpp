#include "qbo_perceptor.h"

QboPerceptor::QboPerceptor()
{
	ROS_INFO("Beginning data conversion");
	OnInit();
}

QboPerceptor::~QboPerceptor()
{
	ROS_INFO("End of data conversion");
}

void QboPerceptor::OnInit()
{
	_resolution = 0.05;
	faces_topic_ = "/qbo/facesdetected";
	cloud_topic_ = "/depth_in_map";
	
	_robot_pose.header.stamp = ros::Time::now();
	_robot_pose.header.frame_id = "who cares ";
	_robot_pose.pose.position.z = 0;
	_possible_poses.header.frame_id ="map";
	_possible_poses_pub = private_nh_.advertise<geometry_msgs::PoseArray>("/destination_face",1,this);
	points_sub_ = private_nh_.subscribe(cloud_topic_,1,&QboPerceptor::pointsCallback,this);
}

void QboPerceptor::pointsCallback(const sensor_msgs::PointCloud2 & points)
{
	pcl::fromROSMsg(points,cloud_converted_);
/*	cout <<		"width=" << cloud_converted_.width << endl;
	cout <<		"height=" << cloud_converted_.height << endl;
	cout << 	"cloudpoint size = " << cloud_converted_.points.size() << endl;
	cout << 	"couldpoint z = " << cloud_converted_.at(300,200).z << endl;
*/
	faces_sub_ = private_nh_.subscribe(faces_topic_,1,&QboPerceptor::facesCallback,this);
	

}

void QboPerceptor::facesCallback(const qbo_face_xtion::FacesPos & faces)
{
	int ratio_width = faces.image_width/cloud_converted_.width;
	int ratio_height = faces.image_height/cloud_converted_.height;
	int i,j;
	if (!faces.points.empty())
	{
		for (i=0;i<faces.points.size();i++)
		{
			if (faces.points.at(i).x!=-1000)
			{
				_pose_face.position.x = cloud_converted_.at(faces.points.at(i).x/ratio_width,faces.points.at(i).y/ratio_height).x;
				_pose_face.position.y = cloud_converted_.at(faces.points.at(i).x/ratio_width,faces.points.at(i).y/ratio_height).y;
				_pose_face.position.z = 0;
				if (isnan(_pose_face.position.x) || isnan(_pose_face.position.x))
				{
					cout << "C'EST UN NAN" << endl;
				}
				else
				{
					cout << "C'est un pas un NAN" << endl;
				}

				for (i=1;i<(int)1./_resolution/2;i++)
				{
					_robot_pose.pose.position.x = _pose_face.position.x - 1 + i*_resolution*4;
					for (j=1;j<(int)1./_resolution/2;j++)
					{
						_robot_pose.pose.position.y = _pose_face.position.y- 1 + j*_resolution*4;
						double scalar = (_pose_face.position.x - _robot_pose.pose.position.x)/sqrt(pow(_pose_face.position.x - _robot_pose.pose.position.x,2)+pow(_pose_face.position.y - _robot_pose.pose.position.y,2));
						double angle = acos(scalar);

						if (_pose_face.position.y - _robot_pose.pose.position.y < 0)
						{
							_robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-angle);	
						}
						else
						{
							_robot_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
						}
						if (i*_resolution*4 != 1 && j*_resolution*4 !=0)
						{
							_possible_poses.poses.push_back(_robot_pose.pose);
						}	
					}
				}
				_possible_poses.header.stamp = ros::Time::now();
				_possible_poses_pub.publish(_possible_poses);
				_possible_poses.poses.clear();

			}
			/*else
			{
				ROS_INFO("Nothing to see here");
			}*/	
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"qbo_perceptor");
	QboPerceptor qbo_perceptor;
	ros::spin();
	return 0;
}
