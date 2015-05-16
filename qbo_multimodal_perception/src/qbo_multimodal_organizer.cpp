#include "qbo_multimodal_organizer.h"


QboMultiOrganizer::QboMultiOrganizer() : client_("move_base",true)
{
	ROS_INFO("No initialization of the MoveBaseClient...Initialized with ""move_base"" and true as arguments");
}

QboMultiOrganizer::QboMultiOrganizer(string name, bool istrue): client_(name,istrue)
{
	ROS_INFO("Creating object QboMultiOrganizer");
	OnInit();
}

QboMultiOrganizer::~QboMultiOrganizer()
{
	ROS_INFO("Leaving the node.");
}

void QboMultiOrganizer::OnInit()
{
	ROS_INFO("OnInit...");
	
	//Initialization of the clients

	while (!client_.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting ...");
	}
	if (client_.isServerConnected())
	{
		ROS_INFO("Connected...");
	}

	plan_client_ = private_nh_.serviceClient<voronoi::MakeNavPlan>("make_plan");
	listen_client_ = listen_client_.serviceClient<qbo_soundstream::SoundAcquisition>("request_sound");

	robot_mode_ = 1;// Default mode is patrol

	//Define semantic map here
	
	semanticMap();

	//Compute closest point in the semantic map
		
	// Subscribe to topic with face

	face_position_sub_ = private_nh_.subscribe("/face_position",1,&QboMultiOrganizer::faceCallback,this);
}

void QboMultiOrganizer::semanticMap()
{
	geometry_msgs::Pose semantic_pose;
	semantic_pose.position.x = 0;
	semantic_pose.position.y = 0;
	semantic_pose.position.z = 0;
	semantic_pose.orientation.x = 0;
	semantic_pose.orientation.y = 0;
	semantic_pose.orientation.z = 0;
	semantic_pose.orientation.w = 0;

	semantic_places_.poses.push_back(semantic_pose);


	semantic_pose.position.x = 0;
	semantic_pose.position.y = 0;
	semantic_pose.position.z = 0;
	semantic_pose.orientation.x = 0;
	semantic_pose.orientation.y = 0;
	semantic_pose.orientation.z = 0;
	semantic_pose.orientation.w = 0;
	
	semantic_places_.poses.push_back(semantic_pose);

	semantic_pose.position.x = 0;
	semantic_pose.position.y = 0;
	semantic_pose.position.z = 0;
	semantic_pose.orientation.x = 0;
	semantic_pose.orientation.y = 0;
	semantic_pose.orientation.z = 0;
	semantic_pose.orientation.w = 0;

	semantic_places_.poses.push_back(semantic_pose);
}

void QboMultiOrganizer::faceCallback(const geometry_msgs::PoseStamped& face_pos)
{
	//Test if a face has been seen or not : need to be in mode 1 or 2 else the new face is discarded
	if (face_pos.pose.position.z != 0 && robot_mode_ != 2)
	{
		//In this case, no face was detected
	}
	else
	{
		//In this case, a face has been detected, we must go see it
		next_move_ = face_pos;
		next_move_.header.stamp = ros::Time::now();
		goal_.target_pose = next_move_;
		client_.sendGoal(goal_);
		
	}
}





int main(int argc, char** argv)
{
	ros::init(argc,argv,"Itinerary_organizer");
	QboMultiOrganizer myMultiOrga("move_base",true);
	ros::spin();
	return 0;
}
