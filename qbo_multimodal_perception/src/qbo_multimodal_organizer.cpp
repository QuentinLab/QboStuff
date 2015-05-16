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
	
	int i; //loop
	int nb_places; //nb of elements of the semantic map

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
	listen_client_ = private_nh_.serviceClient<qbo_soundstream::soundAcquisition>("request_sound");
	find_closest_client_ = private_nh_.serviceClient<voronoi::FindClosestInPoseArray>("compute_closest_point"); 
	robot_mode_ = 1;// Default mode is patrol

	//Define semantic map here
	
	semanticMap();
	nb_places = semantic_places_.poses.size();

	//semantic_visited_ = (unsigned char*) malloc(sizeof(unsigned char)*nb_places);

	for (i=0; i<nb_places;i++)
	{
		//semantic_visited_[i] = 0;
		semantic_visited_.push_back(0);
	}
	//Compute closest point in the semantic map

	closest_asker_.request.semantic_poses = semantic_places_;
	closest_asker_.request.visited_semantic = semantic_visited_;

	if (!find_closest_client_.call(closest_asker_))
	{
		ROS_INFO("Impossible for the organizer to find the first point");
	}
	else
	{
		//Send Goal to the robot (need to compute orientation)
		robot_mode_ = 1; 
	}
		
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
	if (face_pos.pose.position.z == 0 &&  robot_mode_ != 2)
	{
		// We found a face, we want to visit it for sure
		next_move_ = face_pos;
		next_move_.header.stamp = ros::Time::now();
		goal_.target_pose = next_move_;
		client_.sendGoal(goal_);

		//We can wait for result as nothing else matters
		client_.waitForResult();
		if (client_.getState() ==  actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			printf("Hello !");
		}
	}
	else if (face_pos.pose.position.z == 0 && robot_mode_ == 2) 
	{
		
	}
}





int main(int argc, char** argv)
{
	ros::init(argc,argv,"Itinerary_organizer");
	QboMultiOrganizer myMultiOrga("move_base",true);
	ros::spin();
	return 0;
}
