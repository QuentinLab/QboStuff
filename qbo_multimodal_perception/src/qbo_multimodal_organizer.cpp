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

	robot_mode_ = 1;// Default mode is patrol

	//Chose the first point of the itinerary

/* NEED THE SEMANTIC MAP HERE*/

	// Subscribe to topic with face

	face_position_sub_ = private_nh_.subscribe("/face_position",1,&QboMultiOrganizer::faceCallback,this);
}


void QboMultiOrganizer::faceCallback(const geometry_msgs::PoseStamped& face_pos)
{
	//Test if a face has been seen or not : need to be in mode 1 or 2 else the new face is discarded
	if (face_pos.pose.position.z != 0)
	{
		//In this case, no face was detected
	}
	else
	{
		//In this case, a face has been detected, we must go see it
	}
}





int main(int argc, char** argv)
{
	ros::init(argc,argv,"Itinerary_organizer");
	QboMultiOrganizer myMultiOrga("move_base",true);
	ros::spin();
	return 0;
}
