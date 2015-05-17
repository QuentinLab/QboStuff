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
	// Make sure the robot is stopped
	
	client_.cancelGoalsAtAndBeforeTime(ros::Time::now());

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
	feasability_client_ = private_nh_.serviceClient<voronoi::Feasability>("feasability");
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
		exit(0);
	}
	else
	{
		//Send Goal to the robot (need to compute orientation)
		next_move_ = computeNextPosition(semantic_places_.poses[closest_asker_.response.index],closest_asker_.response.inVoro);
		next_move_.header.stamp = ros::Time::now();
		goal_.target_pose = next_move_;
		client_.sendGoal(goal_);


		//Memorize current goal
		current_goal_ = next_move_;
		previous_check_= ros::Time::now().toSec();
		//Make it so that current goal is checked as visited in the semantic map
		semantic_visited_[closest_asker_.response.index] = 1;
		
		//Put the robot in patrol mode
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


geometry_msgs::PoseStamped QboMultiOrganizer::computeNextPosition(geometry_msgs::Pose realPose, geometry_msgs::Pose robotPose)
{
	geometry_msgs::PoseStamped final_pose;
	final_pose.header.frame_id = "map";

	double scalar = (realPose.position.x - robotPose.position.x)/sqrt(pow(realPose.position.x - robotPose.position.x,2)+pow(realPose.position.y - robotPose.position.y,2));
	double angle = acos(scalar);

	if (realPose.position.y - robotPose.position.y < 0)
	{
		final_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-angle);	
	}
	else
	{
		final_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
	}
	final_pose.pose.position = robotPose.position;

	return final_pose;

}

bool QboMultiOrganizer::listening_routine(std::vector<int> delays)
{
	printf("Hello");
	int i; // Loops
	int mean = 0;
	int size;
	double Teta;
	audio_places_.poses.clear();
	
	size  = delays.size();
	for (i = 0; i< size; i++)
	{
		mean+= delays[i];
	}
	mean /= size;
	
	// We compute angle if the source is not directly in front of the robot
	if (mean > 4 )
	{
		Teta = acos(((double)mean/44100)*340./0.1475);
	}
	else if (mean < -4)
	{
		Teta = (PI - acos((-(double)mean/44100)*340./0.1475));
	}
	else 
	{
		return 0;
	}
	Teta += -PI/2;


	//Compute positions in cone
	double resolution=0.05;
	double distance1 = 1.0;
	double distance2 = 2.5;
	double distance3 = 4;
	double uncertainty = PI/8;

	tf::TransformListener tf_listener;
	tf_listener.waitForTransform("base_link","map",ros::Time::now(),ros::Duration(1.0));
	geometry_msgs::PoseStamped pose_to_add;
	geometry_msgs::PoseStamped pose_to_add_map;
	
	// First Point
	pose_to_add.pose.position.x =distance1*resolution*cos(Teta);
	pose_to_add.pose.position.y =distance1*resolution*sin(Teta);
	tf_listener.transformPose("map",pose_to_add,pose_to_add_map);
	pose_to_add_map.pose.position.z = 0;
	audio_places_.poses.push_back(pose_to_add_map.pose);

	//Second Point
	pose_to_add.pose.position.x =distance2*resolution*cos(Teta);
	pose_to_add.pose.position.y =distance2*resolution*sin(Teta);
	tf_listener.transformPose("map",pose_to_add,pose_to_add_map);
	pose_to_add_map.pose.position.z = 0;
	audio_places_.poses.push_back(pose_to_add_map.pose);

	//Third point
	pose_to_add.pose.position.x =distance2*resolution*cos(Teta-uncertainty);
	pose_to_add.pose.position.y =distance2*resolution*sin(Teta-uncertainty);
	tf_listener.transformPose("map",pose_to_add,pose_to_add_map);
	pose_to_add_map.pose.position.z = 0;
	audio_places_.poses.push_back(pose_to_add_map.pose);
	
	//4th point
	pose_to_add.pose.position.x =distance2*resolution*cos(Teta+uncertainty);
	pose_to_add.pose.position.y =distance2*resolution*sin(Teta+uncertainty);
	tf_listener.transformPose("map",pose_to_add,pose_to_add_map);
	pose_to_add_map.pose.position.z = 0;
	audio_places_.poses.push_back(pose_to_add_map.pose);

	//5th point
	pose_to_add.pose.position.x =distance3*resolution*cos(Teta);
	pose_to_add.pose.position.y =distance3*resolution*sin(Teta);
	tf_listener.transformPose("map",pose_to_add,pose_to_add_map);
	pose_to_add_map.pose.position.z = 0;
	audio_places_.poses.push_back(pose_to_add_map.pose);
	
	//6th point
	pose_to_add.pose.position.x =distance3*resolution*cos(Teta-uncertainty);
	pose_to_add.pose.position.y =distance3*resolution*sin(Teta-uncertainty);
	tf_listener.transformPose("map",pose_to_add,pose_to_add_map);
	pose_to_add_map.pose.position.z = 0;
	audio_places_.poses.push_back(pose_to_add_map.pose);
	
	//7th point
	pose_to_add.pose.position.x =distance3*resolution*cos(Teta+uncertainty);
	pose_to_add.pose.position.y =distance3*resolution*sin(Teta+uncertainty);
	tf_listener.transformPose("map",pose_to_add,pose_to_add_map);
	pose_to_add_map.pose.position.z = 0;
	audio_places_.poses.push_back(pose_to_add_map.pose);

	//Check feasability

	feasability_checker_.request.poses = audio_places_;
	feasability_client_.call(feasability_checker_);

	audio_places_ = feasability_checker_.response.feasible_poses;

	//Don't forget to fill audio_visited_ with 0 (no place in the cone has been visited yet)
	size = audio_places_.poses.size();
	for (i=0; i < size; i++)
	{
		audio_visited_.push_back(0);
	}

	
	return 1;
}


void QboMultiOrganizer::faceCallback(const geometry_msgs::PoseStamped& face_pos)
{

	// Paremeters for loops and others
	int i; //loop
	int nbdelays; // will contain the number of delays from sound service
	//Test if a face has been seen or not : need to be in mode 1 or 2 else the new face is discarded
	current_check_ = ros::Time::now().toSec();

	if (robot_mode_ == 3)
	{
		count_callbacks_++;
		{
			if (face_pos.pose.position.z == 0)
			{
				count_faces_++;
			}
		}
		if (count_callbacks_ == 10)
		{
			if (count_faces_ >= 8)
			{
				geometry_msgs::PoseArray face_msg;
				face_msg.poses.push_back(face_pos.pose);
				face_visited_.push_back(0);	
				closest_asker_.request.semantic_poses = face_msg;
				closest_asker_.request.visited_semantic= face_visited_;
				find_closest_client_.call(closest_asker_);

				//Check if we have patrolled through all the points
				if (closest_asker_.response.index == 255)
				{
					ROS_INFO("Unable to find a way to face...shutting down");
					client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
					exit(0);
				}
				//If we haven't, chose next semantic place to visit
				else
				{
					next_move_ = computeNextPosition(semantic_places_.poses[closest_asker_.response.index],closest_asker_.response.inVoro);
					next_move_.header.stamp = ros::Time::now();
					goal_.target_pose = next_move_;
					client_.sendGoal(goal_);
					client_.waitForResult(ros::Duration(360.0));
					if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						ROS_INFO("Visited face with success");
						exit(0);
					}
					else
					{
						ROS_INFO("Was unable to visit face");
						client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
						exit(0);
					}
				}
			}
			else
			{
				count_callbacks_ = 0;
				count_faces_ = 0;
				robot_mode_ = 1;
			}

		}
	}
	else if (face_pos.pose.position.z == 0)
	{
		//First Time we find a face
		count_callbacks_ = 0;
		count_faces_ = 1;
		robot_mode_ = 3;
	}
	else 
	{
		if (robot_mode_ == 1)
		{

			//If it is necessary to do an audio check to see if there is a sound source
			if (client_.getState()== actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				//Immobilize the robot
				client_.cancelGoalsAtAndBeforeTime(ros::Time::now());

				//Sound check
				sound_asker_.request.go = 1;
				listen_client_.call(sound_asker_);
				std::vector<int> delaytable;

				//Acquire delays
				nbdelays = sound_asker_.response.delays.size();
				for (i = 0; i < nbdelays;i++)
				{
					delaytable.push_back(sound_asker_.response.delays[i]);
				}

				//See if we got something...
				if (listening_routine(delaytable))
				{
					robot_mode_ == 2;
					closest_asker_.request.semantic_poses = audio_places_;
					closest_asker_.request.visited_semantic = audio_visited_;
					find_closest_client_.call(closest_asker_);
					if (closest_asker_.response.index == 255)
					{
						robot_mode_ = 1;
					}
					else
					{
						next_move_ = computeNextPosition(audio_places_.poses[closest_asker_.response.index],closest_asker_.response.inVoro);
						next_move_.header.stamp = ros::Time::now();
						goal_.target_pose = next_move_;
						client_.sendGoal(goal_);
						current_goal_ = next_move_;
						audio_visited_[closest_asker_.response.index] = 1;
						find_closest_client_.call(closest_asker_);
						client_.sendGoal(goal_);
					}
				}
				//Or not...
				else
				{
					closest_asker_.request.semantic_poses = semantic_places_;
					closest_asker_.request.visited_semantic= semantic_visited_;
					find_closest_client_.call(closest_asker_);

					//Check if we have patrolled through all the points
					if (closest_asker_.response.index == 255)
					{
						ROS_INFO("Done visiting all points of patrol...shutting down");
						client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
					}
					//If we haven't, chose next semantic place to visit
					else
					{
						next_move_ = computeNextPosition(semantic_places_.poses[closest_asker_.response.index],closest_asker_.response.inVoro);
						next_move_.header.stamp = ros::Time::now();
						goal_.target_pose = next_move_;
						client_.sendGoal(goal_);
						current_goal_ = next_move_;
						semantic_visited_[closest_asker_.response.index] = 1;
					}
					
				}
				
			}
			else if (current_check_ - previous_check_ > 3.0)
			{
				client_.cancelGoalsAtAndBeforeTime(ros::Time::now());

				//Sound check
				sound_asker_.request.go = 1;
				listen_client_.call(sound_asker_);
				std::vector<int> delaytable;

				//Acquire delays
				nbdelays = sound_asker_.response.delays.size();
				for (i = 0; i < nbdelays;i++)
				{
					delaytable.push_back(sound_asker_.response.delays[i]);
				}

				//See if we got something...
				if (listening_routine(delaytable))
				{
					robot_mode_ == 2;
					closest_asker_.request.semantic_poses = audio_places_;
					closest_asker_.request.visited_semantic = audio_visited_;
					find_closest_client_.call(closest_asker_);
					if (closest_asker_.response.index == 255)
					{
						robot_mode_ = 1;
					}
					else
					{
						next_move_ = computeNextPosition(audio_places_.poses[closest_asker_.response.index],closest_asker_.response.inVoro);
						next_move_.header.stamp = ros::Time::now();
						goal_.target_pose = next_move_;
						client_.sendGoal(goal_);
						current_goal_ = next_move_;
						audio_visited_[closest_asker_.response.index] = 1;
						find_closest_client_.call(closest_asker_);
						client_.sendGoal(goal_);

					}
				}
				//Or not...
				else
				{
					current_goal_.header.stamp = ros::Time::now();
					goal_.target_pose = current_goal_;
					client_.sendGoal(goal_);
				}
				previous_check_ = current_check_;
			}


		}
		else if (robot_mode_ == 2)
		{
			if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				closest_asker_.request.semantic_poses = audio_places_;
				closest_asker_.request.visited_semantic= audio_visited_;
				find_closest_client_.call(closest_asker_);

				//Check if we have patrolled through all the points in the audio cone
				if (closest_asker_.response.index == 255)
				{
					robot_mode_ == 1;
					audio_places_.poses.clear();
				}
				//If we haven't, chose next audio point to visit place to visit
				else
				{
					next_move_ = computeNextPosition(semantic_places_.poses[closest_asker_.response.index],closest_asker_.response.inVoro);
					next_move_.header.stamp = ros::Time::now();
					goal_.target_pose = next_move_;
					client_.sendGoal(goal_);
					current_goal_ = next_move_;
					semantic_visited_[closest_asker_.response.index] = 1;
				}
			}
		}

		
	}
}





int main(int argc, char** argv)
{
	ros::init(argc,argv,"Itinerary_organizer");
	QboMultiOrganizer myMultiOrga("move_base",true);
	ros::spin();
	return 0;
}
