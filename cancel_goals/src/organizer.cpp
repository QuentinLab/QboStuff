 #include "organizer.h"

 Organizer::Organizer(std::string name, bool istrue) : client_(name,istrue)
 {
	 ROS_INFO("Hello");
	 onInit();
 }

 Organizer::~Organizer()
 {
	 ROS_INFO("Goodbye");
 }

void Organizer::onInit()
{
	while (!client_.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting ...");
	}
	if (client_.isServerConnected())
	{
		ROS_INFO("Connected...");
	}
	face_found_sub_ = private_nh_.subscribe("/face_found",1,&Organizer::faceFoundCallback,this);
}
void Organizer::faceFoundCallback(const std_msgs::UInt8 &face_found)
{
	if (face_found.data == 0)
	{
		ROS_INFO("NO FACE DOING NOTHING");
	}
	else
	{
	 	client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
		ROS_INFO("Cancelled all goals");
	}
}

int main(int argc, char** argv)
{
        ros::init(argc,argv,"organizer");
        Organizer my_organizer("move_base",true);
        ros::spin();
        return 0;
}
