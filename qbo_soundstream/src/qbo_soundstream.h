#ifndef QBO_SOUNDSTREAM_H_
#define QBO_SOUNDSTREAM_H_



#include "portaudio.h"
#include "pa_linux_alsa.h"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
using namespace std;

class Qbosoundstream
{
private:
	void OnInit();

	/* 
	ROS ELEMENTS
	*/
	
	ros::NodeHandle private_nh_;
	ros::Publisher samples_pub_;
	std_msgs::Int16MultiArray samples_msg_;
	/*
	Settings for sound capture
	*/

	int NUM_CHANNELS_; // Number of audio channels 
	int FRAMES_PER_BUFFER_; // Number of frames in the buffer
	int SAMPLE_RATE_; // Sample rate at which the sensor acquires data
	int err_; //This is going to be used to check for errors
	int deviceNum_; // Device name... the Xtion name is 4	
	PaStreamParameters inputParameters_; // Parameters of the device
	PaStream* stream_; // Handle to the stream
	
	short int* buffer_; // Buffer containing samples to be sent

	/* Functions */
	void read_samples();
public:
	Qbosoundstream();
	virtual ~Qbosoundstream();
};

#endif
