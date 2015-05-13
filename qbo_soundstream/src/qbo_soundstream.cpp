#include "qbo_soundstream.h"

Qbosoundstream::Qbosoundstream()
{
	ROS_INFO("Initializing Qbo soundstream node");
	OnInit();
}

Qbosoundstream::~Qbosoundstream()
{
	err_ = Pa_StopStream(stream_);
	if (err_ != paNoError)
	{
		cout << "The audio stream was not properly stopped" << endl;
	}
	
	err_ = Pa_CloseStream(stream_);
	if (err_ != paNoError)
	{
		cout << "The audio stream was not properly closed" << endl;
	}

	ROS_INFO("Audio acquisition successfully ended !");
}

void Qbosoundstream::read_samples()
{
	std::vector<short int> data;
	while (ros::ok())
	{
//		err_ = Pa_WriteStream( stream_,buffer_, FRAMES_PER_BUFFER_ );

		err_ = Pa_ReadStream(stream_,buffer_,FRAMES_PER_BUFFER_);
		if (err_)
		{
			cout << "Error while reading samples..." << endl;
		}
		else
		{
			for (err_=0;err_<2*FRAMES_PER_BUFFER_;err_++)
			{
				data.push_back(buffer_[err_]);
			}
			samples_msg_.data = data;
			samples_pub_.publish(samples_msg_);
		}
		data.clear();
	}
}




void Qbosoundstream::OnInit()
{
	NUM_CHANNELS_ = 2;
	FRAMES_PER_BUFFER_ = 22050;
	SAMPLE_RATE_ = 44100;
	err_ = Pa_Initialize();
	deviceNum_ = 4;

	if (err_ != paNoError)
	{
		cout << "Problem Initializing the device handle. Restart the node please."<< endl;
	}

	inputParameters_.device = deviceNum_;
	inputParameters_.channelCount = NUM_CHANNELS_;
	inputParameters_.sampleFormat = paInt16;
	inputParameters_.suggestedLatency = Pa_GetDeviceInfo(inputParameters_.device)->defaultHighInputLatency;
	inputParameters_.hostApiSpecificStreamInfo = NULL;

	buffer_ = (short int*) malloc(sizeof(short int)*FRAMES_PER_BUFFER_*2);


	/* Testing for play back */
	/*PaStreamParameters outputParameters;
	outputParameters.device = 0;//default output device 
	outputParameters.channelCount = NUM_CHANNELS_;
	outputParameters.sampleFormat = paInt16;
	outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultHighOutputLatency;
	outputParameters.hostApiSpecificStreamInfo = NULL;*/
	
	// Opening stream

	ROS_INFO("Opening stream\n");
	err_ = Pa_OpenStream(&stream_,&inputParameters_,NULL,SAMPLE_RATE_,FRAMES_PER_BUFFER_,paClipOff,NULL,NULL);
	
	if (err_ != paNoError)
	{
		cout << "There has been an error in opening the stream" << endl;
	}
	else
	{
		ROS_INFO("	Stream opened correctly\n");
	}

	// Starting stream
	
	ROS_INFO("Starting stream\n");
	err_ = Pa_StartStream(stream_);
	
	if (err_!= paNoError)
	{
		cout << "There has been an error in starting the stream" << endl;
	}
	else
	{
		ROS_INFO("	Stream started correctly\n");
	}
	samples_pub_ = private_nh_.advertise<std_msgs::Int16MultiArray>("/qbo_soundstream/samples",1);

	read_samples();	
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"qbo_soundstream");
	Qbosoundstream qbo_soundstream;
	return 0;
}
	
