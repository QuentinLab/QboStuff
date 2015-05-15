#include "qbo_request.h"

QboSoundRequest::QboSoundRequest()
{
	ROS_INFO("Initializing the service node for sound");
	OnInit();
}

QboSoundRequest::~QboSoundRequest()
{
	ROS_INFO("Service node was killed");
}

void QboSoundRequest::OnInit()
{
	int i; //looping parameters
	//Parameters for stream gestion

	NUM_CHANNELS_= 2;
	FRAMES_PER_BUFFER_ = 22050;
	SAMPLE_RATE_ = 44100;
	NB_SECONDS_ = 3;
	deviceNum_ = 4;
	NUMBER_BUFFER_ = NB_SECONDS_*SAMPLE_RATE_/FRAMES_PER_BUFFER_; 

	

	buffer_ = (short int**) malloc(sizeof(short int*)*NUMBER_BUFFER_);
	for (i = 0; i < NUMBER_BUFFER_;i++)
	{
		buffer_[i] = (short int*) malloc(sizeof(short int)*FRAMES_PER_BUFFER_*2);
	}


	// Parameters for angle computation
	n_ = 20;
	left_= (double*) malloc(sizeof(double)*FRAMES_PER_BUFFER_);
	right_= (double*) malloc(sizeof(double)*FRAMES_PER_BUFFER_);
	delays_ = (int*) malloc(sizeof(int)*NUMBER_BUFFER_);
	Ycorr_ = (double*) malloc(sizeof(double)*(2*n_+1));
	Xdec_ = (int*) malloc(sizeof(int)*(2*n_+1));
	D_ = 0.1475; // Distance between the two microphones
	v_ = 340.;
	

	// Advertising the service 
	request_sound_srv_ = private_nh_.advertiseService("request_sound",&QboSoundRequest::acquireService,this);

	//Ready to go
	ROS_INFO("Ready to receive sounds...");
}

bool QboSoundRequest::acquireService(qbo_soundstream::soundAcquisition::Request& req, qbo_soundstream::soundAcquisition::Response& resp)
{
	int i,j; // loops

	// Reading the samples for the needed time

	open_stream();

	read_samples();

	close_stream();
	

	/* For each buffer we compute the delay */

	for (i = 0; i < NUMBER_BUFFER_;i++)
	{

		/* Reorganize left and right buffer */
		for (j = 0; j < FRAMES_PER_BUFFER_;j++)
		{
			left_[j] = (double)buffer_[i][2*j];
			right_[j] = (double) buffer_[i][2*j+1];
		}
		
		/* Correlation */
		Correlation();
		delays_[i] = Delay_;
		cout << "delay : "<< delays_[i] << endl;
		
		/* Compute the angle */
//		TetaRad();
	}

	resp.delays.push_back(1);
	return true;
}


void QboSoundRequest::open_stream()
{
	err_ = Pa_Initialize();
	if (err_ != paNoError)
	{
		cout << "Problem Initializing the device handle. Restart the node please."<< endl;
	}

	inputParameters_.device = deviceNum_;
	inputParameters_.channelCount = NUM_CHANNELS_;
	inputParameters_.sampleFormat = paInt16;
	inputParameters_.suggestedLatency = Pa_GetDeviceInfo(inputParameters_.device)->defaultHighInputLatency;
	inputParameters_.hostApiSpecificStreamInfo = NULL;
	
	ROS_INFO("Opening stream");
	err_= Pa_OpenStream(
	&stream_,
	&inputParameters_,
	NULL,
	SAMPLE_RATE_,
	FRAMES_PER_BUFFER_,
	paClipOff, /* we won't output out of range samples so don't bother clipping them */
	NULL, /* no callback, use blocking API */
	NULL ); /* no callback, so no callback userData */

	if( err_!= paNoError )
	{
		cout <<"There has been an error in opening the stream" << endl;
	}

	/* -- start stream -- */
	err_= Pa_StartStream( stream_ );

	if( err_!= paNoError )
	{
		cout << "There has been an error in starting the stream" << endl;
	}
}

void QboSoundRequest::close_stream()
{
	ROS_INFO("Closing stream");	
	err_= Pa_StopStream( stream_ );
	if( err_!= paNoError )
	{
		cout << "Problem stopping stream" << endl;
	}
	/* -- don't forget to cleanup! -- */
	err_= Pa_CloseStream( stream_ );
	if( err_!= paNoError )
	{
		cout << "problem closing stream" << endl;
	}
	Pa_Terminate();
}


void QboSoundRequest::read_samples()
{
	int i; // loops
	ROS_INFO("Reading samples..");
	for (i = 0; i < NUMBER_BUFFER_; i++)
	{
		err_ = Pa_ReadStream( stream_, buffer_[i], FRAMES_PER_BUFFER_ );
	}
	if (err_)
	{
		cout << "Error when reading the stream" << endl;
	}
		
}

void QboSoundRequest::Correlation()
{
	double Max = 0;
	int SampleDelay = 0;
	int i;
	for(i=0; i < (2 * n_ + 1); i++)
	{
		Ycorr_[i] = 0;
		Xdec_[i] = i - n_;
		if( (n_ - i) >= 0)
		{
			int k;
			for(k = 0; k < (FRAMES_PER_BUFFER_ - (n_ - i)); k += 1)
			{
				Ycorr_[i] += left_[k + (n_ - i)] * right_[k];
			}
		}
		else
		{
			int k;			
			for(k = 0; k < (FRAMES_PER_BUFFER_ + n_ - i); k+=1)
			{
				Ycorr_[i] += left_[k] *right_[k - n_ + i];
			}
		}
		if(Max < Ycorr_[i])
		{
			Max = Ycorr_[i];
			SampleDelay = Xdec_[i];
		}
	}
	Delay_ = SampleDelay;
	cout << "Sample delay :" << SampleDelay << endl;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"qbo_soundrequest");
	QboSoundRequest qbo_soundrequest;
	ros::spin();
	return 0;
}
