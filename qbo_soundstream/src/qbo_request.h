#ifndef QBO_REQUEST_H_
#define QBO_REQUEST_H_

#include "portaudio.h"
#include "pa_linux_alsa.h"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <qbo_soundstream/soundAcquisition.h>

#define PI 3.14159265

using namespace std;
class QboSoundRequest
{
private:

	// ROS parameters

	ros::NodeHandle private_nh_;
	ros::ServiceServer request_sound_srv_;
	// Parameters for stream opening

	int NUM_CHANNELS_;
	int FRAMES_PER_BUFFER_; // Number of frames in the buffer
	int NB_SECONDS_; // Number of second of acquisition 
	int SAMPLE_RATE_; // Sample rate at which the sensor acquires data
	int NUMBER_BUFFER_;
	int err_; //This is going to be used to check for errors
	int deviceNum_; // Device name... the Xtion name is 4	
	PaStreamParameters inputParameters_; // Parameters of the device
	PaStream* stream_; // Handle to the stream
	short int** buffer_; // Buffer containing samples to be sent

	//Parameters for angle computation

	double* left_;
	double* right_;
	int * Xdec_; //Shifting betweenchannels
	double* Ycorr_; // Contains correlation
	int n_; // Number of samples on which the correlation is performed
	int Delay_; // Contains the Delay in number of samples
	double D_;  // Distance between the two microphones
	double v_; // Speeds of sound in the air (m/s)
	double Teta_; // Final angle at which the sound source is
	int* delays_; // Table of Delays in samples
	
	
	/* Functions */
	void OnInit();
	bool acquireService(qbo_soundstream::soundAcquisition::Request& req, qbo_soundstream::soundAcquisition::Response& resp);

	// Stream gestion
	void open_stream();
	void read_samples();
	void close_stream();

	// Correlation gestion

	void Correlation();
	void TetaRad();


public:
	QboSoundRequest();
	virtual ~QboSoundRequest();
};

#endif
