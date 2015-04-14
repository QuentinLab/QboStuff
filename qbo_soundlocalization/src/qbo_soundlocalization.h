#ifndef QBO_SOUNDLOCALIZATION_H_
#define QBO_SOUNDLOCALIZATION_H_

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <ros/ros.h>
#include <math.h>

#define PI 3.14159265
using namespace std;

class QboLocalizer
{
private:
	void OnInit();
	void Correlation();
	void TetaRad();
	void sampleCallback(const std_msgs::Int16MultiArray& samples);

	double computeMean(double* channel);
	double computeStdev(double* channel,double mean);
	/* Ros Elements */
	
	ros::NodeHandle private_nh_; // Node handle
	ros::Subscriber samples_sub_; // Subscriber to samples topic
	ros::Publisher angle_pub_;	
	std_msgs::Float64 angle_;
	/* Some settings for the sound */

	// double * left_filtre_; // Case where there is pre-filtering
	// double * right_filtre_; // Case where there is pre-filtering
	double* left_; // Left channel
	double* right_; // Right channel
	int * Xdec_; //Shifting betweenchannels
	double* Ycorr_; // Contains correlation
	int n_; // Number of samples on which the correlation is performed
	int Delay_; // Contains the Delay in number of samples
	double D_;  // Distance between the two microphones
	double v_; // Speeds of sound in the air (m/s)
	double Teta_; // Final angle at which the sound source is
	int buffersize_;
	int sampleRate_;
	double meanleft_;
	double meanright_;
	double stdevleft_;
	double stdevright_;
public:
	QboLocalizer();
	virtual ~QboLocalizer();
};

#endif
