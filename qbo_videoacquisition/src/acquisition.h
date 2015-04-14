#ifndef _VIDEOACQUISITOR_
#define _VIDEOACQUISITOR_

#include <cv.h>
#include <highgui.h>
#include <sstream>
#include <cvaux.h>
#include <cxmisc.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <string>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

class VideoAcquisitor
{
private:
	void onInit();
	ros::NodeHandle _private_nh;
	image_transport::ImageTransport _it;
	image_transport::Subscriber _image_sub;
	image_transport::Subscriber _depth_sub;
	cv::Mat _myMat;
	cv::Mat _matConverted;
	cv::Mat _matGrayColor;
	cv::VideoWriter _image_writer;
	cv::VideoWriter _depth_writer;
	std::string _extension;
	std::string _topic_image;
	std::string _topic_depth;
	std::string _image_video;
	std::string _depth_video;
	void imageCallback(const sensor_msgs::Image::ConstPtr& image_ptr);
	void depthCallback(const sensor_msgs::Image::ConstPtr& depth_ptr);

public:
	VideoAcquisitor();
	string  _numbersubject;	
	string  _numberimage;
	virtual ~VideoAcquisitor();
};

#endif
	

