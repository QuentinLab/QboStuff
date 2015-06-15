#include "acquisition.h"

VideoAcquisitor::VideoAcquisitor() : _it(_private_nh)
{
	ROS_INFO("Starting Acquisition...");
	onInit();
}

VideoAcquisitor::~VideoAcquisitor()
{
	ROS_INFO("End acqusition...");
}

void VideoAcquisitor::onInit()
{
	std::cout << "Sujet ?" << std::endl;
	std::cin >> _numbersubject;
	std::cout << "Passage ?" << std::endl;
	std::cin >> _numberimage;
	_topic_image = "/camera/rgb/image_raw";
	_topic_depth = "/camera/depth/image";
//	_image_video = "/home/qbobot/Documents/videos/sujet" + _numbersubject + "image" + _numberimage + ".avi";
//	_depth_video = "/home/qbobot/Documents/videos/sujet" + _numbersubject + "depth" + _numberimage + ".avi";
	_image_video = "/home/qbobot/Documents/videos/bin/binimage.avi";
	_depth_video = "/home/qbobot/Documents/videos/bin/bindepth.avi";
	_image_writer.open(_image_video,CV_FOURCC('M','J','P','G'),31,Size(640,480),true);
	_depth_writer.open(_depth_video,CV_FOURCC('M','J','P','G'),30,Size(640,480),true);
	_image_sub = _it.subscribe(_topic_image,10,&VideoAcquisitor::imageCallback,this);
	_depth_sub = _it.subscribe(_topic_depth,10,&VideoAcquisitor::depthCallback,this);
}

void VideoAcquisitor::imageCallback(const sensor_msgs::Image::ConstPtr& image_ptr)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(image_ptr, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	_image_writer.write(cv_ptr->image);
	imshow("RGB",cv_ptr->image);
	waitKey(1);
}

void VideoAcquisitor::depthCallback(const sensor_msgs::Image::ConstPtr& depth_ptr)
{
	cv_bridge::CvImagePtr cv_ptr2;
	try
	{
		cv_ptr2 = cv_bridge::toCvCopy(depth_ptr);
		_myMat = cv_ptr2->image.clone();
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	double minVal,maxVal;
	minMaxLoc(_myMat, &minVal, &maxVal); 
    	_myMat.convertTo(_matConverted, CV_8UC3, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

	minMaxLoc(_matConverted,&minVal,&maxVal);

	cvtColor(_matConverted,_matGrayColor,CV_GRAY2RGB);

	_depth_writer.write(_matGrayColor);
	imshow("Depth",_matGrayColor);
	waitKey(1);
}
	
int main (int argc, char** argv)
{
	ros::init(argc,argv,"acquisitor");
	VideoAcquisitor myacquisitor;
	ros::spin();
	return 0;		
}
