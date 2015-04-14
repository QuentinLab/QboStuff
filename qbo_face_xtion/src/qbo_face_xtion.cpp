#include "qbo_face_xtion.h"

FaceDetector::FaceDetector()
{
	ROS_INFO("Beginning face detection");
	OnInit();
}

FaceDetector::~FaceDetector()
{
	ROS_INFO("Done");
}

void FaceDetector::OnInit()
{
	face_classifier_path_ = "/home/qbobot/Documents/cascades/haarcascade_frontalface_alt2.xml";
	topic_image_ = "/camera/rgb/image_raw";


	/* Loading cascade */

	if (!face_cascade_.load(face_classifier_path_))
	{
		ROS_INFO("Sorry, could not load face cascade classifier");
		return;
	}
	faces_pub_ = private_nh_.advertise<qbo_face_xtion::FacesPos>("/qbo/facesdetected",1,this);
	image_sub_ = private_nh_.subscribe(topic_image_,1,&FaceDetector::imageCallback,this);
	
}

void FaceDetector::imageCallback(const sensor_msgs::Image::ConstPtr& image_ptr)
{
	try
	{
		cv_ptr_ = cv_bridge::toCvCopy(image_ptr,sensor_msgs::image_encodings::RGB8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception : %s", e.what());
		return;
	}

	smallImg_.create(cvRound(cv_ptr_->image.rows/1),cvRound(cv_ptr_->image.cols/1),CV_8UC1);
	cvtColor(cv_ptr_->image,gray_,CV_RGB2GRAY);
	resize(gray_,smallImg_,smallImg_.size(),0,0,INTER_LINEAR);
	cv::equalizeHist(smallImg_,smallImg_);
	face_cascade_.detectMultiScale(smallImg_,faces_,1.1,2,0|CV_HAAR_SCALE_IMAGE,Size(30,30));

	if (!faces_.empty())
	{
		for (vector<Rect>::const_iterator r = faces_.begin(); r!=faces_.end();r++)
		{
			oneFace_.x = r->x+r->width;
			oneFace_.y = r->y+r->height;
			oneFace_.z = 0;
			faces_positions_.points.push_back(oneFace_);
			faces_positions_.image_width = cv_ptr_->image.cols;
			faces_positions_.image_height = cv_ptr_->image.rows;
		}
	}
	else
	{
		oneFace_.x = -1000;
		oneFace_.y = -1000;
		oneFace_.z = -1000;
		faces_positions_.points.push_back(oneFace_);
	}
	faces_pub_.publish(faces_positions_);
	faces_positions_.points.clear();
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"qbo_face_xtion");
	FaceDetector facedetector;
	ros::spin();
	return 0;
}
