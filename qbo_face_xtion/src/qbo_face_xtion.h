/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2012 Thecorpora, S.L.
 *
 * This program is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
 * MA 02110-1301, USA.
 *
 * Author: Arturo Bajuelos <arturo@openqbo.com>
 */

#ifndef FACEDETECTOR_H_
#define FACEDETECTOR_H_

#include <iostream>
#include <vector>
#include <cmath>
#include <string>

#include <cv.h>
#include <highgui.h>
#include <sstream>
#include <cvaux.h>
#include <cxmisc.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/SetCameraInfo.h>

//#include <qbo_face_detection/FacePosAndDist.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
//#include <qbo_face_msgs/RecognizeFace.h>
//#include <qbo_face_msgs/GetName.h>
//#include <qbo_arduqbo/Nose.h>
#include <qbo_face_xtion/FacesPos.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt8.h>

#define HEAD_SIZE 0.20


using namespace std;
using namespace cv;

class FaceDetector
{
private:
	void OnInit();
	/*
	 * ROS Parameters
	 */
	string face_classifier_path_;
	string topic_image_;
	cv::CascadeClassifier face_cascade_;
	std::vector<cv::Rect> faces_;
	cv_bridge::CvImagePtr cv_ptr_;
	cv::Mat gray_;
	cv::Mat smallImg_;
	/*
	 * ROS Elements
	 */
	ros::NodeHandle private_nh_;

	ros::Subscriber image_sub_;
	ros::Publisher faces_pub_;
	qbo_face_xtion::FacesPos faces_positions_;
	geometry_msgs::Point oneFace_;



	std_msgs::UInt8 face_found_;
	ros::Publisher face_found_pub_;
	/*
	 * Elements for the face recognition client
	 */
	/*ros::ServiceClient client_recognize_;
	qbo_face_msgs::RecognizeFace srv;

	ros::ServiceClient client_get_name;
	qbo_face_msgs::GetName srv_get_name;*/

	/*
	 * Internal parameters
	 */


	cv::CascadeClassifier face_classifier_;
	//Regarding FaceRecognizer


	/*
	 * Methods
	 */

	//ROS Callbacks
	void imageCallback(const sensor_msgs::Image::ConstPtr& image_ptr);


	//Recognizer methods





public:
	FaceDetector();

	virtual ~FaceDetector();
};

#endif /* FACEDETECTOR_H_ */
