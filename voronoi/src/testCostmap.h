#ifndef TESTCOSTMAP_H_
#define TESTCOSTMAP_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <cv.h>
#include <highgui.h>
#include <sstream>
#include <cvaux.h>
#include <cxmisc.h>
#define COST_UNKNOWN_ROS -1 // 255 = Unknown cost
#define COST_OBS 254 // 254 = forbidden region
#define COST_OBS_ROS 253 // ROS values of 253 are obstacles

#define COST_NEUTRAL 50 // Open space value
#define COST_FACTOR 0.8 // Might be used to translate costs 

#define THRESH_OCC 1
#define THRESH_FREE 0

#ifndef COSTTYPE
#define COSTTYPE unsigned char
#endif

using namespace cv;
class testCostmap
{
public:
	testCostmap();
	testCostmap(costmap_2d::Costmap2DROS* costmap_ros);
	~testCostmap();
	void onInit(costmap_2d::Costmap2DROS* costmap_ros);
	void setNavArray();
	void setCostmap( COSTTYPE* cmap,bool isROS, bool allow_unknown); 
	void computeBrushfire(); // Compute distance transform
	void mapThresholding(); // Compute the binary map
	void distanceInit();
	void distanceFilling();
	void findNeighbours(int ind);
 	std::vector<int> neighbours_;	
	void queueInit();
	void computeGrad();
	void orderModule();
private:

	ros::NodeHandle private_nh_; // Node handle
	costmap_2d::Costmap2DROS* costmap_ros_; //Costmap wrapper for ROS
	costmap_2d::Costmap2D* costmap_; // Costmap class
	

	/* cell arrays */
	COSTTYPE *costarr_; //Cost array
	std::vector<int> priorityqueue_; // Priority queue
	int* distance_transform_; // Distance transform
	int *costarr_thresh_;
	int* checking_;
	int* gradx_;
	int* grady_;
	double* module_;
	/* Parameters */
	int xs_; // Size of Costmap in X
	int ys_; // Size of costmap in Y
	int total_size_; // Total size X*Y
};

#endif
