#ifndef TESTCOSTMAP_H_
#define TESTCOSTMAP_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <cv.h>
#include <highgui.h>
#include <sstream>
#include <cvaux.h>
#include <cxmisc.h>

#include<voronoi/MakemyNavPlan.h>

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
using namespace std;
using namespace cv;

namespace global_planner{

class testCostmap : public nav_core::BaseGlobalPlanner
{
public:
	testCostmap();
	testCostmap(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	~testCostmap();



	void initialize(std::string name,costmap_2d::Costmap2DROS* costmap_ros);
	bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>&plan);
	bool makePlanService(voronoi::MakemyNavPlan::Request& req, voronoi::MakemyNavPlan::Response& resp);
	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & goal);



	void setNavArray(); // Memory allocation for tables that can be directly allocated
	void setCostmap( COSTTYPE* cmap,bool isROS, bool allow_unknown); // Prepare costmap

	void dijkstraPath(int s); // Dijkstra computation on skeleton

	void computeBrushfire(); // Compute distance transform
	void mapThresholding(); // Compute the binary map
	void distanceInit(); //Initialize distance transfrom (obstacles : -1 others : 0)
	void distanceFilling(); // Create the distance transform
	void findNeighbours(int ind,int N);  // Find the neighbours of a pixel in the costmap (4 connected)
 	std::vector<int> neighbours_; // Neighbours found	
	void queueInit(); // Initialize queue for distance transform
	void computeGrad(); // Compute gradient and module of gradient over the map
	void orderModule(); // Find the module minimum to obtain skeleton
	void computeGraph(); // Compute graph (order the pixels of the skeleton)
	int computeClosest(int goal); //Compute closest point of the Voronoi diagram to "goal"
	void computePathVoro(); // Compute path on Voronoi diagram
	void computePathWorld(std::vector<geometry_msgs::PoseStamped>& path); // Compute path on Voronoi diagram in map coordinates
	void mapToWorld(double mx, double my, double & wx, double& wy);
	void test(double k,double z);
private:

	ros::NodeHandle private_nh_; // Node handle
	ros::Subscriber pose_sub_; // Subscriber to get goal
	ros::ServiceServer make_plan_srv_; // Service advertiser
	ros::Publisher plan_pub_;
	costmap_2d::Costmap2DROS* costmap_ros_; //Costmap wrapper for ROS
	costmap_2d::Costmap2D* costmap_; // Costmap class
	
	int startVoro_; // Starting point of path in Voronoi skeleton
	int goalVoro_; // Goal point of path in Voronoi skeleton
	std::vector<int> distance_;
	std::vector<int> predecessors_;
	std::vector<int> pathcm_;
	std::vector<geometry_msgs::PoseStamped> pathWorld_;
	/* cell arrays */
	COSTTYPE *costarr_; //Cost array
	std::vector<int> priorityqueue_; // Priority queue
	std::vector<int> skeleton_;
	std::vector<int> skel_ordered_;
	vector<Point>  skelcv_;
	vector<int> skelcostmap_;
	std::vector<std::vector<std::pair<int,int> > > graph_;
	int* distance_transform_; // Distance transform
	int startcm_;
	int goalcm_;
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
};
#endif
