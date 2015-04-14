#ifndef VORONOI_PLANNER_H
#define VORONOI_PLANNER_H


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


#define COST_UNKNOWN_ROS 255 // 255 = Unknown cost
#define COST_OBS 254 // 254 = forbidden region
#define COST_OBS_ROS 253 // ROS values of 253 are obstacles

#define COST_NEUTRAL 50 // Open space value
#define COST_FACTOR 0.8 // Might be used to translate costs 

#ifndef COSTTYPE
#define COSTTYPE unsigned char
#endif

namespace voronoi_planner
{

class VoronoiPlanner : public nav_core::BaseGlobalPlanner
{
public:

	VoronoiPlanner();
	VoronoiPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	
	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	
	bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

	
	void setNavArray(); // Set arrays for path planning
	void setCostmap (const COSTTYPE* cmap, bool isROS, bool allow_unknown); // Set costmap array
	void computeBrushfire(const COSTTYPE* cmap, int* distance_transform); // Compute distance transform
	void mapThresholding(const COSTTYPE* cmap, int* costarr_thresh); // Compute the binary map
	void distanceInit(int* distance_transform);

private:

	ros::NodeHandle private_nh_; // Node handle
	ros::Publisher plan_pub_; //Used to publish plan to goal
	ros::Subscriber pose_sub_; //Pose subscriber
	ros::ServiceServer make_plan_service_; // Service to make plan
	costmap_2d::Costmap2DROS* costmap_ros_; //Costmap wrapper for ROS
	costmap_2d::Costmap2D* costmap_; // Costmap class
	
	bool initialized_;

	/* cell arrays */
	COSTTYPE *costarr_; //Cost array
	std::vector<int> priorityqueue_; // Priority queue
	int* distance_transform_; // Distance transform
	int *costarr_thresh_;

	/* Parameters */
	int xs_; // Size of Costmap in X
	int ys_; // Size of costmap in Y
	int total_size_; // Total size X*Y


};

};
#endif	
