#include <voronoi/voronoi.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(voronoi_planner::VoronoiPlanner, nav_core::BaseGlobalPlanner)
using namespace std;
namespace rm = geometry_msgs;

namespace voronoi_planner
{

VoronoiPlanner::VoronoiPlanner(): costmap_ros_(NULL), initialized_(false)
{
}

VoronoiPlanner::VoronoiPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL),initialized_(false), allow_unknown_(true)
{
	initialize(name,costmap_ros);
}

void VoronoiPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	if (!initialized_)
	{
		costmap_ros_ = costmap_ros;
		costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
		plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan",1);

		/* Initialize tables for priority and costarray */
		costarr_ = NULL;

		/* Get size of costmap */
		xs_ = costmap->getSizeInCellsX();
		ys_ = costmap->getSizeInCellsY();
		total_size_ =xs_ * ys_;

		setNavArr();
		setCostmap(costmap->getCharMap(),true,allow_unknown_);
		
		pose_sub_ = private_nh_.subscribe<rm::PoseStamped>("goal",1,&VoronoiPlanner::poseCallback,this);
	}
	else
	{
		ROS_WARN("The Voronoi planner was already initialized somewhere, can't intialize anymore");
	}
	initialized_ = true;
}

void VoronoiPlanner::poseCallback(const rm::PoseStamped::ConstrPtr& goal)
{
	tf::Stamped<tf::Pose> global_pose;
	costmap_ros_->getRobotPose(global_pose);
	vector<PoseStamped> path;
	rm::PoseStamped start;
	start.header.frame_id = global_pose.frame_id_;
	start.pose.position.x = global_pose.getOrigin().x();
	start.pose.position.y = global_pose.getOrigin().y();
	start.pose.position.z = global_pose.getOrigin().z();
	start.pose.orientation.x = global_pose.getRotation().x();
	start.pose.orientation.y = global_pose.getRotation().y();
	start.pose.orientation.z = global_pose.getRotation().w();
	makePlan(start, *goal, path)
}

void VoronoiPlanner::setNavArr()
{
	ROS_DEBUG("[VoroPlanner] Setting Navigation Arrays of size %d x %d", xs_,ys_);
	//Creation of arrays
	
	if (costarr_)
	{	
		delete[] costarr_;
	}
	if (distance_transform_)
	{
		delete[] distance_transform_;
	}
	if (costarr_thresh_)
	{
		delete[] costarr_thresh_;
	}
	

	costarr_ = new COSTTYPE[total_size_];
	priorityqueue_.clear();
	distance_transform_ = new int[total_size_];
	costarr_thresh_ = new int[total_size_];	

}

void VoronoiPlanner::setCostmap(const COSTTYPE *cmap, bool isROS, bool allow_unknown)
{

	COSTTYPE* cm = costarr_;

	if (isROS)
	{
		for (int i=0; i<ys_;i++)
		{
			int k=i*xs_;
			for (int j=0;j<xs_; j++, k++, cmap++, cm++)
			{
				*cm = COST_OBS;
				int v = *cmap;
				if (v < COST_OBS_ROS)
				{
					v = COST_NEUTRAL + COST_FACTOR*v;
					if (v >= COST_OBS)
					{
						v = COST_OBS-1;
					}
					*cm = v;
				}
				else if (v== COST_UNKNOWN_ROS && allow_unknown)
				{
					v = COST_OBS - 1;
					*cm = v;
				}
			
			}
		}
	}
	else
	{
		for (int i=0;i<ys_;i++)
		{
			int k=i*xs_;
			for (int j=0; j<xs_;j++,j++,cmap++,cm++)
			{
				*cm = COST_OBS;
				if (i<7 || i > ys_ -8 || j < 7 || j > xs_ - 8)
				{
					continue;
				}
				int v = *cmap;
				if (v < COST_OBS_ROS)
				{
					v = COST_NEUTRAL + COST_FACTOR*v;
					if (v >= COST_OBS)
					{
						v = COST_NEUTRAL + COST_FACTOR*v;
						if (v >= COST_OBS)
						{
							v = COST_OBS-1;
						}
						*cm = v;
					}
				}
				else if (v == COST_UNKNOWN_ROS)
				{
					v = COST_OBS - 1;
					*cm = v;
				}
			}
		}
	}
}

void VoronoiPlanner::ComputeBrushfire(const COSTTYPE* cmap, int* distance_transform)
{
	mapThresholding(cmap,costarr_thresh_);
	distanceInit(distance_transform);
}

void VoronoiPlanner::mapThresholding(const COSTTYPE* cmap, int* costarr_thresh)
{
	int i;
	int* it = costarr_thresh;
	COSTTYPE* cm_it = cmap;
	for (i=0;i<total_size_;i++,it++,cm_it++)
	{
		int v = *cm_it;
		if (v > COST_NEUTRAL)
		{
			*it = 1;
		}
		else
		{
			*it = 0;
		}
	}
}		

void VoronoiPlanner::distanceInit(int * distance_transform);
{
	int i;
	for (i=0;i<total_size_;i++,distance_transform)
	{
		*distance_transform = 0;
		
	}
}
	


void VoronoiPlanner::queueInit(int costarr_thresh* cmap)
{
	int* cm = costarr_thresh;
	for (int i=0;i<total_size_;i++)
	{
		if (*cm == THRESH_OCC)
		{
			priorityqueue_.push_back(i);
		}
	}
}
























};
