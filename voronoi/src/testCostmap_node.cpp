#include "testCostmap.h"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"global_planning");

	tf::TransformListener tf(ros::Duration(10));
	costmap_2d::Costmap2DROS lcr("costmap",tf);
	global_planner::testCostmap costmap("my_costap",&lcr);
	ros::spin();
	return 0;
}
