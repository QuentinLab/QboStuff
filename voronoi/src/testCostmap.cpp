#include "testCostmap.h" 
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(global_planner::testCostmap, nav_core::BaseGlobalPlanner)

using namespace std;

namespace global_planner{

testCostmap::testCostmap()
{
}

testCostmap::testCostmap(std::string name,costmap_2d::Costmap2DROS* costmap_ros)
{
	initialize(name,costmap_ros);
}

testCostmap::~testCostmap()
{
	ROS_INFO("The end");
}

void testCostmap::initialize(std::string name,costmap_2d::Costmap2DROS* costmap_ros)
{
	costmap_ros_ = costmap_ros;	
	ROS_INFO("Alright let's go");
	costmap_ = costmap_ros_->getCostmap();
	std::string frame_id = costmap_ros->getGlobalFrameID();
	std::cout << "global frame id" << frame_id << std::endl;
	xs_ = costmap_ -> getSizeInCellsX();
	ys_ = costmap_ -> getSizeInCellsY();
	total_size_ = xs_ * ys_;
	printf("xs_ = %d\nys_ = %d\ntotal size = %d\n",xs_,ys_,total_size_);


	setNavArray();
	setCostmap(costmap_->getCharMap(),true,true);
	computeBrushfire();
	make_plan_srv_ = private_nh_.advertiseService("make_plan",&testCostmap::makePlanService,this);
	private_nh_.subscribe<geometry_msgs::PoseStamped>("goal",1,&testCostmap::poseCallback,this);
}

void testCostmap::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & goal)
{
	tf::Stamped<tf::Pose> global_pose;
	costmap_ros_->getRobotPose(global_pose);
	vector<geometry_msgs::PoseStamped> path;
	geometry_msgs::PoseStamped start;
	start.header.stamp = global_pose.stamp_;
	start.header.frame_id = global_pose.frame_id_;
	start.pose.position.x = global_pose.getOrigin().x();
	start.pose.position.y = global_pose.getOrigin().y();
	start.pose.position.z = global_pose.getOrigin().z();
	start.pose.orientation.x = global_pose.getRotation().x();
	start.pose.orientation.y = global_pose.getRotation().y();
	start.pose.orientation.z = global_pose.getRotation().z();
	start.pose.orientation.w = global_pose.getRotation().w();
	makePlan(start, *goal, path);

}


bool testCostmap::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan )
{
	unsigned int mx,my;
	if (!costmap_->worldToMap(start.pose.position.x,start.pose.position.y,mx,my))
	{
		ROS_WARN("The robot's position is off the mapp... cannot process.");
		return  false;
	}
	else
	{
		startcm_ = mx + my*xs_;
	}

	if (!costmap_->worldToMap(goal.pose.position.x,goal.pose.position.y,mx,my))
	{
		ROS_WARN("The goal is off the map... cannot process");
		return false;
	}
	else
	{
		goalcm_ = mx + my*xs_;
	}
	startVoro_ = computeClosest(startcm_);
	goalVoro_ = computeClosest(goalcm_);
	dijkstraPath(startVoro_);
	computePathVoro();
	computePathWorld(plan);
	/*plan.push_back(start);
	for (int i=0; i<20; i++)
	{
		geometry_msgs::PoseStamped new_goal = goal;
		tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

		new_goal.pose.position.x = -2.5+(0.05*i);
		new_goal.pose.position.y = -3.5+(0.05*i);

		new_goal.pose.orientation.x = goal_quat.x();
		new_goal.pose.orientation.y = goal_quat.y();
		new_goal.pose.orientation.z = goal_quat.z();
		new_goal.pose.orientation.w = goal_quat.w();

		plan.push_back(new_goal);
	}   
	plan.push_back(goal);*/
	return true; 
}

bool testCostmap::makePlanService(nav_msgs::GetPlan::Request & req, nav_msgs::GetPlan::Response& resp)
{
	bool mybool = makePlan(req.start,req.goal,resp.plan.poses);
	resp.plan.header.stamp = ros::Time::now();
	resp.plan.header.frame_id = costmap_ros_ -> getGlobalFrameID();
	return mybool;
}

void testCostmap::computePathVoro()
{
	pathcm_ = std::vector<int>(distance_[goalVoro_]);
	int cur = predecessors_[goalVoro_];
	int next,i;
	pathcm_[0] = startVoro_;
	pathcm_[distance_[goalVoro_]] = goalVoro_;
	for (i=0;i<distance_[goalVoro_]-1;i++)
	{
		pathcm_[distance_[goalVoro_]-i-2] = cur;
		next = predecessors_[cur];
		cur = next;
			 
	}
	
}

void testCostmap::computePathWorld(std::vector<geometry_msgs::PoseStamped>& path)
{
	double x,y;
	geometry_msgs::PoseStamped pose;
	ros::Time plan_time = ros::Time::now();
	for (std::vector<int>::iterator it=pathcm_.begin(); it!=pathcm_.end();it++)
	{
		mapToWorld((double)(*it%xs_),(double)(*it/xs_),x,y);
		pose.header.stamp = plan_time;
		pose.header.frame_id = "map";
		pose.pose.position.x = x;
		pose.pose.position.y = y;
		pose.pose.position.z = 0.0;

		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;
		path.push_back(pose);
	}

}


void testCostmap::mapToWorld(double mx, double my, double& wx, double& wy) 
{
	wx = costmap_->getOriginX() + mx * costmap_->getResolution();
	wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void testCostmap::dijkstraPath(int s)
{
	distance_ = std::vector<int>(total_size_,100000000);
	predecessors_ = std::vector<int>(total_size_,0);	
	set<pair<int,int> > Q;
	distance_[s] = 0;
	Q.insert(std::pair<int,int>(0,s));

	while(!Q.empty())
	{
		std::pair<int,int> top = *Q.begin();
		Q.erase(Q.begin());
		int v = top.second;
		int d = top.first;

		for (std::vector<pair<int,int> >::const_iterator it = graph_[v].begin(); it != graph_[v].end(); it++)
		{
			int v2 = it->first;
			int cost = it->second;
			if (distance_[v2] > distance_[v] + cost)
			{
				if (distance_[v2] != 1000000000)
				{
					Q.erase(Q.find(std::pair<int,int>(distance_[v2], v2)));
				}
				distance_[v2] = distance_[v] + cost;
				Q.insert(std::pair<int,int>(distance_[v2], v2));
				predecessors_[v2] = v;
		    	}
		}
	}
}

int testCostmap::computeClosest(int goal)
{
	std::vector<int>::iterator skelit;
	int goalx = goal%xs_;
	int goaly = goal/xs_;
	int skelx,skely;
	int indmin=0;
	int indcount=0;
	double min = 999999;
	double cur = 0;
	for (skelit = skelcostmap_.begin(); skelit != skelcostmap_.end(); skelit++,indcount++)
	{
		// Computing euclidean distance to goal
		skelx = *skelit%xs_;
		skely = *skelit/xs_;
		cur =  sqrt(pow(goalx-skelx,2) + pow(goaly-skely,2));
		if (min > cur)
		{
			min = cur;
			indmin = indcount;
		}	
	}
	return indmin;
		
}	

void testCostmap::setNavArray()
{
	ROS_DEBUG("[VoroPlanner] Setting Navigation Arrays of size %d x %d", xs_,ys_);
	//Creation of arrays
	
	
	costarr_ = new COSTTYPE[total_size_];
	priorityqueue_.clear();
	distance_transform_ = new int[total_size_];
	costarr_thresh_ = new int[total_size_];	
	checking_ = new int[total_size_];
	for (int i = 0; i < total_size_;i++)
	{
		checking_[i] = 0;
	}
	gradx_ = new int[total_size_];
	grady_ = new int[total_size_];
	module_ = new double[total_size_];
	
	for (int i=0;i<total_size_;i++)
	{
		module_[i] = -1;
	}

}
void testCostmap::setCostmap( COSTTYPE *cmap, bool isROS, bool allow_unknown)
{

	COSTTYPE* cm = costarr_;
	COSTTYPE* cm2 = cmap;
	int count_unknown = 0;
	for (int bibu = 0; bibu < total_size_;bibu++)
	{
		if (cmap[bibu] == 254)
		{
			count_unknown++;
		}
	}
	printf("Number of unkown = %d\n",count_unknown);

	if (isROS)
	{
		for (int i=0; i<ys_;i++)
		{
			int k=i*xs_;
			for (int j=0;j<xs_; j++, k++, cm2++, cm++)
			{
				*cm = COST_OBS;
				int v = *cm2;
				if (v < COST_OBS_ROS)
				{
			//		printf("Free ! i = %d k = %d\n",i,k);
					v = COST_NEUTRAL + COST_FACTOR*v;
					if (v >= COST_OBS)
					{
						v = COST_OBS-1;
					}
					*cm = v;
				}
				else if (v== COST_UNKNOWN_ROS && allow_unknown)
			//	else
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

void testCostmap::computeBrushfire()
{
	mapThresholding();
	distanceInit();
	queueInit();
	distanceFilling();
	computeGrad();
	orderModule();
	computeGraph();

	ROS_INFO("Going to try to print image");
	int max = 0;
	Mat distMat(xs_,ys_,CV_8U);
	Mat rays(xs_,ys_,CV_8U);
	int * it = distance_transform_;
	for (int j = 0; j < total_size_; j++,it++)
	{
		if (*it > max)
		{
			max = *it;
		}
	}
	printf("MAX= %d\n", max);
	it = distance_transform_;
	int i;
	for ( i = 0;i<total_size_;i++,it++)
	{
		if (*it == -1 || *it == 0)
		{
			distMat.at<unsigned char>((int)i%xs_,(int)i/xs_) = 255;
		}
		else
		{
			distMat.at<unsigned char>((int)i%xs_,(int)i/xs_) =(unsigned char) (((float)*it/(float)max)*255.);
			//distMat.at<unsigned char>((int)i%xs_,(int)i/xs_) = 1;
		}
		if (*it == -1 || *it == 0)
		{
			rays.at<unsigned char>((int)i%xs_,(int)i/xs_) = 255;
		}
		else if (*it %2 == 0)
		{
			rays.at<unsigned char>(i%xs_,i/xs_) = 140;
		}
		else
		{	
			rays.at<unsigned char>(i%xs_,i/xs_) = 1;
		}
	}

	Mat skeleton(xs_,ys_,CV_8U);
	it = distance_transform_;
	for ( i = 0;i<total_size_;i++,it++)
	{
		if (*it == -1 || *it == 0)
		{
			skeleton.at<unsigned char>((int)i%xs_,(int)i/xs_) = 255;
		}
		else if (*it > 10)
		{
			skeleton.at<unsigned char>(i%xs_,i/xs_) = 140;
		}
		else
		{
			skeleton.at<unsigned char>((int)i%xs_,(int)i/xs_) = 1;
		}

	}		
			
	
	imwrite("/home/qbobot/Documents/Images_brushfire/rays.jpg", rays);
	imwrite("/home/qbobot/Documents/Images_brushfire/image.jpg",distMat);
	imwrite("/home/qbobot/Documents/Images_brushfire/skeleton.jpg",skeleton);
	waitKey(2);
	

	/* Test space */

	/* find contour in the binary image */

	Mat img(xs_,ys_,CV_8U);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	for (i = 0; i<total_size_;i++)
	{
		img.at<unsigned char>(i%xs_,i/xs_) = 0;
	}
	for (std::vector<int>::iterator sk = skeleton_.begin();sk!=skeleton_.end();sk++)
	{
		img.at<unsigned char>(*sk%xs_,*sk/xs_) = 255;
	}

	imwrite("/home/qbobot/Documents/Images_brushfire/playground.jpg",img);
	
	Mat element = getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
	Mat eroded;
	Mat dilated;
	erode(img,eroded,element);
	dilate(img,dilated,element);


	imwrite("/home/qbobot/Documents/Images_brushfire/eroded.jpg",eroded);
	imwrite("/home/qbobot/Documents/Images_brushfire/dilated.jpg",dilated);

	findContours(img,contours,hierarchy, CV_RETR_CCOMP,CV_CHAIN_APPROX_NONE);
	Mat allContours(xs_,ys_,CV_8U);
	Mat dst = Mat::zeros(img.rows,img.cols,CV_8U);

    /*	int idx = 0;
    	for( ; idx >= 0; idx = hierarchy[idx][0] )
    	{
        	Scalar color( rand()&255, rand()&255, rand()&255 );
        	drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
    	}

	imwrite("/home/qbobot/Documents/Images_brushfire/contourscolor.jpg",dst);*/
	max = 0;
	int indmax = 0;
	int indp = 0;
	int numberOfContours = contours.size();
	int skeletonsize = skeleton_.size();
	
	for (vector<vector<Point> >::iterator cont = contours.begin();cont != contours.end();cont++,indp++)
	{	
		if (cont->size() > max)	
		{
			max = cont->size();
			indmax = indp;
			skelcv_ = *cont;
		}
	}
	for (vector<Point>::iterator skelit = skelcv_.begin(); skelit != skelcv_.end(); skelit++)
	{
		dst.at<unsigned char>(*skelit) = 255;
		skelcostmap_.push_back(skelit->x+skelit->y*xs_);
	}

	it = distance_transform_;
	for (i = 0; i< total_size_ ; i++,it++)
	{
		if (*it == -1 || *it == 0)
		{
			dst.at<unsigned char>(i%xs_,i/xs_) = 140;
		}
	}

	int inter = 0;

	imwrite("/home/qbobot/Documents/Images_brushfire/contourscolor.jpg",dst);
	printf("Number of contours : %d\n",numberOfContours);
	printf("Biggest contour size : %d\n",max);
	printf("Total size of skeleton : %d\n",skeletonsize);
	getchar();
	exit(0);
}
void testCostmap::orderModule()
{
	double* order = new double[total_size_];	
	int* order_ind = new int[total_size_];
	int p,h;
	for ( p = 0; p < total_size_; p++)
	{
		order[p] = module_[p];
	}
	int max_inter,a,b,ind;
	for (p = 0; p < 1500;p++)
	{
		max_inter = 65000;
		ind = 0;
		for (h = 0; h <total_size_ - p; h++)
		{
			if (max_inter > order[h] && order[h] != -1)
			{	
				max_inter = order[h];
				ind = h;
			}
		}
		a = order[ind];
		order[ind] = order[total_size_ - p - 1];
		order[total_size_ - p - 1] = a;
		order_ind[p] = ind;
	}
	
	// Tried using the function std::sort to sort the module... did not work

	/*for (p=0;p<total_size_;p++)
	{
		order_ind[p] = p;
	}

	printf("Gonna sort...");
	getchar()	std::sort(order_ind,order_ind+total_size_, [&](size_t a,size_t b){return order[a] < order[b];});
	printf("Order[total_size_ - 200] = %f\n",order[total_size_-200]);
	for (p = total_size_-20;p<total_size_;p++)
	{
		printf("Order[%d] = %f\n",p,order[p]);
	}
	exit(0);*/

	/* Image of the minimum of gradient */
	ROS_INFO("COMPUTING SKELETON FILE");

	int i,count_skel;
	int* it = distance_transform_;
	Mat res(xs_,ys_,CV_8U);
	for (i=0; i< total_size_;i++,it++)
	{
		if (*it==-1 || *it == 0)
		{
			res.at<unsigned char>(i%xs_,i/xs_) = 255;
		}
		else
		{
			res.at<unsigned char>(i%xs_,i/xs_) = 1;
		}
	}
	count_skel = 0;
	for (i=0; i<2000; i++)
	{
		if (distance_transform_[order_ind[i]] >= 6)
		{
			res.at<unsigned char>(order_ind[i]%xs_,order_ind[i]/xs_) = 140;
			skeleton_.push_back(order_ind[i]);
			count_skel++;
		}
	}
	
	
	printf("Numbers of pixels inside the skeleton : %d\n",count_skel);
	imwrite("/home/qbobot/Documents/Images_brushfire/skel.jpg",res);	
}

void testCostmap::mapThresholding()
{
	int memo;
	int i;
	int* it = costarr_thresh_;
	cv::Mat myMat(xs_,ys_,CV_8U);
	COSTTYPE* cm_it = costarr_;
	for (i=0;i<total_size_;i++,it++,cm_it++)
	{
		int v = *cm_it;
		if (v > COST_NEUTRAL)
		{
			*it = 1;
			memo = i;
			//printf("OBSTACLE : position [%dx%d]\n",(int) i%xs_/2, (int) i/xs_/2);
			myMat.at<unsigned char>((int)i%xs_,(int)i/xs_) = 0;
		}
		else
		{
			//printf("FREE ZONE : position [%dx%d] !\n",(int)i%xs_/2,(int) i/xs_/2);
			*it = 0;
			myMat.at<unsigned char>((int)i%xs_,(int)i/xs_) = 255;
		}
	}
	imwrite("/home/qbobot/Documents/Images_brushfire/thresholded_map.jpg",myMat);


	/* Test for distanceTransform from opencv*/

	Mat distance_transform(xs_,ys_,CV_32FC1);
	Mat voronoi_diagram(xs_,ys_,CV_32SC1);
	distanceTransform(myMat,distance_transform,voronoi_diagram,CV_DIST_L2,5,DIST_LABEL_CCOMP);
	double max,min;

	minMaxLoc(distance_transform,&min,&max,NULL,NULL);

	Mat distance_transform_norm = distance_transform/max*255;

	minMaxLoc(voronoi_diagram,&min,&max,NULL,NULL);
	
	Mat voronoi_diagram_norm = voronoi_diagram/max*255;


	// Test for morphological thinning 

	cv::Mat skel(myMat.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp(myMat.size(), CV_8UC1);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	bool done;
	do
	{
	  cv::morphologyEx(myMat, temp, cv::MORPH_OPEN, element);
	  cv::bitwise_not(temp, temp);
	  cv::bitwise_and(myMat, temp, temp);
	  cv::bitwise_or(skel, temp, skel);
	  cv::erode(myMat, myMat, element);
	 
	  double max;
	  cv::minMaxLoc(myMat, 0, &max);
	  done = (max == 0);
	} while (!done);


	imwrite("/home/qbobot/Documents/Images_brushfire/distance_transform.jpg",distance_transform);
	imwrite("/home/qbobot/Documents/Images_brushfire/voronoi_diagram.jpg",voronoi_diagram);
	imwrite("/home/qbobot/Documents/Images_brushfire/distance_transform_norm.jpg",distance_transform_norm);
	imwrite("/home/qbobot/Documents/Images_brushfire/voronoi_diagram_norm.jpg",voronoi_diagram_norm);
	imwrite("/home/qbobot/Documents/Images_brushfire/morphomat.jpg",skel);


}		

void testCostmap::distanceInit()
{
	ROS_INFO("Initializing distance table");
	int i;
	int count_obs = 0;
	int* it_dist = distance_transform_;
	int* cm = costarr_thresh_;
	Mat myMat(xs_,ys_,CV_8U);
	for (i=0;i<total_size_;i++,it_dist++,cm++)
	{
		if (*cm == THRESH_OCC)
		{
			*it_dist = -1;
			count_obs ++;
			myMat.at<unsigned char>((int)i%xs_,(int)i/xs_) = 250;
		}
		else
		{
			*it_dist = 0;
			myMat.at<unsigned char>((int)i%xs_,(int)i/xs_) = 1;
		}
		
	}
	/*namedWindow("distance_table",WINDOW_NORMAL);	
	imshow("distance_table",myMat);
	waitKey(2);*/
	imwrite("/home/qbobot/Documents/Images_brushfire/distance_init.jpg",myMat);
	printf("Count of obstacles : %d\n",count_obs);
	ROS_INFO("Distance table intialized well");
}
	


void testCostmap::queueInit()
{
	ROS_INFO("Initializing queue");
	int* cm = costarr_thresh_;
	int* dit = distance_transform_;
	for (int i=0;i<total_size_;i++,cm++,dit++)
	{
		if (*cm == THRESH_OCC)
		{
			findNeighbours(i);
			if (!neighbours_.empty())
			{
				for (std::vector<int>::iterator it= neighbours_.begin(); it != neighbours_.end();it++)
				{ 
					if (distance_transform_[*it] == 0 && checking_[*it] == 0)
					{
						priorityqueue_.push_back(*it);
						checking_[*it] = 1;
					}
				}
			}
		}
	}
	if (priorityqueue_.empty())
	{
		ROS_INFO("Can't init queue");
		exit(1);
	}
	else
	{
		int count_queue = priorityqueue_.size();
		printf("Number of elements added to the queue : %d\n",count_queue);
		ROS_INFO("Queue initialized well !");
	}
	Mat initiatedqueue(xs_,ys_,CV_8U);
	for (int bib = 0;bib < total_size_; bib++)
	{
		if (distance_transform_[bib] == -1)
		{
			initiatedqueue.at<unsigned char>(bib%xs_,bib/xs_) = 1;
		}
		else 
		{
			initiatedqueue.at<unsigned char>(bib%xs_,bib/xs_) = 75;
		}
	}

	for (std::vector<int>::iterator it = priorityqueue_.begin(); it!= priorityqueue_.end(); it++)
	{
		initiatedqueue.at<unsigned char>(*it%xs_,*it/xs_) = 250;
	}
	imwrite("/home/qbobot/Documents/Images_brushfire/priorityqueue.jpg",initiatedqueue);

}	

void testCostmap::distanceFilling()
{
	ROS_INFO("Filling distance map");
	std::vector<int>::iterator it;
	std::vector<int>::iterator neighb;
	int k = priorityqueue_.size();
	printf("First queue :%d\n",k);
	int firstround=k;
	int firstround2=-1;
	//printf("Queue size before entering queue : %d\n",k);
	int times = 0;
	int Nn;
	int minneighb;
	int l = 0;
	int count_secondround=0;
	//for (it=priorityqueue_.begin(); it!=priorityqueue_.end();it++)
	while (l!=k)
	{
		times++;
		//printf("times = %d\n",times);
		//findNeighbours(*it);
		findNeighbours(priorityqueue_.at(l));
		k = priorityqueue_.size();
		if (l == firstround-1)
		{
			firstround2 = k;
			printf("firstround2 = %d\n",firstround2);
		}
		//printf("Queue size : %d\n",k);
		
		if (!neighbours_.empty())
		{
			minneighb = distance_transform_[*neighbours_.begin()];
			neighb = neighbours_.begin();
			while (minneighb == 0 && neighb != neighbours_.end())
			{
				minneighb = *neighb;
			}
			/*if (*neighbours_.begin() < 0 || *neighbours_.begin() > total_size_)
			{
				printf(" Iteration next : ERREUR : neighb = %d and k = %d\n",minneighb,times);
				exit(1);
			}*/
				
			for (neighb = neighbours_.begin(); neighb != neighbours_.end();neighb++)
			{
				/*if (*neighb < 0 || *neighb >= total_size_)
				{
					printf("ERREUR : neighb = %d and k = %d\n",*neighb,k);
					exit(1);
				}*/
				if (minneighb > distance_transform_[*neighb] && distance_transform_[*neighb] != 0)
				{
					minneighb = distance_transform_[*neighb];
				}
				if (distance_transform_[*neighb] == 0 && checking_[*neighb] == 0)
				{
			//		printf("Pushing !\n");
					priorityqueue_.push_back(*neighb);
					checking_[*neighb] = 1;
				}
			}
			if (minneighb == -1)
			{
				if (l > firstround - 1)
				{
					printf("PROBLEM\n");
				}
			//	distance_transform_[*it] = 1;
				distance_transform_[priorityqueue_.at(l)] = 1;
			}
			else
			{
//				distance_transform_[*it] = minneighb + 1;
				distance_transform_[priorityqueue_.at(l)] = minneighb+1;
				if (l > firstround - 1 && l < firstround2 - 1 && distance_transform_[priorityqueue_.at(l)] == 2)
				{
					count_secondround++;
				}
			}
		}
		if (l == firstround2 - 1)
		{
			printf("Going in\n");
			Mat initiatedqueue(xs_,ys_,CV_8U);
			for (int bib = 0;bib < total_size_; bib++)
			{
				if (distance_transform_[bib] == -1)
				{
					initiatedqueue.at<unsigned char>(bib%xs_,bib/xs_) = 1;
				}
				else if (distance_transform_[bib] == 0) 
				{
					initiatedqueue.at<unsigned char>(bib%xs_,bib/xs_) = 75;
				}
				else if (distance_transform_[bib] == 1)
				{
					initiatedqueue.at<unsigned char>(bib%xs_,bib/xs_) = 140;
				}
				else if (distance_transform_[bib] == 2)
				{
					initiatedqueue.at<unsigned char>(bib%xs_,bib/xs_) = 250;
				}
				else
				{
					initiatedqueue.at<unsigned char>(bib%xs_,bib/xs_)  = 25;
				}

			}
				imwrite("/home/qbobot/Documents/Images_brushfire/priorityqueuesecondround.jpg",initiatedqueue);
		}
		if (l == firstround -1)
		{
			printf("Going in\n");
			Mat initiatedqueue(xs_,ys_,CV_8U);
			for (int bib = 0;bib < total_size_; bib++)
			{
				if (distance_transform_[bib] == -1)
				{
					initiatedqueue.at<unsigned char>(bib%xs_,bib/xs_) = 1;
				}
				else if (distance_transform_[bib] == 0) 
				{
					initiatedqueue.at<unsigned char>(bib%xs_,bib/xs_) = 75;
				}
				else if (distance_transform_[bib] == 1)
				{
					initiatedqueue.at<unsigned char>(bib%xs_,bib/xs_) = 250;
				}
				else if (distance_transform_[bib] == 2)
				{
					initiatedqueue.at<unsigned char>(bib%xs_,bib/xs_) = 140;
				}
				else
				{
					initiatedqueue.at<unsigned char>(bib%xs_,bib/xs_)  = 25;
				}

			}
				imwrite("/home/qbobot/Documents/Images_brushfire/priorityqueuefirstround.jpg",initiatedqueue);
		}
			 
		l++;
	}
	printf("Count of second round : %d\n", count_secondround);
	int count_zeros = 0;
	for (int bibu = 0; bibu < total_size_; bibu++)
	{
		if (distance_transform_[bibu] == 0)
		{
			count_zeros++;
		}
	}
	printf("Zero count : %d\n",count_zeros);
	printf("k = %d, 1984*1984-1984x4 = %d\n",k,1984*1984-1984*4);
}

void testCostmap::findNeighbours(int ind)
{
	neighbours_.clear();
	if (ind%xs_!= 0 && ind/xs_ != 0 && ind/xs_ != ys_-1 && ind%xs_ != xs_-1 )
	{
		neighbours_.push_back(ind + 1);
		neighbours_.push_back(ind - 1);
		neighbours_.push_back(ind-xs_);
		neighbours_.push_back(ind+xs_);
		/* 8 neighbours_
		neighbours_.push_back(ind-xs_-1);
		neighbours_.push_back(ind-xs_+1);
		neighbours_.push_back(ind+xs_-1);
		neighbours_.push_back(ind+xs_+1);*/
	}  
}

void testCostmap::computeGrad()
{
	/* Compute gradient and module */

	int i;
	int * it = distance_transform_;
	for (i=0;i<total_size_;i++,it++)
	{
		if (i%xs_!= 0 && i/xs_ != 0 && i/xs_ != ys_-1 && i%xs_ != xs_-1 )
		{
			gradx_[i] = *(it + 1) + *(it+xs_+1) + *(it-xs_+1) - *(it - 1) - *(it-xs_-1) - *(it+xs_-1);
			grady_[i] = *(it - xs_) + *(it-xs_+1) + *(it-xs_-1) - *(it+ xs_) - *(it+xs_+1) - *(it+xs_-1);
		}
		if (*it != -1 && *it!=0)
		{
			module_[i] = sqrt((double)pow(gradx_[i],2) + (double)pow(grady_[i],2));
		}
	}
	
	/* Check the result on image */
	
	double max = 0;	
	for (i = 0; i<total_size_;i++)
	{
		if (max < module_[i])
		{
			max = module_[i];
		}
	}	
	Mat res(xs_,ys_,CV_8U);
	it = distance_transform_;
	for (i = 0; i < total_size_;i++,it++)
	{
		if (*it == 0 || *it == -1)
		{
			res.at<unsigned char>((int)i%xs_,(int)i/xs_) = 255;
		}
		else 
		{
			res.at<unsigned char>((int)i%xs_,(int)i/xs_) = (unsigned char) ((module_[i]/max)*255);
		}
	}
	ROS_INFO("Writing gradient to jpg file");
	imwrite("/home/qbobot/Documents/Images_brushfire/gradient.jpg",res);
			
}

void testCostmap::computeGraph()
{
	vector<int>::iterator it;
	vector<int>::iterator neighb;
	std::vector< std::pair<int,int> > addToGraph;
	vector<int>::iterator found;
	graph_ = std::vector< std::vector<pair<int,int> > >(total_size_);
	for (it =skelcostmap_.begin();it!= skelcostmap_.end();it++)
	{
		findNeighbours(*it);
		for (neighb = neighbours_.begin();neighb!= neighbours_.end();neighb++)
		{
			found = std::find(skelcostmap_.begin(),skelcostmap_.end(),*neighb);
			if (found != skelcostmap_.end())
			{
				std::pair<int,int> thepair(*neighb,1);
				graph_[*it].push_back(thepair);
			}
		}
	} 
}

};
/*int main(int argc,char** argv)
{
	ros::init(argc,argv,"testCostmap");
	tf::TransformListener tf2(ros::Duration(10));
	costmap_2d::Costmap2DROS lcr2("global_costmap",tf2);
	testCostmap mytestcostmap(&lcr2);
	ros::spin();
	return 0;
}*/
