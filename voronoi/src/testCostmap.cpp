#include "testCostmap.h" 

testCostmap::testCostmap()
{
}

testCostmap::testCostmap(costmap_2d::Costmap2DROS* costmap_ros)
{
	onInit(costmap_ros);
}

testCostmap::~testCostmap()
{
	ROS_INFO("The end");
}

void testCostmap::onInit(costmap_2d::Costmap2DROS* costmap_ros)
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

	ROS_INFO("Set 3");

	computeBrushfire();

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
					printf("No! ! i = %d k = %d\n",i,k);
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
	exit(0);
	queueInit();
	distanceFilling();
	computeGrad();
	orderModule();

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
	printf("MAXipipipipipipidididididdii = %d\n", max);
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
	/*int* order = new int[total_size_];
	int* order_ind = new int[total_size_];
	int p,h;
	for ( p = 0; p < total_size_; p++)
	{
		order[p] = distance_transform_[p];
	}
	int max_inter,a,b,ind;
	for (p = 0; p < total_size_;p++)
	{
		printf("OK :D");
		max_inter = order[0];
		ind = 0;
		for (h = 0; h <total_size_ - p; h++)
		{
			if (max_inter < order[h])
			{	
				max_inter = order[h];
				ind = h;
			}
		}
		a = order[ind];
		order[ind] = order[total_size_ - p - 1];
		order[total_size_ - p - 1] = a;
		order_ind[p] = ind;
	}*/

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
	/*int bibs;
	for (bibs = 0; bibs < 50; bibs++);
	{
		skeleton.at<unsigned char>(order_ind[bibs]%xs_,order_ind[bibs]/xs_) = 140;
	}*/
			
			
	
	/*namedWindow("image_transform",WINDOW_NORMAL);
	resizeWindow("image_transform", 700,700);
	imshow("image_transform", distMat);*/
	imwrite("/home/qbobot/Documents/rays.jpg", rays);
	imwrite("/home/qbobot/Documents/image.jpg",distMat);
	imwrite("/home/qbobot/Documents/skeleton.jpg",skeleton);
	waitKey(2);
	exit(1);
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
	for (p = 0; p < 1000;p++)
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

	/* Image of the minimum of gradient */
	ROS_INFO("COMPUTING SKELETON FILE");

	int i;
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
	for (i=0; i<1000; i++)
	{
		res.at<unsigned char>(order_ind[i]%xs_,order_ind[i]/xs_) = 140;
	}
	imwrite("/home/qbobot/Documents/skel.jpg",res);	
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
	imwrite("/home/qbobot/Documents/thresholded_map.jpg",myMat);


	/* Test for distanceTransform from opencv*/

	Mat distance_transform(xs_,ys_,CV_32FC1);
	Mat voronoi_diagram(xs_,ys_,CV_32SC1);
	distanceTransform(myMat,distance_transform,voronoi_diagram,CV_DIST_L2,5,DIST_LABEL_CCOMP);
	double max,min;

	minMaxLoc(distance_transform,&min,&max,NULL,NULL);
	printf("the max is : %hf",max);

	Mat distance_transform_norm = distance_transform/max*255;

	minMaxLoc(voronoi_diagram,&min,&max,NULL,NULL);
	
	Mat voronoi_diagram_norm = voronoi_diagram/max*255;
	imwrite("/home/qbobot/Documents/distance_transform.jpg",distance_transform);
	imwrite("/home/qbobot/Documents/voronoi_diagram.jpg",voronoi_diagram);
	imwrite("/home/qbobot/Documents/distance_transform_norm.jpg",distance_transform_norm);
	imwrite("/home/qbobot/Documents/voronoi_diagram_norm.jpg",voronoi_diagram_norm);


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
	imwrite("/home/qbobot/Documents/distance_init.jpg",myMat);
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
		printf("Number of elements added to the queue : %d",count_queue);
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
	imwrite("/home/qbobot/Documents/priorityqueue.jpg",initiatedqueue);

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
				imwrite("/home/qbobot/Documents/priorityqueuesecondround.jpg",initiatedqueue);
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
				imwrite("/home/qbobot/Documents/priorityqueuefirstround.jpg",initiatedqueue);
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
	printf("k = %d, 1984*1984-1984x4 = %d",k,1984*1984-1984*4);
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
		neighbours_.push_back(ind+xs_+1) ;*/
	}  
}

void testCostmap::computeGrad()
{
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
	imwrite("/home/qbobot/Documents/gradient.jpg",res);
			
}
int main(int argc,char** argv)
{
	ros::init(argc,argv,"testCostmap");
	tf::TransformListener tf2(ros::Duration(10));
	costmap_2d::Costmap2DROS lcr2("global_costmap",tf2);
	testCostmap mytestcostmap(&lcr2);
	ros::spin();
	return 0;
}
