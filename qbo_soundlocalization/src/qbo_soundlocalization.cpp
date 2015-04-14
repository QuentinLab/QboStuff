#include "qbo_soundlocalization.h"

QboLocalizer::QboLocalizer()
{
	ROS_INFO("Starting sound localization");
	OnInit();
}

QboLocalizer::~QboLocalizer()
{
	ROS_INFO("Done localizing");
}

void QboLocalizer::Correlation()
{
	double Max = 0;
	int SampleDelay = 0;
	int i;
	for(i=0; i < (2 * n_ + 1); i+=1)
	{
		Ycorr_[i] = 0;
		Xdec_[i] = i - n_;
		if( (n_ - i) >= 0)
		{
			int k;
			for(k = 0; k < (buffersize_ - (n_ - i)); k += 1)
			{
				Ycorr_[i] += ((left_[k + (n_ - i)]-meanleft_) * (right_[k]-meanright_))/(stdevleft_*stdevright_);
			}
		}
		else
		{
			int k;			
			for(k = 0; k < (buffersize_ + n_ - i); k+=1)
			{
				Ycorr_[i] += ((left_[k]-meanleft_) *(right_[k - n_ + i]-meanright_))/(stdevleft_*stdevright_);
			}
		}
		if(Max < Ycorr_[i])
		{
			Max = Ycorr_[i];
			SampleDelay = Xdec_[i];
		}
	}
	Delay_ = SampleDelay;
	cout << "Max :" << Max << endl;
}

void QboLocalizer::TetaRad()
{
	if (Delay_ >= 0)
	{
		Teta_ = acos(((double)Delay_/sampleRate_)*v_/D_);
	}
	else
	{
		Teta_ = (PI - acos((-(double)Delay_/sampleRate_)*v_/D_));
	}
}

void QboLocalizer::OnInit()
{
	n_ = 20; // Number of samples tested for correlation (both left and right)
	int sampleCorr = 40; // Size of the correlation table 
	sampleRate_ = 44100;
	buffersize_ = 44100;
	left_ = (double*) malloc(sizeof(double)*buffersize_);
	right_ = (double*) malloc(sizeof(double)*buffersize_);
	Ycorr_ = (double*) malloc(sizeof(double)*sampleCorr);
	Xdec_ = (int*) malloc(sizeof(int)*sampleCorr);
	D_ = 0.1475; // Distance between the two microphones
	v_ = 340.0;

	angle_pub_ = private_nh_.advertise<std_msgs::Float64>("/audio_angle",100);
	samples_sub_ = private_nh_.subscribe("/qbo_soundstream/samples",1,&QboLocalizer::sampleCallback,this);
}

void QboLocalizer::sampleCallback(const std_msgs::Int16MultiArray& samples)
{
	int i;
	for (i=0;i<buffersize_;i++)
	{
		left_[i] = (double)samples.data[2*i];
		right_[i] = (double)samples.data[2*i+1];
	}
	meanleft_ = computeMean(left_);
	meanright_ = computeMean(right_);
	stdevleft_ = computeStdev(left_,meanleft_);
	stdevright_ = computeStdev(right_,meanright_);
	Correlation();
	TetaRad();
	Teta_ += -PI/2 ;
	angle_.data = Teta_;
	angle_pub_.publish(angle_);
	cout << "Angle : " << Teta_ << endl;
}

double QboLocalizer::computeMean(double* table)
{
	int i;
	double temp=0;
	for (i=0;i<buffersize_;i++)
	{
		temp+= table[i];
	}
	return temp/i;
}

double QboLocalizer::computeStdev(double* table, double mean)
{
	double temp = 0;
	for(int i = 0; i < buffersize_; i++)
	{
		temp += (table[i] - mean) * (table[i] - mean);
        }
	return temp /buffersize_;
} 

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"qbo_soundlocalization");
	QboLocalizer qbo_localizer;
	ros::spin();
	return 0;
}
