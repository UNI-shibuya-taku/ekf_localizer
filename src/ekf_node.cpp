#include "ekf/ekf.h"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"ekf");
	EKF ekf;
	ekf.process();
	return 0;
}