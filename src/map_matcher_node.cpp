#include "map_matcher/map_matcher.h"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"map_matcher");
	MapMatcher matcher;
	matcher.process();
	return 0;
}