#include "tf/tf_broadcaster.h"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"tf_broadcaster");
	TFBroadcaster broadcaster;
	broadcaster.process();
	return 0;
}