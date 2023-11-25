#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
// #include <sensor_msgs/Imu.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>

class OdomFilter
{
public:
	OdomFilter();
	~OdomFilter();
	void process();

private:
	void odom_callback(const nav_msgs::OdometryConstPtr& msg);
	double get_yaw(geometry_msgs::Quaternion q);
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber odom_sub_;
	ros::Publisher odom_filter_pub_;
	nav_msgs::Odometry current_odom_;
	// nav_msgs::Odometry last_odom_;
	bool init_callback = false;
	double current_yaw;
	double last_yaw;
};
OdomFilter::OdomFilter() :
	private_nh_("~")
{
	// private_nh_.param("ndt_pose_topic_name", ndt_pose_topic_name_, {"/ndt_pose_in"});
	// private_nh_.param("INIT_X", INIT_X_, {0.0});
	odom_sub_ = nh_.subscribe("/atom/odometry",10,&OdomFilter::odom_callback,this);
	odom_filter_pub_ = nh_.advertise<nav_msgs::Odometry>("/atom/odometry/filter",10);
}
double OdomFilter::get_yaw(geometry_msgs::Quaternion q)
{
    double roll , pitch , yaw;
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}
OdomFilter::~OdomFilter()
{
}
void OdomFilter::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
	current_odom_ = *msg;
	// filterinng
	current_yaw = get_yaw(current_odom_.pose.pose.orientation);
	std::cout << "current_yaw: " << current_yaw << std::endl;
	if(init_callback == true && fabs(current_yaw - last_yaw) < 10.0){
		odom_filter_pub_.publish(current_odom_);
	}
	last_yaw = current_yaw;
	init_callback = true;
}

void OdomFilter::process()
{
	while(ros::ok()){
		ros::spinOnce();
		// rate.sleep();
	}
}

int main (int argc,char **argv)
{
    ros::init(argc, argv, "odom_filter");
    OdomFilter odom_filter;
    odom_filter.process();
    return 0;
}