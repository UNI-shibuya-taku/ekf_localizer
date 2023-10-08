#ifndef TF_BROADCASTER_H_
#define TF_BROADCASTER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

struct Coordinate{
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
};

class TFBroadcaster
{
public:
    TFBroadcaster();
    ~TFBroadcaster();
    void process();

private:
    void odometry_callback(const nav_msgs::OdometryConstPtr& msg);
	geometry_msgs::TransformStamped create_transform_stamped_msg(std::string frame_id,std::string child_frame_id,Coordinate coordinate);

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

    geometry_msgs::TransformStamped camera_transform_;
	geometry_msgs::TransformStamped velodyne_transform_;

    std::string odom_topic_name_;
    std::string odom_frame_id_;
	std::string base_link_frame_id_;
	std::string camera_frame_id_;
	std::string velodyne_frame_id_;
	bool is_odom_tf_;

    Coordinate camera_coordinate_;
	Coordinate velodyne_coordinate_;

	ros::Subscriber odom_sub_;

	boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
	boost::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

#endif  // TF_BROADCASTER_H_
