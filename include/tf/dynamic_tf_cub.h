#ifndef __DYNAMIC_TF_CUB_H
#define __DYNAMIC_TF_CUB_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

class DynamicTfCub
{
public:
    DynamicTfCub();
    void process();

private:
    //method
    void pub_dynamic_tf();
    //parameter

    //member
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher pub_next_waypoint;
    ros::Subscriber sub_current_pose;

    geometry_msgs::PoseStamped current_pose;
    tf2_ros::TransformBroadcaster dynamic_br_;
};

#endif
