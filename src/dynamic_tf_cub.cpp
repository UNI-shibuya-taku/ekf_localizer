#include "tf/dynamic_tf_cub.h"
DynamicTfCub::DynamicTfCub():private_nh("~")
{
    /*subscriber*/
    sub_current_pose = nh.subscribe("/ekf_pose",10,&NextWaypointCreator::current_pose_callback,this);
}
void DynamicTfCub::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
    pub_dynamic_tf();
}

void DynamicTfCub::pub_dynamic_tf()
{
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = current_pose.pose.position.x;
    transformStamped.transform.translation.y = current_pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;

    // tf2::Quaternion q;
    // q.setRPY(0, 0, 0.0); // YAW
    transformStamped.transform.rotation.x = current_pose.pose.orientation.x;
    transformStamped.transform.rotation.y = current_pose.pose.orientation.y;
    transformStamped.transform.rotation.z = current_pose.pose.orientation.z;
    transformStamped.transform.rotation.w = current_pose.pose.orientation.w;
    dynamic_br_.sendTransform(transformStamped);
}

void DynamicTfCub::process()
{
    ros::Spin();
}

int main (int argc,char **argv)
{
    ros::init(argc, argv, "dynamic_tf_cub");
    DynamicTfCub dynamic_tf_cub;
    dynamic_tf_cub.process();
    return 0;
}