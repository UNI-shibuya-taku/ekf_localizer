#include "sq_scan_matcher/test_sq_scan_matcher.h"
SqScanMather::SqScanMather() : private_nh("~")
{
    // private_nh.param("store_num", store_num, {10});
    /*subscriber*/
    sub_cloud = nh.subscribe("/cloud/lcl",10, &SqScanMather::cloud_callback,this);
    /*publisher*/
    // pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("",1);
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/scan_matching/pose",20); // lclへ送る？
    
    last_cloud.reset(new CloudNormal);
    std::cout << "========start sq_scan_matcher!!!!!!==========" << std::endl;
}
// void MatchingPointsMover::odom_callback(const nav_msgs::Odometry::ConstPtr& msg_odom)
// {
//     current_odom = *msg_odom;
// }

void SqScanMather::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg_cloud)
{
    ros::Time t_start = ros::Time::now();
    CloudNormalPtr cloud(new CloudNormal);
    pcl::fromROSMsg(*msg_cloud, *cloud);
    
    pcl::IterativeClosestPoint<PointNormal, PointNormal> icp;
    icp.setInputCloud(last_cloud); // before moving
    icp.setInputTarget(cloud); // after moving
    CloudNormalPtr final_cloud(new CloudNormal);
    icp.align(*final_cloud);
    
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    transformation_matrix = icp.getFinalTransformation().cast<double>();
    print4x4Matrix(transformation_matrix);
    
    Eigen::Matrix3d q = transformation_matrix.block(0,0,3,3);
    Eigen::Quaterniond quaternion(q);
    quaternion.normalize();

    // scan matchingによる相対移動量
    geometry_msgs::PoseStamped diff_pose;
    diff_pose.pose.position.x = transformation_matrix(0,3);
    diff_pose.pose.position.y = transformation_matrix(1,3);
    diff_pose.pose.position.z = transformation_matrix(2,3);
    //diff_pose.pose.position.z = 0.0;
    diff_pose.pose.orientation = quat_eigen_to_msg(quaternion);
    diff_pose.header.frame_id = "base_link";
    pub_pose.publish(diff_pose);

    *last_cloud = *cloud;
    double dt = (ros::Time::now() - t_start).toSec();
    std::cout << "time[s]: " << dt << std::endl;
}
geometry_msgs::Quaternion SqScanMather::quat_eigen_to_msg(Eigen::Quaterniond q)
{
	geometry_msgs::Quaternion msg;
	msg.x = (double)q.x();
	msg.y = (double)q.y();
	msg.z = (double)q.z();
	msg.w = (double)q.w();

	return msg;
}
void SqScanMather::print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}
void SqScanMather::process()
{
    ros::spin();
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sq_scan_matcher");
    SqScanMather sq_scan_matcher;
    sq_scan_matcher.process();
    return 0;
}