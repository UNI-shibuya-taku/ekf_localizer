#ifndef __ICP_SQ_SCAN_MATCHER_H
#define __ICP_SQ_SCAN_MATCHER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_representation.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
// #include <pcl/registration/ndt.h>
// #include <pcl/registration/registration.h>

class IcpSqScanMather{
	public:
		typedef pcl::PointXYZI PointXYZI;
		typedef pcl::PointCloud<PointXYZI> CloudXYZI;
		typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;

		// typedef pcl::PointXYZINormal PointXYZIN;
		// typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
		// typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;
		
		// typedef pcl::PointNormal PointNormal;
		// typedef pcl::PointCloud<PointNormal> CloudNormal;
		// typedef pcl::PointCloud<PointNormal>::Ptr CloudNormalPtr; 
		
		IcpSqScanMather();
		void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);
		void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		void initial_registration(void);
		bool transformation(void);
		void PassThroughFilter(CloudXYZIPtr, CloudXYZIPtr, std::vector<double>);
		void Downsampling(CloudXYZIPtr, double);
		// void Visualization(void);
		void publication(void);
		Eigen::Quaternionf quat_msg_to_eigen(geometry_msgs::Quaternion q_msg);
		geometry_msgs::Quaternion quat_eigen_to_msg(Eigen::Quaternionf q_eigen);
    private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle private_nh;
		/*subscribe*/
		ros::Subscriber sub_pc;
		ros::Subscriber sub_pose;
		/*publish*/
		ros::Publisher pub_pose;
		// ros::Publisher pub_pc;
		/*cloud*/
		CloudXYZIPtr pc_now{new CloudXYZI};
		CloudXYZIPtr pc_last{new CloudXYZI};
		CloudXYZIPtr pc_map{new CloudXYZI};
		CloudXYZIPtr pc_trans{new CloudXYZI};
		CloudXYZIPtr pc_map_filtered{new CloudXYZI};
		CloudXYZIPtr pc_now_filtered{new CloudXYZI};
		geometry_msgs::PoseStamped pose_ekf;
		geometry_msgs::PoseStamped pose_icp;
		geometry_msgs::PoseStamped pose_test;
		int max_iterations;
		bool first_callback_pose = true;
		double pc_range;
		double leafsize_source;
		double leafsize_target;
		double leafsize;
		double trans_epsilon;
		double matching_score_th;
		double correspondence_distance;
		// double stepsize;
		// double resolution;
};
#endif
