#ifndef __NDT_H
#define __NDT_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

class ScanMatchingNDT{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		ros::Subscriber sub_pose;
		/*publish*/
		ros::Publisher pub_pose;
		ros::Publisher pub_pc;
		/*viewer*/
		pcl::visualization::PCLVisualizer viewer{"scan_matching_ndt"};
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_now {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_trans {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map_filtered {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_now_filtered {new pcl::PointCloud<pcl::PointXYZ>};
		/*pose*/
		geometry_msgs::PoseStamped pose_ekf;
		geometry_msgs::PoseStamped pose_ndt;
		/*flags*/
		bool first_callback_pose = true;
		/*parameters*/
		double pc_range;
		double leafsize_source;
		double leafsize_target;
		double trans_epsilon;
		double stepsize;
		double resolution;
		int max_iterations;
	public:
		ScanMatchingNDT();
		void CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg);
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void InitialRegistration(void);
		bool Transformation(void);
		void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, std::vector<double> range);
		void Downsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double leafsize);
		void Visualization(void);
		void Publication(void);
		Eigen::Quaternionf QuatMsgToEigen(geometry_msgs::Quaternion q_msg);
		geometry_msgs::Quaternion QuatEigenToMsg(Eigen::Quaternionf q_eigen);
};
#endif
