#ifndef MAP_MATCHER_H_
#define MAP_MATCHER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

class MapMatcher
{
public:
	MapMatcher();
	void process();

private:
	void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
	void orb_pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
	void ekf_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);
	void map_callback(const sensor_msgs::PointCloud2ConstPtr& msg);


 	void init_map();
	void read_map();

	void set_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl,pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pcl,double x,double y,double z);
	void downsample_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl,pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pcl,double voxel_size);
	void matching(pcl::PointCloud<pcl::PointXYZI>::Ptr map_pcl,pcl::PointCloud<pcl::PointXYZI>::Ptr local_pcl);

	double get_yaw_from_quat(geometry_msgs::Quaternion q);
	geometry_msgs::Quaternion rpy_to_msg(double roll,double pitch,double yaw);
	geometry_msgs::Quaternion quat_eigen_to_msg(Eigen::Quaternionf q);
	Eigen::Quaternionf msg_to_quat_eigen(geometry_msgs::Quaternion q);

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Publisher ndt_pose_pub_;
	ros::Publisher map_pub_;
	ros::Publisher ndt_pc_pub_;
	ros::Subscriber pc_sub_;
	ros::Subscriber orb_pc_sub_;
	ros::Subscriber ekf_pose_sub_;
	ros::Subscriber map_sub_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
	std::shared_ptr<tf2_ros::TransformListener> listener_;
	std::shared_ptr<tf2_ros::Buffer> buffer_;

	pcl::PointCloud<pcl::PointXYZI>::Ptr map_pcl_;		// map 点群
	pcl::PointCloud<pcl::PointXYZI>::Ptr current_pcl_;	// lidar 点群

	sensor_msgs::PointCloud2 pc_;
	sensor_msgs::PointCloud2 orb_pc_;
	geometry_msgs::PoseStamped ekf_pose_;
	ros::Time pc_time_;
	bool is_reset_;
	bool is_start_;
	bool has_received_ekf_pose_;
	bool has_received_pc_;
	bool has_read_map_;
	bool is_first;
	bool is_publish_map_;

	// parameter
	std::string pcd_file_path_;
	std::string pc_topic_name_;
	std::string orb_pc_topic_name_;
	std::string ekf_pose_topic_name_;
	std::string ndt_pose_topic_name_;
	std::string map_topic_name_;
	std::string ndt_pc_topic_name_;
	std::string map_frame_id_;

	bool is_pcl_offset_;

	double VOXEL_SIZE_;
	double LIMIT_RANGE_;
	double TRANS_EPSILON_;
	double STEP_SIZE_;
	double RESOLUTION_;
	double MAX_ITERATION_;
	double MATCHING_SCORE_TH_;
	double MAP_OFFSET_X_;
	double MAP_OFFSET_Y_;
	double MAP_OFFSET_Z_;
	double MAP_OFFSET_ROLL_;
	double MAP_OFFSET_PITCH_;
	double MAP_OFFSET_YAW_;
};

#endif	// MAP_MATCHER_H_