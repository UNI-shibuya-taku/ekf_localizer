#ifndef EKF_H_
#define EKF_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>

class EKF
{
public:
	EKF();
	~EKF();
	void process();

private:
	void ndt_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);
	void odom_callback(const nav_msgs::OdometryConstPtr& msg);
	void imu_callback(const sensor_msgs::ImuConstPtr& msg);
	
	void measurement_callback(const std_msgs::BoolConstPtr& msg);
	void respawn_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);

	void initialize(double x,double y,double z,double roll,double pitch,double yaw);
	void set_pose(double x,double y,double z,double roll,double pitch,double yaw);
	void calc_rpy_from_quat(geometry_msgs::Quaternion q,double& roll,double& pitch,double& yaw);
	
	void motion_update_3DoF(double dt);
	void motion_update_6DoF(double dt);
	void motion_update(double dt);
	void measurement_update();
	void measurement_update_3DoF();
	void measurement_update_6DoF();
	
	void respawn();

	void publish_ekf_pose();
	void publish_tf();

	double normalize_angle(double angle);
	double calc_yaw_from_quat(geometry_msgs::Quaternion q);
	geometry_msgs::Quaternion rpy_to_msg(double roll,double pitch,double yaw);
	Eigen::Matrix3d calc_rotation_matrix(Eigen::Vector3d euler_angle);
	Eigen::VectorXd measurement_function(Eigen::VectorXd x,Eigen::MatrixXd h);

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber ndt_pose_sub_;
	ros::Subscriber imu_sub_;
	ros::Subscriber odom_sub_;
	ros::Publisher ekf_pose_pub_;

	ros::Subscriber measurement_sub_;
	ros::Subscriber respawn_pose_sub_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;

	nav_msgs::Odometry odom_;
	geometry_msgs::PoseStamped ndt_pose_;
	geometry_msgs::PoseStamped ekf_pose_;
	sensor_msgs::Imu imu_;
	ros::Time now_time_;
	ros::Time last_time_;

	std_msgs::Bool is_measurement_;
	geometry_msgs::PoseStamped respawn_pose_;

	std::string ndt_pose_topic_name_;
	std::string imu_topic_name_;
	std::string odom_topic_name_;
	std::string ekf_pose_topic_name_;
	std::string map_frame_id_;
	std::string odom_frame_id_;
	std::string base_link_frame_id_;

	std::string measurement_topic_name_;
	std::string respawn_pose_topic_name_;

	bool has_received_odom_;
	bool has_received_imu_;
	bool has_received_ndt_pose_;
	bool is_first_;
	bool is_odom_tf_;
	bool is_3DoF_;

	bool is_respawn_;

	double INIT_X_;
	double INIT_Y_;
	double INIT_Z_;
	double INIT_ROLL_;
	double INIT_PITCH_;
	double INIT_YAW_;
	double INIT_SIGMA_;
	double SIGMA_IMU_;
	double SIGMA_ODOM_;
	double SIGMA_NDT_;
	double MOTION_NOISE_NN_;
	double MOTION_NOISE_NO_;
	double MOTION_NOISE_ON_;
	double MOTION_NOISE_OO_;

	int STATE_SIZE_;
	Eigen::VectorXd X_;
	Eigen::MatrixXd P_;
};

#endif	// EKF_H_