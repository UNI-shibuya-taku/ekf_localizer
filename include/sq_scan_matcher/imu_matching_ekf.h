#ifndef __IMU_MATCHING_EKF_H
#define __IMU_MATCHING_ EKF_H
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/LU>
class ImuMatchingEKF{
	public:
		ImuMatchingEKF();
	private:
		void imu_callback(const sensor_msgs::ImuConstPtr&);
		void odom_callback(const nav_msgs::OdometryConstPtr&);
		void initialize(double, double, double, double, double, double);
		void imu_prediction(sensor_msgs::Imu, double);
		void odom_prediction(nav_msgs::Odometry, double);
		void pose_callback(const geometry_msgs::PoseStampedConstPtr &);
		void pose_observation(geometry_msgs::PoseStamped);
		void Publication();
		// void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<double>);
		geometry_msgs::PoseStamped StateVectorToPoseStamped(void);
		Eigen::Matrix3d GetRotationXYZMatrix(const Eigen::Vector3d&, bool);
		double PiToPi(double);
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		ros::Subscriber sub_imu;
		ros::Subscriber sub_odom;
		ros::Subscriber sub_pose;
		tf::TransformBroadcaster tf_broadcaster;
		ros::Publisher pub_pose;
		const int size_robot_state = 6;	//X, Y, Z, R, P, Y (Global)
		Eigen::VectorXd X;
		Eigen::MatrixXd P;
		bool first_callback_imu = true;
		bool first_callback_odom = true;
		bool odom_flag;
		bool pose_flag = false;
		ros::Time time_publish;
		ros::Time time_imu_now;
		ros::Time time_imu_last;
		ros::Time time_odom_now;
		ros::Time time_odom_last;
		/*parameters*/
		std::string child_frame_name;
		std::string parent_frame_name;
		sensor_msgs::Imu last_imu;
		geometry_msgs::PoseStamped icp_pose;
		geometry_msgs::PoseStamped icp_last_pose;
		double init_x;
		double init_y;
		double init_z;
		double init_roll;
		double init_pitch;
		double init_yaw;
		double sigma_imu;
		double sigma_odom;
		double sigma_ndt;
		double delta_pose;
		double current_pose;
		double last_pose;
		double imu_velocity;
		double filter_coefficient = 0.9;
		double lowpass_value = 0.0;
		double highpass_value = 0.0;
		double old_accel;	
		double old_speed;	
		double difference = 0.0;	
		double imu_bias = 0.01;	
};
#endif