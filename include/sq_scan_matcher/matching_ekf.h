#ifndef __MATCHING_EKF_H
#define __MATCHING_ EKF_H
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/LU>
class ScanMatchingEKF{
	public:
		ScanMatchingEKF();
	private:
		void imu_callback(const sensor_msgs::ImuConstPtr&);
		void odom_callback(const nav_msgs::OdometryConstPtr&);
        void initialize(double, double, double, double, double, double);
		void imu_prediction(sensor_msgs::Imu, double);
		void odom_prediction(nav_msgs::Odometry, double);
		void pose_callback(const geometry_msgs::PoseStampedConstPtr &);
		void pose_observation(geometry_msgs::PoseStamped);
		void Publication();
		geometry_msgs::PoseStamped StateVectorToPoseStamped(void);
		Eigen::Matrix3d GetRotationXYZMatrix(const Eigen::Vector3d&, bool);
		double PiToPi(double);
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_imu;
		ros::Subscriber sub_odom;
		ros::Subscriber sub_pose;
		/*publish*/
		tf::TransformBroadcaster tf_broadcaster;
		ros::Publisher pub_pose;
		/*const*/
		const int size_robot_state = 6;	//X, Y, Z, R, P, Y (Global)
		/*objects*/
		Eigen::VectorXd X;
		Eigen::MatrixXd P;
		/*flags*/
		bool first_callback_imu = true;
		bool first_callback_odom = true;
		bool odom_flag;
		/*time*/
		ros::Time time_publish;
		ros::Time time_imu_now;
		ros::Time time_imu_last;
		ros::Time time_odom_now;
		ros::Time time_odom_last;
		/*parameters*/
		std::string child_frame_name;
		std::string parent_frame_name;
		double init_x;
		double init_y;
		double init_z;
		double init_roll;
		double init_pitch;
		double init_yaw;
		double sigma_imu;
		double sigma_odom;
		double sigma_ndt;
};
#endif
