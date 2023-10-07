#ifndef __EKF_H
#define __EKF_H
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
// #include "geometry_msgs/Pose.h"
// #include "geometry_msgs/PoseArray.h"

// pcl
// #include <pcl_ros/transforms.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types_conversion.h>
// #include <pcl/point_types.h>
// #include <pcl/conversions.h> // new!!!
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/registration/icp.h>
// #include <pcl/PointIndices.h>
// #include <pcl_msgs/PointIndices.h> // new!!!
// #include <pcl/io/pcd_io.h>
//#include <pcl/filters/passthrough.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>
#include <string>
// #include <math.h>

class EKF
{
    public:
        EKF();
        void process();
    private:
        // callback
        void match_callback(const geometry_msgs::PoseStamped::ConstPtr&);
        void imu_callback(const sensor_msgs::ImuConstPtr&);
        void odom_callback(const nav_msgs::Odometry::ConstPtr&);
        void measurement_callback(const std_msgs::BoolConstPtr&);
        void respawn_pose_callback(const geometry_msgs::PoseStampedConstPtr&);

        void initialize(double, double, double, double, double, double);
        void set_pose(double, double, double, double, double, double);
        void calc_rpy_from_quat(geometry_msgs::Quaternion, double&, double&, double&);
        
        void motion_update_3DoF(double);
        void motion_update_6DoF(double);
        void motion_update(double);
        void measurement_update();
        void measurement_update_3DoF();
        void measurement_update_6DoF();
        
        void respawn();

        void publish_ekf_pose();
        void publish_tf();

        double normalize_angle(double);
        double calc_yaw_from_quat(geometry_msgs::Quaternion);
        geometry_msgs::Quaternion rpy_to_msg(double, double, double);
        Eigen::Matrix3d calc_rotation_matrix(Eigen::Vector3d);
        Eigen::VectorXd measurement_function(Eigen::VectorXd, Eigen::MatrixXd);
        // void print4x4Matrix (const Eigen::Matrix4d &);
        // geometry_msgs::Quaternion quat_eigen_to_msg(Eigen::Quaterniond);

        int state_size;
        double init_x;
        double init_y;
        double init_z;
        double init_roll;
        double init_pitch;
        double init_yaw;
        double init_sigma;
        double sigma_imu;
        double sigma_odom;
        double sigma_match;
        double motion_noise_nn;
        double motion_noise_no;
        double motion_noise_on;
        double motion_noise_oo;
        bool has_received_odom;
        bool has_received_imu;
        bool has_received_icp_pose;
        bool is_first_;
        bool is_odom_tf_;
        bool is_3DoF_;
        bool odom_flag;

        // nodehandle
        ros::NodeHandle private_nh;
        ros::NodeHandle nh;
        // subscriber
        ros::Subscriber sub_match_pose;
        ros::Subscriber sub_imu;
        ros::Subscriber sub_odom;
        // publisher
        ros::Publisher pub_ekf_pose;

        geometry_msgs::PoseStamped icp_pose;
        geometry_msgs::PoseStamped ekf_pose;
        sensor_msgs::Imu imu;
        nav_msgs::Odometry current_odom;
        std_msgs::Bool is_measurement_;
        // nav_msgs::Odometry last_odom;
        // Eigen::Vector2d last_odom_position;
        // std::vector<Eigen::Vector3d> vehicle_velocities;
        Eigen::VectorXd X_;
        Eigen::MatrixXd P_;
        ros::Time now_time_;
        ros::Time last_time_;
};
#endif
