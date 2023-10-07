#ifndef __SQ_SCAN_MATHCER_H
#define __SQ_SCAN_MATCHER_H
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
// #include "geometry_msgs/Pose.h"
// #include "geometry_msgs/PoseArray.h"

// pcl
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h> // new!!!
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
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
#include <pcl/PointIndices.h>

#include <vector>
#include <iostream>
#include <omp.h>
#include <string>
#include <random>
// #include <math.h>

class SqScanMather
{
    public:
            SqScanMather();
            typedef pcl::PointXYZINormal PointXYZIN;
            typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
            typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;
            
            typedef pcl::PointNormal PointNormal;
            typedef pcl::PointCloud<PointNormal> CloudNormal;
            typedef pcl::PointCloud<PointNormal>::Ptr CloudNormalPtr;
            void process();
    private:
            // callback
            void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr&);
            void print4x4Matrix (const Eigen::Matrix4d &);
            geometry_msgs::Quaternion quat_eigen_to_msg(Eigen::Quaterniond);
            // void odom_callback(const nav_msgs::Odometry::ConstPtr&);

           int store_num;
           bool first_flag;
           double store_num_dynamic;

            // nodehandle
            ros::NodeHandle private_nh;
            ros::NodeHandle nh;
           // subscriber
            ros::Subscriber sub_cloud;
            // publisher
            ros::Publisher pub_cloud;
            ros::Publisher pub_pose;
            std::string TARGET_FRAME;
            sensor_msgs::PointCloud2 sq_cloud;
        //     nav_msgs::Odometry current_odom;
        //     nav_msgs::Odometry last_odom;
            Eigen::Vector2d last_odom_position;
            std::vector<Eigen::Vector3d> vehicle_velocities;
            CloudNormalPtr last_cloud;
};

#endif
