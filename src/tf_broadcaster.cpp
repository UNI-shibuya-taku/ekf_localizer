#include "tf/tf_broadcaster.h"

TFBroadcaster::TFBroadcaster() :
    private_nh_("~")
{
    private_nh_.param("is_odom_tf",is_odom_tf_,{true});
	private_nh_.param("odom_topic_name",odom_topic_name_,{"/odom"});
	private_nh_.param("odom_frame_id",odom_frame_id_,{"odom"});
	private_nh_.param("base_link_frame_id",base_link_frame_id_,{"base_link"});
	private_nh_.param("camera_frame_id",camera_frame_id_,{"camera_color_optical_frame"});
	private_nh_.param("velodyne_frame_id",velodyne_frame_id_,{"velodyne"});

    private_nh_.param("camera_x",camera_coordinate_.x,{0.0});
	private_nh_.param("camera_y",camera_coordinate_.y,{0.0});
	private_nh_.param("camera_z",camera_coordinate_.z,{0.5});
	private_nh_.param("camera_roll",camera_coordinate_.roll,{0.0});
	private_nh_.param("camera_pitch",camera_coordinate_.pitch,{0.0});
	private_nh_.param("camera_yaw",camera_coordinate_.yaw,{0.0});
	private_nh_.param("velodyne_x",velodyne_coordinate_.x,{0.0});
	private_nh_.param("velodyne_y",velodyne_coordinate_.y,{0.0});
	private_nh_.param("velodyne_z",velodyne_coordinate_.z,{1.3});
	private_nh_.param("velodyne_roll",velodyne_coordinate_.roll,{0.0});
	private_nh_.param("velodyne_pitch",velodyne_coordinate_.pitch,{0.0});
	private_nh_.param("velodyne_yaw",velodyne_coordinate_.yaw,{0.0});

    odom_sub_ = nh_.subscribe(odom_topic_name_,1,&TFBroadcaster::odometry_callback,this);
	
	camera_transform_ = create_transform_stamped_msg(base_link_frame_id_,camera_frame_id_,camera_coordinate_);
	velodyne_transform_ = create_transform_stamped_msg(base_link_frame_id_,velodyne_frame_id_,velodyne_coordinate_);

	broadcaster_.reset(new tf2_ros::TransformBroadcaster);
	static_broadcaster_.reset(new tf2_ros::StaticTransformBroadcaster);
}

TFBroadcaster::~TFBroadcaster() { }

void TFBroadcaster::odometry_callback(const nav_msgs::OdometryConstPtr& msg)
{
	if(is_odom_tf_){
		geometry_msgs::TransformStamped transform;
		transform.header = msg->header;
		transform.header.frame_id = odom_frame_id_;
 	   	transform.child_frame_id = base_link_frame_id_;

    	transform.transform.translation.x = msg->pose.pose.position.x;
    	transform.transform.translation.y = msg->pose.pose.position.y;
    	transform.transform.translation.z = msg->pose.pose.position.z;
    	transform.transform.rotation = msg->pose.pose.orientation;

		broadcaster_->sendTransform(transform);
	}
}

geometry_msgs::TransformStamped TFBroadcaster::create_transform_stamped_msg(std::string frame_id,std::string child_frame_id,Coordinate coordinate)
{
	geometry_msgs::TransformStamped static_transform_stamped;
	static_transform_stamped.header.stamp = ros::Time::now();
	static_transform_stamped.header.frame_id = frame_id;
	static_transform_stamped.child_frame_id = child_frame_id;

	static_transform_stamped.transform.translation.x = coordinate.x;
	static_transform_stamped.transform.translation.y = coordinate.y;
	static_transform_stamped.transform.translation.z = coordinate.z;

	tf2::Quaternion quaternion;
	quaternion.setRPY(coordinate.roll,coordinate.pitch,coordinate.yaw);
	static_transform_stamped.transform.rotation.x = quaternion.x();
	static_transform_stamped.transform.rotation.y = quaternion.y();
	static_transform_stamped.transform.rotation.z = quaternion.z();
	static_transform_stamped.transform.rotation.w = quaternion.w();

	return static_transform_stamped;
}

void TFBroadcaster::process()
{
	static_broadcaster_->sendTransform({ camera_transform_, velodyne_transform_ });
	ros::spin(); 
}