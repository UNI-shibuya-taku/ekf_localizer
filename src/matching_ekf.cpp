#include <sq_scan_matcher/matching_ekf.h>
ScanMatchingEKF::ScanMatchingEKF() : nhPrivate("~")
{
	/*parameters*/
	nhPrivate.param("init_x", init_x, {0.0});
	nhPrivate.param("init_y", init_y, {0.0});
	nhPrivate.param("init_z", init_z, {0.0});
	nhPrivate.param("init_roll", init_roll, {0.0});
	nhPrivate.param("init_pitch", init_pitch, {0.0});
	nhPrivate.param("init_yaw", init_yaw, {0.0});
	nhPrivate.param("child_frame_name", child_frame_name, std::string("/velodyne"));
	nhPrivate.param("parent_frame_name", parent_frame_name, std::string("/odom"));
	nhPrivate.param("sigma_imu", sigma_imu, {1e-5});
	nhPrivate.param("sigma_odom", sigma_odom, {1e-5});
	nhPrivate.param("sigma_ndt", sigma_ndt, {1e-1});
	nhPrivate.param("odom_flag", odom_flag, {true});

	/*subscribe*/
	sub_imu = nh.subscribe("/imu/data", 1, &ScanMatchingEKF::imu_callback, this);
	sub_odom = nh.subscribe("/odom", 1, &ScanMatchingEKF::odom_callback, this);
	sub_pose = nh.subscribe("/scan_matching/pose", 1, &ScanMatchingEKF::pose_callback, this);
	/*publish*/
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/ekf/pose", 1);

	/*state*/
	X = Eigen::VectorXd::Zero(size_robot_state);
	const double initial_sigma = 1.0e-100;
	P = initial_sigma*Eigen::MatrixXd::Identity(size_robot_state, size_robot_state);
	initialize(init_x, init_y, init_z, init_roll, init_pitch, init_yaw);
	/*print*/
	std::cout << "child_frame_name = " << child_frame_name << std::endl;
	std::cout << "parent_frame_name = " << parent_frame_name << std::endl;
	std::cout << "sigma_imu = " << sigma_imu << std::endl;
	std::cout << "sigma_odom = " << sigma_odom << std::endl;
	std::cout << "sigma_ndt = " << sigma_ndt << std::endl;
}
void ScanMatchingEKF::initialize(double x,double y,double z,double roll,double pitch,double yaw)
{
	std::cout << "init pose X" << std::endl;
	X(0) = x;
	X(1) = y;
	X(2) = z;
	X(3) = roll;
	X(4) = pitch;
	X(5) = yaw;
	std::cout << X << std::endl;
}
void ScanMatchingEKF::imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
	time_publish = msg->header.stamp;
	time_imu_now = msg->header.stamp;
	double dt;
	try{
		dt = (time_imu_now - time_imu_last).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
	}
	time_imu_last = time_imu_now;
	if(first_callback_imu)	dt = 0.0;
	else	imu_prediction(*msg, dt);
	
	Publication();

	first_callback_imu = false;
}

void ScanMatchingEKF::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
	time_publish = msg->header.stamp;
	time_odom_now = msg->header.stamp;
	double dt;
	try{
		dt = (time_odom_now - time_odom_last).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
	}
	time_odom_last = time_odom_now;
	if(first_callback_odom)	dt = 0.0;
	else	odom_prediction(*msg, dt);
	
	Publication();

	first_callback_odom = false;
}
// 回転担当
void ScanMatchingEKF::imu_prediction(sensor_msgs::Imu imu, double dt)
{
	double r_ = X(3);
	double p_ = X(4);
	double y_ = X(5);

	double delta_r = imu.angular_velocity.x*dt;
	double delta_p = imu.angular_velocity.y*dt;
	double delta_y = imu.angular_velocity.z*dt;
	Eigen::Vector3d Drpy = {delta_r, delta_p, delta_y};
	
	// IMU情報で回転行列を生成
	Eigen::Matrix3d Rot_rpy;	//normal rotation
	Rot_rpy <<	1,	sin(r_)*tan(p_),	cos(r_)*tan(p_),
							0,	cos(r_)					,		-sin(r_),
							0,	sin(r_)/cos(p_) ,	cos(r_)/cos(p_);

	/*F*/
	Eigen::VectorXd F(X.size());
	F.segment(0, 3) = X.segment(0, 3);
	F.segment(3, 3) = X.segment(3, 3) + Rot_rpy*Drpy;
	for(int i=3;i<6;i++)	F(i) = PiToPi(F(i));

	/*jF*/
	Eigen::MatrixXd jF = Eigen::MatrixXd::Zero(X.size(), X.size());
	/*jF-xyz*/
	jF.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
	jF.block(0, 3, 3, 3) = Eigen::Matrix3d::Zero();
	/*jF-rpy*/
	jF.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
	jF(3, 3) = 1 + cos(r_)*tan(p_)*delta_p - sin(r_)*tan(p_)*delta_y;
	jF(3, 4) = sin(r_)/cos(p_)/cos(p_)*delta_p + cos(r_)/cos(p_)/cos(p_)*delta_y;
	jF(3, 5) = 0;
	jF(4, 3) = -sin(r_)*delta_p - cos(r_)*delta_y;
	jF(4, 4) = 1;
	jF(4, 5) = 0;
	jF(5, 3) = cos(r_)/cos(p_)*delta_p - sin(r_)/cos(p_)*delta_y;
	jF(5, 4) = sin(r_)*sin(p_)/cos(p_)/cos(p_)*delta_p + cos(r_)*sin(p_)/cos(p_)/cos(p_)*delta_y;
	jF(5, 5) = 1;
	
	/*Q*/
	Eigen::MatrixXd Q = sigma_imu*Eigen::MatrixXd::Identity(X.size(), X.size());
	Q.block(0, 0, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
	
	/*Update*/
	X = F;
	P = jF*P*jF.transpose() + Q;
}
// 並進担当
void ScanMatchingEKF::odom_prediction(nav_msgs::Odometry odom, double dt)
{
	double r_ = X(3);
	double p_ = X(4);
	double y_ = X(5);
	Eigen::Vector3d Dxyz = {odom.twist.twist.linear.x*dt, 0, 0}; // delta position

	/*F*/
	Eigen::VectorXd F(X.size());
	F.segment(0, 3) = X.segment(0, 3) + GetRotationXYZMatrix(X.segment(3, 3), false)*Dxyz;
	F.segment(3, 3) = X.segment(3, 3);

	/*jF*/
	Eigen::MatrixXd jF = Eigen::MatrixXd::Zero(X.size(), X.size());
	/*jF-xyz*/
	jF.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
	jF(0, 3) = Dxyz(1)*(cos(r_)*sin(p_)*cos(y_) + sin(r_)*sin(y_)) + Dxyz(2)*(-sin(r_)*sin(p_)*cos(y_) + cos(r_)*sin(y_));
	jF(0, 4) = Dxyz(0)*(-sin(p_)*cos(y_)) + Dxyz(1)*(sin(r_)*cos(p_)*cos(y_)) + Dxyz(2)*(cos(r_)*cos(p_)*cos(y_));
	jF(0, 5) = Dxyz(0)*(-cos(p_)*sin(y_)) + Dxyz(1)*(-sin(r_)*sin(p_)*sin(y_) - cos(r_)*cos(y_)) + Dxyz(2)*(-cos(r_)*sin(p_)*sin(y_) + sin(r_)*cos(y_));
	jF(1, 3) = Dxyz(1)*(cos(r_)*sin(p_)*sin(y_) - sin(r_)*cos(y_)) + Dxyz(2)*(-sin(r_)*sin(p_)*sin(y_) - cos(r_)*cos(y_));
	jF(1, 4) = Dxyz(0)*(-sin(p_)*sin(y_)) + Dxyz(1)*(sin(r_)*cos(p_)*sin(y_)) + Dxyz(2)*(cos(r_)*cos(p_)*sin(y_));
	jF(1, 5) = Dxyz(0)*(cos(p_)*cos(y_)) + Dxyz(1)*(sin(r_)*sin(p_)*cos(y_) - cos(r_)*sin(y_)) + Dxyz(2)*(cos(r_)*sin(p_)*cos(y_) + sin(r_)*sin(y_));
	jF(2, 3) = Dxyz(1)*(cos(r_)*cos(p_)) + Dxyz(2)*(-sin(r_)*cos(p_)) ;
	jF(2, 4) = Dxyz(0)*(-cos(p_)) + Dxyz(1)*(-sin(r_)*sin(p_)) + Dxyz(2)*(-cos(r_)*sin(p_)) ;
	jF(2, 5) = 0;
	/*jF-rpy*/
	jF.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
	jF.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();

	/*Q*/
	Eigen::MatrixXd Q = sigma_odom*Eigen::MatrixXd::Identity(X.size(), X.size());
	Q.block(3, 3, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
	
	/*Update*/
	X = F;
	P = jF*P*jF.transpose() + Q;
}
// icp matchingで得られた姿勢情報で補正
void ScanMatchingEKF::pose_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
	time_publish = msg->header.stamp;
	
	pose_observation(*msg);
	
	Publication();

	first_callback_odom = false;
}

void ScanMatchingEKF::pose_observation(geometry_msgs::PoseStamped pose)
{
	double r_, p_, y_;
	tf::Quaternion q_orientation;
	quaternionMsgToTF(pose.pose.orientation, q_orientation);
	tf::Matrix3x3(q_orientation).getRPY(r_, p_, y_);

	Eigen::VectorXd Z(6);
	Z <<	pose.pose.position.x,
		pose.pose.position.y,
		pose.pose.position.z,
		r_,
		p_,
		y_;
	Eigen::VectorXd Zp = X;
	Eigen::MatrixXd jH = Eigen::MatrixXd::Identity(Z.size(), X.size());
	Eigen::VectorXd Y = Z - Zp;
	Eigen::MatrixXd R = sigma_ndt*Eigen::MatrixXd::Identity(Z.size(), Z.size());
	Eigen::MatrixXd S = jH*P*jH.transpose() + R;
	Eigen::MatrixXd K = P*jH.transpose()*S.inverse();
	X = X + K*Y;
	for(int i=3;i<6;i++)	X(i) = PiToPi(X(i));
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(X.size(), X.size());
	P = (I - K*jH)*P;
}

void ScanMatchingEKF::Publication(void)
{
	/*pose*/
	geometry_msgs::PoseStamped pose_pub = StateVectorToPoseStamped();
	// pose_pub.header.frame_id = parent_frame_name;
	pose_pub.header.frame_id = "map";
	pose_pub.header.stamp = time_publish;
	pub_pose.publish(pose_pub);

	/*tf broadcast*/
// geometry_msgs::TransformStamped transform;
// 	transform.header.stamp = time_publish;
// 	transform.header.frame_id = parent_frame_name;
// 	transform.child_frame_id = child_frame_name;
// 	transform.transform.translation.x = pose_pub.pose.position.x;
// 	transform.transform.translation.y = pose_pub.pose.position.y;
// 	transform.transform.translation.z = pose_pub.pose.position.z;
// 	transform.transform.rotation = pose_pub.pose.orientation;
// 	tf_broadcaster.sendTransform(transform);
}

geometry_msgs::PoseStamped ScanMatchingEKF::StateVectorToPoseStamped(void)
{
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = X(0);
	pose.pose.position.y = X(1);
	pose.pose.position.z = X(2);
	tf::Quaternion q_orientation = tf::createQuaternionFromRPY(X(3), X(4), X(5));
	pose.pose.orientation.x = q_orientation.x();
	pose.pose.orientation.y = q_orientation.y();
	pose.pose.orientation.z = q_orientation.z();
	pose.pose.orientation.w = q_orientation.w();

	return pose;
}

Eigen::Matrix3d ScanMatchingEKF::GetRotationXYZMatrix(const Eigen::Vector3d& RPY, bool inverse)
{
	Eigen::Matrix3d Rot_xyz;
	Rot_xyz <<
		cos(RPY(1))*cos(RPY(2)),
			sin(RPY(0))*sin(RPY(1))*cos(RPY(2)) - cos(RPY(0))*sin(RPY(2)),
			cos(RPY(0))*sin(RPY(1))*cos(RPY(2)) + sin(RPY(0))*sin(RPY(2)),
		cos(RPY(1))*sin(RPY(2)),
			sin(RPY(0))*sin(RPY(1))*sin(RPY(2)) + cos(RPY(0))*cos(RPY(2)),
			cos(RPY(0))*sin(RPY(1))*sin(RPY(2)) - sin(RPY(0))*cos(RPY(2)),
		-sin(RPY(1)),
			sin(RPY(0))*cos(RPY(1)),
			cos(RPY(0))*cos(RPY(1));
	
	Eigen::Matrix3d Rot_xyz_inv;
	Rot_xyz_inv <<
		cos(RPY(1))*cos(RPY(2)),
			cos(RPY(1))*sin(RPY(2)),
			-sin(RPY(1)),
		sin(RPY(0))*sin(RPY(1))*cos(RPY(2)) - cos(RPY(0))*sin(RPY(2)),
			sin(RPY(0))*sin(RPY(1))*sin(RPY(2)) + cos(RPY(0))*cos(RPY(2)),
			sin(RPY(0))*cos(RPY(1)),
		cos(RPY(0))*sin(RPY(1))*cos(RPY(2)) + sin(RPY(0))*sin(RPY(2)),
			cos(RPY(0))*sin(RPY(1))*sin(RPY(2)) - sin(RPY(0))*cos(RPY(2)),
			cos(RPY(0))*cos(RPY(1));

	if(!inverse)	return Rot_xyz;
	else	return Rot_xyz_inv;	//=Rot_xyz.transpose()
}

double ScanMatchingEKF::PiToPi(double angle)
{
	return atan2(sin(angle), cos(angle)); 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "scan_matching_ekf");
	std::cout << "Planar Landmark EKF" << std::endl;
	ScanMatchingEKF scan_matching_ekf;
	ros::spin();
}