#include <sq_scan_matcher/imu_matching_ekf.h>
ImuMatchingEKF::ImuMatchingEKF() : nhPrivate("~")
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

	sub_imu = nh.subscribe("/imu/data", 1, &ImuMatchingEKF::imu_callback, this);
	// sub_odom = nh.subscribe("/odom", 1, &ImuMatchingEKF::odom_callback, this);
	sub_pose = nh.subscribe("/scan_matching/pose", 1, &ImuMatchingEKF::pose_callback, this);
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/ekf/pose", 1);

	X = Eigen::VectorXd::Zero(size_robot_state);
	const double initial_sigma = 1.0e-100;
	P = initial_sigma*Eigen::MatrixXd::Identity(size_robot_state, size_robot_state);
	initialize(init_x, init_y, init_z, init_roll, init_pitch, init_yaw);
	
	std::cout << "child_frame_name = " << child_frame_name << std::endl;
	std::cout << "parent_frame_name = " << parent_frame_name << std::endl;
	std::cout << "sigma_imu = " << sigma_imu << std::endl;
	std::cout << "sigma_odom = " << sigma_odom << std::endl;
	std::cout << "sigma_ndt = " << sigma_ndt << std::endl;
}
void ImuMatchingEKF::initialize(double x,double y,double z,double roll,double pitch,double yaw)
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
void ImuMatchingEKF::imu_callback(const sensor_msgs::ImuConstPtr& msg)
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
	else{
		imu_prediction(*msg, dt);
	}
	Publication();

	first_callback_imu = false;
	last_imu = *msg;
}
// 観測更新が来るまではimu単体で頑張る
// 観測更新が来たらIMU情報は誤差蓄積があるのでリセットする
void ImuMatchingEKF::imu_prediction(sensor_msgs::Imu imu, double dt)
{
	std::cout << "----------imu_prediction-----------" << std::endl;
	double r_ = X(3);
	double p_ = X(4);
	double y_ = X(5);

	double delta_r = imu.angular_velocity.x*dt;
	double delta_p = imu.angular_velocity.y*dt;
	double delta_y = imu.angular_velocity.z*dt;
	if(delta_r < 1e-3) delta_r = 0.0;
	if(delta_p < 1e-3) delta_p = 0.0;
	if(delta_y < 1e-3) delta_y = 0.0;
	Eigen::Vector3d Drpy = {delta_r, delta_p, delta_y};
	if(!first_callback_imu && pose_flag){ // 観測更新が行われた
		std::cout << "ICP estimate!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
		// 現在IMUの位置と１step前の位置でdelta_pose
		// current_pose += imu.linear_acceleration.x*dt*dt/2;
		// last_pose += last_imu.linear_acceleration.x*dt*dt/2;
		current_pose = icp_pose.pose.position.x;
		last_pose = icp_last_pose.pose.position.x;
		delta_pose = current_pose - last_pose;
		imu_velocity = delta_pose / dt;
		pose_flag = false;
	}else if(!first_callback_imu){ // 観測更新が行われなかったので、IMUのみで推定
		// 現在IMUの位置と１step前の位置でdelta_pose
		std::cout << "only IMU estimate" << std::endl;
		imu_velocity = imu.linear_acceleration.x*dt;
		current_pose = imu_velocity*dt + (imu.linear_acceleration.x - imu_bias)*dt*dt/2;
		last_pose = imu_velocity*dt +  (last_imu.linear_acceleration.x - imu_bias)*dt*dt/2;
		delta_pose = current_pose - last_pose;
	}
	std::cout << "current_pose: " << current_pose << std::endl;
	std::cout << "last_pose   : " << last_pose << std::endl;
	std::cout << "delta_pose: " << delta_pose << std::endl;

	// if(pose_flag){ // 蓄積移動量を一度リセットしたい
		// dt分進む量を算出
		// delta_pose = (icp_last_pose.pose.position.x - icp_pose.pose.position.x);
		// std::cout << "last pose: " << icp_last_pose << std::endl;
		// std::cout << "icp  pose: " << icp_pose << std::endl;
		// std::cout << "delta pose: " << delta_pose << std::endl;
		// imu_velocity = delta_pose / dt;
		// pose_flag = false;
	// }
	// else{
	// 	if(imu.linear_acceleration.x > 0.01){
	// 		imu_velocity = imu_velocity + imu.linear_acceleration.x*dt;
	// 		delta_pose = delta_pose + imu_velocity*dt + imu.linear_acceleration.x*dt*dt/2;
	// 	}
	// 	// if(imu.linear_acceleration.x < 1e-3) imu_velocity = 0.0;
	// 	// // 1.ローパスフィルターでノイズを取り除く
	// 	// // ローパスフィルター(現在の値 = 係数 * ひとつ前の値 ＋ センサの値 * (1 - 係数))
	// 	// lowpass_value = lowpass_value * filter_coefficient + imu.linear_acceleration.x * (1 - filter_coefficient);
	// 	// // 2.ハイパス・フィルターで重力加速度の影響を取り除く
	// 	// // ハイパスフィルター(センサの値 - ローパスフィルターの値)
	// 	// highpass_value = imu.linear_acceleration.x - lowpass_value;
	// 	// // 速度計算(加速度を台形積分する)
	// 	// imu_velocity += ((highpass_value + old_accel) * dt) / 2;
	// 	// old_accel = highpass_value;
	// 	// // 変位計算(速度を台形積分する)
	// 	// delta_pose += ((imu_velocity + old_speed) * dt) / 2;
	// 	// old_speed = imu_velocity;
	// }
	Eigen::Vector3d Dxyz = {delta_pose, 0, 0}; // delta position

	// IMU情報で回転行列を生成
	Eigen::Matrix3d Rot_rpy;	//normal rotation
	Rot_rpy <<	1,	sin(r_)*tan(p_),	cos(r_)*tan(p_),
										0,	cos(r_)						,		-sin(r_)					,
										0,	sin(r_)/cos(p_),	cos(r_)/cos(p_);

	/*F*/
	Eigen::VectorXd F(X.size());
	// F.segment(0, 3) = X.segment(0, 3);
	F.segment(0, 3) = X.segment(0, 3) + GetRotationXYZMatrix(X.segment(3, 3), false)*Dxyz;
	F.segment(3, 3) = X.segment(3, 3) + Rot_rpy*Drpy;
	for(int i = 3; i < 6; i++)	F(i) = PiToPi(F(i));

	/*jF*/
	Eigen::MatrixXd jF = Eigen::MatrixXd::Zero(X.size(), X.size());
	/*jF-xyz*/
	jF.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
	jF.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();
	jF.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
	jF.block(0, 3, 3, 3) = Eigen::Matrix3d::Zero();
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
	// Eigen::MatrixXd Q = sigma_imu*Eigen::MatrixXd::Identity(X.size(), X.size());
	Eigen::MatrixXd Q(X.size(), X.size());
	Q.setZero();
	Q.block(0, 0, 3, 3) = sigma_odom * Eigen::Matrix3d::Identity();
	Q.block(3, 3, 3, 3) = sigma_imu * Eigen::Matrix3d::Identity();

	// Q.block(0, 0, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
	// Eigen::MatrixXd Q = sigma_odom*Eigen::MatrixXd::Identity(X.size(), X.size());
	// Q.block(3, 3, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
	
	/*Update*/
	X = F;
	P = jF*P*jF.transpose() + Q;
}
// icp matchingで得られた姿勢情報で補正
void ImuMatchingEKF::pose_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
	icp_pose = *msg;
	time_publish = msg->header.stamp;
	
	pose_observation(*msg);
	
	Publication();

	first_callback_odom = false;
	pose_flag = true;
	icp_last_pose = icp_pose;
}

void ImuMatchingEKF::pose_observation(geometry_msgs::PoseStamped pose)
{
	std::cout << "----------pose_observation-----------" << std::endl;
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

void ImuMatchingEKF::Publication(void)
{
	/*pose*/
	geometry_msgs::PoseStamped pose_pub = StateVectorToPoseStamped();
	// pose_pub.header.frame_id = parent_frame_name;
	pose_pub.header.frame_id = "map";
	pose_pub.header.stamp = time_publish;
	pub_pose.publish(pose_pub);

	/*tf broadcast*/
	// geometry_msgs::TransformStamped transform;
	// transform.header.stamp = time_publish;
	// transform.header.frame_id = parent_frame_name;
	// transform.child_frame_id = child_frame_name;
	// transform.transform.translation.x = pose_pub.pose.position.x;
	// transform.transform.translation.y = pose_pub.pose.position.y;
	// transform.transform.translation.z = pose_pub.pose.position.z;
	// transform.transform.rotation = pose_pub.pose.orientation;
	// tf_broadcaster.sendTransform(transform);
}

geometry_msgs::PoseStamped ImuMatchingEKF::StateVectorToPoseStamped(void)
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

Eigen::Matrix3d ImuMatchingEKF::GetRotationXYZMatrix(const Eigen::Vector3d& RPY, bool inverse)
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

double ImuMatchingEKF::PiToPi(double angle)
{
	return atan2(sin(angle), cos(angle)); 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_matching_ekf");
	ImuMatchingEKF imu_matching_ekf;
	std::cout << "-------imu_matching_ekf!!!!!!----------" << std::endl;
	ros::spin();
}
// 並進担当
void ImuMatchingEKF::odom_prediction(nav_msgs::Odometry odom, double dt)
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
void ImuMatchingEKF::odom_callback(const nav_msgs::OdometryConstPtr& msg)
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