#include "sq_scan_matcher/icp_sq_scan_matcher.h"
IcpSqScanMather::IcpSqScanMather() : private_nh("~")
{
	// private_nh.param("stepsize", stepsize, 0.1);
	// private_nh.param("resolution", resolution, 0.1);
	// private_nh.param("leafsize_target", leafsize_target, 0.1);
	private_nh.param("leafsize_source", leafsize_source, 0.1);
	private_nh.param("pc_range", pc_range, 100.0);
	private_nh.param("trans_epsilon", trans_epsilon, 0.001);
	private_nh.param("max_iterations", max_iterations, 100);
	private_nh.param("correspondence_distance", correspondence_distance, 0.1);
	private_nh.param("matching_score_th", matching_score_th, 0.1);
  
	/*subscriber*/
	sub_pc = nh.subscribe("/velodyne_points", 1, &IcpSqScanMather::cloud_callback, this);
	sub_pose = nh.subscribe("/ekf/pose", 1, &IcpSqScanMather::pose_callback, this);
  /*publisher*/
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/scan_matching/pose", 1);
	std::cout << "========start icp_sq_scan_matcher!!!!!!==========" << std::endl;
}
void IcpSqScanMather::pose_callback(const geometry_msgs::PoseStampedConstPtr& msg_pose)
{
	pose_ekf = *msg_pose;
	first_callback_pose = false;
}
void IcpSqScanMather::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg_cloud)
{
	// first_callback_pose = true;
	ros::Time t_start = ros::Time::now();
	pcl::fromROSMsg(*msg_cloud, *pc_now);
	if(!first_callback_pose){ // ekf_poseがあるかどうか
    initial_registration();
		if(transformation());
	}
	double dt = (ros::Time::now() - t_start).toSec();
	std::cout << "time[s]: " << dt << std::endl;
	*pc_last = *pc_now;
	// *pc_last = *pc_now_filtered;
}
void IcpSqScanMather::initial_registration(void)
{
	/*transform*/
	Eigen::Vector3f offset(
		pose_ekf.pose.position.x,
		pose_ekf.pose.position.y,
		pose_ekf.pose.position.z
	);  
	Eigen::Quaternionf rotation(
		pose_ekf.pose.orientation.w,
		pose_ekf.pose.orientation.x,
		pose_ekf.pose.orientation.y,
		pose_ekf.pose.orientation.z
	);
	pcl::transformPointCloud(*pc_now, *pc_map, offset, rotation);
}

bool IcpSqScanMather::transformation(void)
{
	double time_start = ros::Time::now().toSec();
	/*initialize*/
	pcl::IterativeClosestPointNonLinear<PointXYZI, PointXYZI> icp;
	/*filtering*/
	std::vector<double> range_local{
		-pc_range,
		pc_range,
		-pc_range,
		pc_range
	};
	// PassThroughFilter(pc_now, pc_now_filtered, range_local); // scan点群の整理
	// std::vector<double> range_global{
	// 	pose_ekf.pose.position.x - pc_range,
	// 	pose_ekf.pose.position.x + pc_range,
	// 	pose_ekf.pose.position.y - pc_range,
	// 	pose_ekf.pose.position.y + pc_range
	// };
	
	/*downsampling*/
	// Downsampling(pc_now_filtered, leafsize_source);
	
  /*drop out*/
	if(pc_now->points.empty()) return false;

	/*set parameters*/
	icp.setTransformationEpsilon(trans_epsilon);
	icp.setMaximumIterations(max_iterations); // 繰り返し最大数
	icp.setMaxCorrespondenceDistance(correspondence_distance);
	// icp.setPointRepresentation(pcl::make_shared<const MyPointRepresentation>(point_representation));
	
	/*set cloud*/
	icp.setInputSource(pc_now);
	// icp.setInputSource(pc_now_filtered);
	icp.setInputTarget(pc_last);

	/*initial guess*/
  // EKFによる推定姿勢を入力して、この姿勢からどれくらい移動した場所にいるかをmatchingで推定する
	Eigen::Translation3f init_translation(
		(float)pose_ekf.pose.position.x,
		(float)pose_ekf.pose.position.y,
		(float)pose_ekf.pose.position.z
	);
	Eigen::AngleAxisf init_rotation(
		quat_msg_to_eigen(pose_ekf.pose.orientation)
	);
	std::cout << "init_translation = (" << init_translation.x() << ", " << init_translation.y() << ", " << init_translation.z() << ")" << std::endl; 
	std::cout << "init_rotation : (" << init_rotation.axis()(0) << ", " << init_rotation.axis()(1) << ", " << init_rotation.axis()(2) << "), " << init_rotation.angle() << " [rad]" << std::endl; 
	Eigen::Matrix4f init_guess = (init_translation*init_rotation).matrix();
	/*drop out*/
	// if(pc_now_filtered->points.size() > pc_map_filtered->points.size()){
	// 	pcl::transformPointCloud (*pc_now_filtered, *pc_now_filtered, init_guess);
	// 	*pc_map += *pc_now_filtered;
	// 	return false;
	// }
	/*align*/
	std::cout << "aligning ..." << std::endl;
	icp.align(*pc_trans, init_guess);
	std::cout << "DONE" << std::endl;

	/*drop out*/
	if(!icp.hasConverged())	{
		std::cout << "has not converged!!!" << std::endl;
		return false;
	}
	/*print*/
	std::cout << "ICP has converged:" << icp.hasConverged ()  << std::endl << " score: " << icp.getFitnessScore () << std::endl;
	std::cout << "icp.getFinalTransformation()" << std::endl << icp.getFinalTransformation() << std::endl;
	std::cout << "init_guess" << std::endl << init_guess << std::endl;

	// if(icp.getFitnessScore() <= matching_score_th){
		/*input*/
		Eigen::Matrix4f T = icp.getFinalTransformation();
		Eigen::Matrix3f R = T.block(0, 0, 3, 3);
		Eigen::Quaternionf q_rot(R);
		q_rot.normalize();
		pose_icp.pose.position.x = T(0, 3);
		pose_icp.pose.position.y = T(1, 3);
		pose_icp.pose.position.z = T(2, 3);
		pose_icp.pose.orientation = quat_eigen_to_msg(q_rot);
		publication();
	// }
	std::cout << "transformation time [s] = " << ros::Time::now().toSec() - time_start << std::endl;

	return true;
}
void IcpSqScanMather::Downsampling(CloudXYZIPtr pc, double leafsize)
{
	CloudXYZIPtr tmp (new CloudXYZI);
	pcl::VoxelGrid<PointXYZI> vg;
	vg.setInputCloud(pc);
	vg.setLeafSize((float)leafsize, (float)leafsize, (float)leafsize);
	vg.filter(*tmp);
	*pc = *tmp;
}
void IcpSqScanMather::PassThroughFilter(CloudXYZIPtr pc_in, CloudXYZIPtr pc_out, std::vector<double> range)
{
	pcl::PassThrough<PointXYZI> pass;
	pass.setInputCloud(pc_in);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(range[0], range[1]);
	pass.filter(*pc_out);
	pass.setInputCloud(pc_out);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(range[2], range[3]);
	pass.filter(*pc_out);
}

void IcpSqScanMather::publication(void)
{
	/*publish*/
	pose_icp.header.stamp = pose_ekf.header.stamp;
	// pose_icp.header.frame_id = pose_ekf.header.frame_id;
	pose_icp.header.frame_id = "map";
	pub_pose.publish(pose_icp);
}
Eigen::Quaternionf IcpSqScanMather::quat_msg_to_eigen(geometry_msgs::Quaternion q_msg)
{
	Eigen::Quaternionf q_eigen(
		(float)q_msg.w,
		(float)q_msg.x,
		(float)q_msg.y,
		(float)q_msg.z
	);
	q_eigen.normalize();
	return q_eigen;
}
geometry_msgs::Quaternion IcpSqScanMather::quat_eigen_to_msg(Eigen::Quaternionf q)
{
	geometry_msgs::Quaternion msg;
	msg.x = (double)q.x();
	msg.y = (double)q.y();
	msg.z = (double)q.z();
	msg.w = (double)q.w();

	return msg;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_sq_scan_matcher");
    IcpSqScanMather icp_sq_scan_matcher;
    ros::spin();
    return 0;
}