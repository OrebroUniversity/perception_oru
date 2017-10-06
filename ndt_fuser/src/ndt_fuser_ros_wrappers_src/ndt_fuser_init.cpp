#include "ndt_fuser/ndt_fuser_ros_wrappers/ros_fuser_init.hpp"


void perception_oru::ndt_fuser::initSensorPose(perception_oru::NDTFuserHMT& fuser, const std::string& robot_frame, const std::string& sensor_frame){
	tf::TransformListener listener;
	tf::StampedTransform transform;
	try {
		listener.waitForTransform(sensor_frame, robot_frame, ros::Time(0), ros::Duration(10.0) );
		listener.lookupTransform(sensor_frame, robot_frame, ros::Time(0), transform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}
	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();
	double z = transform.getOrigin().z();
	double roll, pitch, yaw;
	transform.getBasis().getRPY(roll, pitch, yaw);

	Eigen::Affine3d pose_sensor = Eigen::Translation<double,3>(x,y,z)*
	Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
	Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
	Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;
	
	std::cout << "POSE SENSOR " << pose_sensor.matrix() << std::endl;
// 	exit(0);
	
	
	fuser.setSensorPose(pose_sensor);

}

void perception_oru::ndt_fuser::initRobotPose(perception_oru::NDTFuserHMT& fuser, pcl::PointCloud< pcl::PointXYZ >& cloud, std::string& world_frame, std::string& robot_frame, bool preLoad)
{
	tf::TransformListener listener;
	tf::StampedTransform transform;
	try {
		listener.waitForTransform(robot_frame, world_frame, ros::Time(0), ros::Duration(1.0) );
		listener.lookupTransform(world_frame, robot_frame, ros::Time(0), transform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}
	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();
	double z = transform.getOrigin().z();
	double roll, pitch, yaw;
	transform.getBasis().getRPY(roll, pitch, yaw);

	Eigen::Affine3d pose = Eigen::Translation<double,3>(x,y,z)*
	Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
	Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
	Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;
	std::cout << "POSE ROBOT " << pose.matrix() << std::endl;

// 	exit(0);
	fuser.initialize(pose, cloud, preLoad);
}

void perception_oru::ndt_fuser::initSensorPose2D(perception_oru::NDTFuserHMT& fuser, const std::string& robot_frame, const std::string& sensor_frame){
	tf::TransformListener listener;
	tf::StampedTransform transform;
	try {
		listener.waitForTransform(sensor_frame, robot_frame, ros::Time(0), ros::Duration(10.0) );
		listener.lookupTransform(sensor_frame, robot_frame, ros::Time(0), transform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}
	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();
	double z = 0;
	double roll, pitch, yaw;
	transform.getBasis().getRPY(roll, pitch, yaw);
	

	Eigen::Affine3d pose_sensor = Eigen::Translation<double,3>(x,y,z)*
	Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
	Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
	Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;
	fuser.setSensorPose(pose_sensor);
	
	
	std::cout << "POSE SENSOR 2d " << pose_sensor.matrix() << std::endl;
// 	exit(0);

}

void perception_oru::ndt_fuser::initRobotPose2D(perception_oru::NDTFuserHMT& fuser, pcl::PointCloud< pcl::PointXYZ >& cloud, std::string& world_frame, std::string& robot_frame, bool preLoad)
{
	tf::TransformListener listener;
	tf::StampedTransform transform;
	try {
		listener.waitForTransform(robot_frame, world_frame, ros::Time(0), ros::Duration(1.0) );
		listener.lookupTransform(world_frame, robot_frame, ros::Time(0), transform);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}
	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();
	double z = 0;
	double roll, pitch, yaw;
	transform.getBasis().getRPY(roll, pitch, yaw);

	Eigen::Affine3d pose = Eigen::Translation<double,3>(x,y,z)*
	Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
	Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
	Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;
	std::cout << "POSE ROBOT 2d " << pose.matrix() << std::endl;
	exit(0);

	fuser.initialize(pose, cloud, preLoad);
}
