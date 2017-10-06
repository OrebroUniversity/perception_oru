#ifndef ROS_FUSER_INIT_03042017
#define ROS_FUSER_INIT_03042017

#include <tf/transform_listener.h>
#include "ndt_fuser/ndt_fuser_hmt.h"

namespace perception_oru{

namespace ndt_fuser{
	
	/**
	* @brief Initialise the sensor pose use tf frame
	* @param[in] fuser : the fuser to intialise
	*/
	void initSensorPose(perception_oru::NDTFuserHMT& fuser, const std::string& robot_frame, const std::string& sensor_frame);
	
	/**
	* @brief Initialise the robot pose use tf frame
	*/
	void initRobotPose(perception_oru::NDTFuserHMT& fuser, pcl::PointCloud< pcl::PointXYZ >& cloud, std::string& world_frame, std::string& robot_frame, bool preLoad=false);

	/**
	* @brief Initialise the sensor pose use tf frame. Any information on the z axis is discarded
	*/
	void initSensorPose2D(perception_oru::NDTFuserHMT& fuser, const std::string& robot_frame, const std::string& sensor_frame);
	
	/**
	* @brief Initialise the robot pose use tf frame. Any information on the z axis is discarded
	*/
	void initRobotPose2D(perception_oru::NDTFuserHMT& fuser, pcl::PointCloud< pcl::PointXYZ >& cloud, std::string& world_frame, std::string& robot_frame, bool preLoad=false);
}	
	
}

#endif

