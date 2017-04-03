#ifndef ROS_FUSER_INIT_03042017
#define ROS_FUSER_INIT_03042017

#include <tf/transform_listener.h>
#include "ndt_fuser/ndt_fuser_hmt.h"

namespace perception_oru{

namespace ndt_fuser{
	
	void initSensorPose(lslgeneric::NDTFuserHMT& fuser, const std::string& robot_frame, const std::string& sensor_frame);
	
	void initRobotPose(lslgeneric::NDTFuserHMT& fuser, pcl::PointCloud< pcl::PointXYZ >& cloud, std::string& world_frame, std::string& robot_frame, bool preLoad=false);
}	
	
}

#endif

