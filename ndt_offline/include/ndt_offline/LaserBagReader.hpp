#ifndef PERCEPTIONORU_LASERBARREADER_08052017
#define PERCEPTIONORU_LASERBARREADER_08052017
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <laser_geometry/laser_geometry.h>
// #include <velodyne_pointcloud/rawdata.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
// #include <velodyne_pointcloud/point_types.h>
#include <ndt_offline/PoseInterpolationNavMsgsOdo.h>
#include <iostream>
#include <pcl_ros/impl/transforms.hpp>

#include "ndt_offline/BagReaderInterface.hpp"

#ifdef READ_RMLD_MESSAGES
#include<SynchronizedRMLD.h>
#endif

namespace perception_oru{
	namespace ndt_offline{
		template<typename PointT>
		class LaserBagReader : public BagReaderInterface<PointT>{
			
		private:
// 			PoseInterpolationNavMsgsOdo *odosync;
// 			rosbag::Bag bag;
// 			rosbag::View *view;
// 			rosbag::View::iterator I;
// 			std::string velodynetopic_;
// 			std::string tf_pose_id_;
// 			sensor_msgs::LaserScan::ConstPtr global_scan;
// 			ros::Time timestamp_of_last_sensor_message;
// 			ros::Duration sensor_time_offset_;
			
			
		public:
			sensor_msgs::PointCloud2 last_pointcloud;
			/**
			* Constructor
			* @param calibration_file path and name to your velodyne calibration file
			* @param bagfilename The path and name of the bagfile you want to handle
			* @param velodynetopic The topic that contains velodyne_msgs/VelodyneScan
			* @param tf_pose_id The id of the tf that you want to use
			* @param fixed_frame_id The name of the fixed frame in tf (default = "/world")
			* @param tftopic name of tf (default "/tf")
			* @param dur The buffer size (must be larger than the length of the bag file gives) default = 3600s
			* @param sensor_link An optional static link that takes e.g. your /odom to the sensor frame 
			*/
			LaserBagReader(std::string calibration_file, 
				std::string bagfilename,
				std::string velodynetopic,
				std::string tf_base_link, 
				std::string tf_sensor_link,
				std::string fixed_frame_id="/world",
				std::string tftopic="/tf", 
				ros::Duration dur = ros::Duration(3600),
				tf::StampedTransform *sensor_link=NULL,
				double velodyne_max_range=130.0, 
				double velodyne_min_range=2.0,
				double sensor_time_offset=0.0) : BagReaderInterface<PointT>(calibration_file, bagfilename,velodynetopic, tf_base_link, tf_sensor_link, fixed_frame_id, tftopic, dur, sensor_link, velodyne_max_range, velodyne_min_range, sensor_time_offset){}
			
			
			void convertLaser(const sensor_msgs::LaserScan::ConstPtr laser, sensor_msgs::PointCloud2& cloud){
				laser_geometry::LaserProjection projector;
				projector.projectLaser(*laser, cloud);
			}
			
// 			/**
// 			* @param[in] NMeas : number of point cloud to stack
// 			* @param[out] cloud<PointT> : point cloud out at the end. pointT is the type of the cloud
// 			* @param[out] sensor_pose : sensor pose of measurement
// 			* @param[out] base_pose : base pose of measurement
// 			* @param[in] base_pose_id : base_pose frame name
// 			*/
// 			bool readMultipleMeasurements(unsigned int Nmeas, pcl::PointCloud<PointT> &cloud){
// 
// 				std::cout << "Initr rmm" << std::endl;
// 				assert(this->I != this->view->end());
// 				
// 				if(readMultipleMeasurements(Nmeas, cloud)){
// 					return true;
// 				}else{
// 					return false;
// 				}
// 				
// 			}
			
			/**
			* @param[in] NMeas : number of point cloud to stack
			* @param[out] cloud<PointT> : point cloud out at the end. pointT is the type of the cloud but it needs to be x,y,z even though it's templated.
			*/
			bool readMultipleMeasurements(unsigned int Nmeas, pcl::PointCloud<PointT> &cloud){

// 				std::cout << "Start read mmo" << std::endl;
				assert(this->I != this->view->end());
				
				tf::Transform T;
				ros::Time t0;
// 				velodyne_rawdata::VPointCloud pnts,conv_points;

				if(!this->getNextScanMsg()) return false;
				t0 = this->global_scan->header.stamp + this->sensor_time_offset_ ;
				this->timestamp_of_last_sensor_message = t0;
				
				ros::Time t1=this->global_scan->header.stamp + this->sensor_time_offset_;
				
// 				laser_geometry::LaserProjection projector;
// 				projector.projectLaser(*(this->global_scan), last_pointcloud);
// 				
// 				
// 				pcl::PointCloud<pcl::PointXYZ> pcl_cloud_unfiltered;
// 				pcl::fromROSMsg (last_pointcloud, pcl_cloud_unfiltered);
// 
// 				pcl::PointXYZ pt;
// 				//add some variance on z
// 				for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
// 					pt = pcl_cloud_unfiltered.points[i];
// 					if(sqrt(pt.x*pt.x+pt.y*pt.y) > 0.1) {
// 						pt.z += (0.2/4)*((double)rand())/(double)INT_MAX;
// 						cloud.points.push_back(pt);
// 					}
// 				}
				
				
// 				sensor_msgs::PointCloud2 cloud_msg;
				convertLaser(this->global_scan, last_pointcloud);
				pcl::PointCloud<pcl::PointXYZ> pcl_cloud_unfiltered;
				pcl::fromROSMsg (last_pointcloud, pcl_cloud_unfiltered);
				
				//add some variance on z <- NEEDED BUT WHY
				pcl::PointXYZ pt;
				for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
					pt = pcl_cloud_unfiltered.points[i];
					if(sqrt(pt.x*pt.x+pt.y*pt.y) > 0.1) {
						pt.z += (0.2/4)*((double)rand())/(double)INT_MAX;
						cloud.points.push_back(pt);
					}
				}

				return true;
			}
			
			
			
			
			
		};
	}
}

#endif