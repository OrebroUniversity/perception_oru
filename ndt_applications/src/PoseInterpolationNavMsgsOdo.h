#ifndef POSE_INTERPOLATION_NAV_MSG_ODO
#define POSE_INTERPOLATION_NAV_MSG_ODO


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <tf/message_filter.h>
#include <string>
#include <vector>
#include <pcl/ros/conversions.h>
/**
* This makes an offline interpolation for poses using tf::transform
*  This reads the whole file into a vector and after this you can ask poses based on timestamps
*
*/

class PoseInterpolationNavMsgsOdo{
	public:
		/**
		* Constructor
		* @param bagfilename the name of the bagfile
		* @param tftopic the name of the topic for tf like "/tf"
		* @param fixedframe The name of the fixed frame (e.g. "/world")
		* @param maxduration The maximum duration of bag file in secs (the buffer for tftransformer), default 3600
		* @param sensor_link Possibility to add a link, e.g. from base to sensor frame.
		*/ 
		PoseInterpolationNavMsgsOdo(std::string bagfilename, std::string tftopic, 
																std::string _fixedframe, 
																ros::Duration dur = ros::Duration(180), 
																tf::StampedTransform *sensor_link=NULL):
																
																transformer( true, dur)
		{
			bag.open(bagfilename, rosbag::bagmode::Read);
			topics.push_back(tftopic);
			fixedframe = _fixedframe;
			sensor_link_ = sensor_link;
			viewer = new rosbag::View(bag, rosbag::TopicQuery(topics));
			I = viewer->begin();
		}
		
		
		/**
		* Constructor
		* @param bagfilename the name of the bagfile
		* @param tftopic the name of the topic for tf like "/tf"
		* @param fixedframe The name of the fixed frame (e.g. "/world")
		* @param maxduration The maximum duration of bag file in secs (the buffer for tftransformer), default 3600
		* @param sensor_link Possibility to add a link, e.g. from base to sensor frame.
		*/ 
		PoseInterpolationNavMsgsOdo(rosbag::View *view,
																std::string tftopic, 
																std::string _fixedframe, 
																ros::Duration dur = ros::Duration(180), 
																tf::StampedTransform *sensor_link=NULL):																
																transformer( true, dur)
		{
			topics.push_back(tftopic);
			fixedframe = _fixedframe;
			sensor_link_ = sensor_link;
			viewer = view;
			I = viewer->begin();
		}
		
		~PoseInterpolationNavMsgsOdo(){
			
		}
		
		
		/**
		* Reads everything to cache (SLOW to use)
		*/
		void readBagFile(tf::StampedTransform *sensor_link=NULL);
		
		/**
		* Reads up until given time
		*/
		void readUntilTime(ros::Time t);
		
		/**
		* Returns the interpolated Affine transformation for a Time t1
		* @param t0 The time that acts as a reference for computing differential motion
		* @param t1 The time (after t0) that is estimated for T
		* @param frame_id The id of the frame that you want to use (e.g. "/odom")
		* @param &T Output as Eigen::Affine3d 
		*/
		bool getTransformationForTime(ros::Time t0,ros::Time t1, std::string frame_id,Eigen::Affine3d &T);
		
		/**
		* Returns the global pose for the t0
		*/		
		bool getTransformationForTime(ros::Time t0, std::string frame_id,tf::Transform &T);
		/**
		* Returns the interpolated tf transformation for a Time t1
		* @param t0 The time that acts as a reference for computing differential motion
		* @param t1 The time (after t0) that is estimated for T
		* @param frame_id The id of the frame that you want to use (e.g. "/odom")
		* @param &T Output as tf::Transform 
		*/
		bool getTransformationForTime(ros::Time t0,ros::Time t1, std::string frame_id,tf::Transform &T);
		
	private:
		rosbag::Bag bag;
		std::vector<std::string> topics;
		tf::Transformer transformer;
		std::string fixedframe;
		tf::StampedTransform *sensor_link_;
		rosbag::View *viewer; 
		rosbag::View::iterator I; 
		void TransformTFToEigen(const tf::Transform &t, Eigen::Affine3d &k);
		ros::Time last_read_tf;
};


#endif
