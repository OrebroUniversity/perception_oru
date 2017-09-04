#ifndef POINTCLOUDBAGREADER_H
#define POINTCLOUDBAGREADER_H


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>
#include <ndt_offline/PoseInterpolationNavMsgsOdo.h>
#include <iostream>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
template<typename PointT>
class PointCloudBagReader{
public:
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
  PointCloudBagReader(std::string calibration_file,
                           std::string bagfilename,
                           const std::string & point_cloud_topic,
                           std::string tf_pose_id,
                           std::string fixed_frame_id="/world",
                           std::string tftopic="/tf",
                           ros::Duration dur = ros::Duration(3600),
                           tf::StampedTransform *sensor_link=NULL,
                           double velodyne_max_range=130.0,
                           double velodyne_min_range=2.0,
                           double sensor_time_offset=0.0
      )

  {
    // The view_direction / view_width is important to have set. The min/max range is overwritten in the setupOffline

    double view_direction = 0;
    double view_width = 2*M_PI;
    dataParser.setParameters(velodyne_min_range,
                             velodyne_max_range,
                             view_direction,
                             view_width);

    dataParser.setupOffline(calibration_file, velodyne_max_range, velodyne_min_range);
    sensor_time_offset_ = ros::Duration(sensor_time_offset);
    fprintf(stderr,"Opening '%s'\n",bagfilename.c_str());

    bag.open(bagfilename, rosbag::bagmode::Read);

    point_cloud_topic_ = point_cloud_topic;
    tf_pose_id_ = tf_pose_id;

    std::vector<std::string> topics;
    topics.push_back(tftopic);
    topics.push_back(point_cloud_topic);


    for(int i=0; i<topics.size(); ++i) {
      fprintf(stderr,"Searched Topic [%d] = '%s'\n",i,topics[i].c_str());
    }

    view = new rosbag::View(bag, rosbag::TopicQuery(topics));
    I = view->begin();
    odosync = new PoseInterpolationNavMsgsOdo(view,tftopic, fixed_frame_id,dur, sensor_link);

    //odosync = NULL;
  }

  /**
   * Reads the next measurment.
   * @param cloud The generated point cloud
   * @param sensor_pose [out] The pose of the sensor origin. (Utilizes the tf_pose_id and sensor_link from the constructor).
   **/

  void sensorMessageToPclCloud(const sensor_msgs::PointCloudConstPtr &source, pcl::PointCloud<pcl::PointXYZ> &target){

    target.header.frame_id="/velodyne";
    target.header.seq=source->header.seq;
    pcl_conversions::toPCL(source->header.stamp,target.header.stamp);

    for(int i=0;i<source->points.size();i++){
      pcl::PointXYZ p;
      p.x=source->points[i].x;
      p.y=source->points[i].y;
      p.z=source->points[i].z;
      target.push_back(p);
    }
  }


    bool readNextMeasurement(pcl::PointCloud<PointT> &cloud){
      cloud.clear();
      bool found_scan=false;
      while(!found_scan){
        if(I == view->end()){
          fprintf(stderr,"End of measurement file Reached!!\n");
          return false;
        }
        rosbag::MessageInstance const m = *I;
        std::string str =m.getTopic();
        if(m.getTopic()==point_cloud_topic_){
          sensor_msgs::PointCloudConstPtr scan = m.instantiate<sensor_msgs::PointCloud>();
          if (scan != NULL){
            found_scan=true;
           for(int i=0;i<scan->points.size();i++){
           PointT p_target;
            geometry_msgs::Point32 p_src=  scan->points[i];
            p_target.x=p_src.x;
            p_target.y=p_src.y;
            p_target.z=p_src.z;
            cloud.push_back(p_target);
           }
           cloud.header.frame_id=scan->header.frame_id;
           pcl_conversions::toPCL(scan->header.stamp,cloud.header.stamp);

          //  cloud=*scan;
           timestamp_of_last_sensor_message=scan->header.stamp;
          }
        }
        I++;
      }
      return true;
    }
    void CloseOutputBag(){
      outbag.close();
    }


    /**
       * Get pose for latest measurement with pose id
       */
    bool getPoseFor(tf::Transform &pose, std::string pose_id){
      if(odosync->getTransformationForTime(timestamp_of_last_sensor_message,pose_id,pose)){
        return true;
      }
      return false;
    }


    ros::Time getTimeStampOfLastSensorMsg() const {
      return timestamp_of_last_sensor_message;
    }


    private:
    unsigned int counter=0;
    rosbag::Bag outbag;
    std::string outbag_name_;
    PoseInterpolationNavMsgsOdo *odosync;
    velodyne_rawdata::RawData dataParser;
    rosbag::Bag bag;
    rosbag::View *view;
    rosbag::View::iterator I;
    std::string point_cloud_topic_;
    std::string tf_pose_id_;
    velodyne_msgs::VelodyneScan::ConstPtr global_scan;
    ros::Time timestamp_of_last_sensor_message;
    ros::Duration sensor_time_offset_;


  };



#endif // POINTCLOUDBAGREADER_H
