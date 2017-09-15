#ifndef READBAGFILEGENERIC_H
#define READBAGFILEGENERIC_H

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
#include "VelodyneBagReader.h"

template<typename PointT>
class ReadBagFileGeneric{
public:
  ReadBagFileGeneric(const std::string &type,
                     const std::string velodyne_tf_interpolation_string,
                     std::string calibration_file,
                     std::string bagfilename,
                     const std::string &point_cloud_topic,
                     std::string tf_pose_id,
                     std::string fixed_frame_id="/world",
                     std::string tftopic="/tf",
                     ros::Duration dur = ros::Duration(3600),
                     tf::StampedTransform *sensor_link=NULL,
                     double velodyne_max_range=130.0,
                     double velodyne_min_range=2.0,
                     double sensor_time_offset=0.0,
                     int nb_measurements=1){
    type_=type;
    velodyne_tf_interpolation_string_=velodyne_tf_interpolation_string;
    nb_measurements_=nb_measurements;

    if(type.compare("pcl_reader")==0){
      preader_=new PointCloudBagReader<PointT>(calibration_file,bagfilename,point_cloud_topic,tf_pose_id,fixed_frame_id,
                                                      tftopic,dur,sensor_link,velodyne_max_range,velodyne_min_range,sensor_time_offset);
    }
      else{
        vreader_=new VelodyneBagReader<PointT>(calibration_file,bagfilename,point_cloud_topic,tf_pose_id,fixed_frame_id,
                                                      tftopic,dur,sensor_link,velodyne_max_range,velodyne_min_range,sensor_time_offset);
      }
    }
    bool ReadNextMeasurement(pcl::PointCloud<PointT> &cloud) {
      if(vreader_!=NULL){
        tf::Transform tf_scan_source;
        tf::Transform tf_gt_base;
        return vreader_->readMultipleMeasurements(nb_measurements_,cloud,tf_scan_source,tf_gt_base,velodyne_tf_interpolation_string_);//if velodyne reader is used, the cloud needs to be compensated for movement during scan
      }
      else if(preader_!=NULL)
        return preader_->readNextMeasurement(cloud);//if preader reader is used, the cloud was already adjusted for movement in theconversion.
    }
    bool getPoseFor(tf::Transform &pose, const std::string &pose_id){
      if(vreader_!=NULL)
        return  vreader_->getPoseFor(pose,pose_id);
      else if(preader_!=NULL)
        return  preader_->getPoseFor(pose,pose_id);
    }
    bool getPoseFor(Eigen::Affine3d &pose_out, const std::string &pose_id){
      tf::Transform pose;
      bool pose_succesfull=false;
      if(vreader_!=NULL){
        pose_succesfull=  vreader_->getPoseFor(pose,pose_id);
        tf::transformTFToEigen(pose,pose_out);
      }
      else if(preader_!=NULL){
        pose_succesfull=  preader_->getPoseFor(pose,pose_id);
        tf::transformTFToEigen(pose,pose_out);
      }
      return pose_succesfull;
    }
    ros::Time getTimeStampOfLastSensorMsg() const {
      if(vreader_!=NULL)
        return vreader_->getTimeStampOfLastSensorMsg();
      else if(preader_!=NULL)
        return preader_->getTimeStampOfLastSensorMsg();
      else
        exit(0);
    }

    private:
    VelodyneBagReader<PointT> *vreader_=NULL;
    PointCloudBagReader<PointT> *preader_=NULL;
    std::string velodyne_tf_interpolation_string_="";
    std::string type_="";
    int nb_measurements_;
  };

#endif // READBAGFILEGENERIC_H
