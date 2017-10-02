#ifndef CONVERTVELODYNEBAGS_H
#define CONVERTVELODYNEBAGS_H
/**
* Reads a bag file that contains
* 1) Velodyne raw messages
* 2) tf messages (e.g. as odometry)
* This class reads the _whole_ tf to cache and uses this to sync the velodyne messages with the motion of the platform.
* The result is returned as pcl::PointCloud<PointXYZI> in the sensor coordinates
*
* The class uses Velodyne ros pkg, with one hack:

Add the following to rawdata.h:
#include <angles/angles.h>

int setupOffline(std::string calibration_file, double max_range_, double min_range_)
{

    config_.max_range = max_range_;
    config_.min_range = min_range_;
    ROS_INFO_STREAM("data ranges to publish: ["
  << config_.min_range << ", "
  << config_.max_range << "]");

    config_.calibrationFile = calibration_file;

    ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

    calibration_.read(config_.calibrationFile);
    if (!calibration_.initialized) {
  ROS_ERROR_STREAM("Unable to open calibration file: " <<
      config_.calibrationFile);
  return -1;
    }

    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
  float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
  cos_rot_table_[rot_index] = cosf(rotation);
  sin_rot_table_[rot_index] = sinf(rotation);
    }
    return 0;
}

*
* NOTE: In order for the synchronization of velodyne and vehicle motion to work
* you have to express the vehicle motion in the velodyne sensor coordinates.
* If your log file only contains odometry or similar for the vehicle you can give
* an extra link as a parameter (note that then give id to this extra link in constructor in this case).
*
* @author Jari Saarinen (jari.saarinen@aalto.fi)
*/

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
#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>
#include <ndt_offline/PoseInterpolationNavMsgsOdo.h>
#include <iostream>
#include <pcl_ros/impl/transforms.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <angles/angles.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ConvertVelodyneBagsToPcl{
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
  ConvertVelodyneBagsToPcl(std::string outbag_name,
                           std::string calibration_file,
                           std::string bagfilename,
                           std::string velodynetopic,
                           std::string tf_pose_id,
                           std::string fixed_frame_id="/world",
                           std::string tftopic="/tf",
                           ros::Duration dur = ros::Duration(3600),
                           tf::StampedTransform *sensor_link=NULL,
                           double velodyne_max_range=130.0,
                           double velodyne_min_range=2.0,
                           double sensor_time_offset=0.0,
                           int height = 64,
                           int width = 1000
      )

  {
    // The view_direction / view_width is important to have set. The min/max range is overwritten in the setupOffline
    outbag_name_=outbag_name;
    outbag.open(outbag_name,rosbag::bagmode::Write);
    double view_direction = 0;
    double view_width = 2*M_PI;
    dataParser.setParameters(velodyne_min_range,
                             velodyne_max_range,
                             view_direction,
                             view_width);

    if(dataParser.setupOffline(calibration_file, velodyne_max_range, velodyne_min_range)!=0)
      exit(0);
    sensor_time_offset_ = ros::Duration(sensor_time_offset);
    fprintf(stderr,"Opening '%s'\n",bagfilename.c_str());

    bag.open(bagfilename, rosbag::bagmode::Read);

    velodynetopic_ = velodynetopic;
    tf_pose_id_ = tf_pose_id;

    std::vector<std::string> topics;
    topics.push_back(tftopic);
    topics.push_back(velodynetopic_);
    topics.push_back("diagnostics");
    topics.push_back("rosout");
    topics.push_back("rosout_agg");
    topics.push_back("velodyne_nodelet_manager/bond");
    topics.push_back("/vmc_navserver/encoders");
    topics.push_back("/vmc_navserver/laserway");
    topics.push_back("/vmc_navserver/odom");
    topics.push_back("/vmc_navserver/state");
    topics.push_back("/wifi_sniffer/wlan0");
    topics.push_back("/wifi_sniffer/wlan1");


    for(int i=0; i<topics.size(); ++i) {
      fprintf(stderr,"Searched Topic [%d] = '%s'\n",i,topics[i].c_str());
    }

    view = new rosbag::View(bag, rosbag::TopicQuery(topics));
    I = view->begin();

    odosync = new PoseInterpolationNavMsgsOdo(view,tftopic, fixed_frame_id,dur, sensor_link);

    //odosync = NULL;2
    view_width_ = view_width;
    velodyne_max_range_ = velodyne_max_range;
    velodyne_min_range_ = velodyne_min_range;
    image_clean_ = true;
    depth_img_ = cv::Mat(height, width, CV_32FC1, 0.);
    intensity_img_ = cv::Mat(height, width, CV_32FC1, 0.);
    width_ = width;
    height_ = height;
  }

  /**
   * Reads the next measurment.
   * @param cloud The generated point cloud
   * @param sensor_pose [out] The pose of the sensor origin. (Utilizes the tf_pose_id and sensor_link from the constructor).
   **/
  bool ConvertToPclBag(tf::Transform &sensor_pose){
    if(I == view->end()){
      fprintf(stderr,"End of measurement file Reached!!\n");
      return false;
    }
    //if(odosync == NULL) return true;
    rosbag::MessageInstance const m = *I;
    if(m.getTopic()==velodynetopic_){

      velodyne_msgs::VelodyneScan::ConstPtr scan = m.instantiate<velodyne_msgs::VelodyneScan>();
      if (scan != NULL){
        sensor_msgs::PointCloudPtr cloud(new sensor_msgs::PointCloud);
        velodyne_rawdata::VPointCloud pnts,conv_points;
        // process each packet provided by the driver
        tf::Transform T;
        ros::Time t0=scan->header.stamp + sensor_time_offset_;
        timestamp_of_last_sensor_message=t0;
        if(odosync->getTransformationForTime(t0, tf_pose_id_, sensor_pose)){
          for (size_t next = 0; next < scan->packets.size(); ++next){
            dataParser.unpack(scan->packets[next], pnts); // unpack the raw data
            ros::Time t1=scan->packets[next].stamp + sensor_time_offset_;

            if(odosync->getTransformationForTime(t0,t1,tf_pose_id_,T)){
              pcl_ros::transformPointCloud(pnts,conv_points,T);
              for(size_t i = 0;i<pnts.size();i++){
                // PointT p;
                geometry_msgs::Point32 p;
                p.x = conv_points.points[i].x;
                p.y = conv_points.points[i].y;
                p.z = conv_points.points[i].z;
                cloud->points.push_back(p);
              }
            }else{
              //fprintf(stderr,"No transformation\n");
            }
            pnts.clear();
          }


        }else{
          fprintf(stderr,"No transformation\n");
        }

        /*  geometry_msgs::Point32 p;
        sensor_msgs::PointCloud cld;
        for(int i=0;i<cloud.size();i++){
          p.x=cloud[i].x;
          p.y=cloud[i].y;
          p.z=cloud[i].z;
          cld.points.push_back(p);*/
        std::cout<<"Frame:"<<++counter<<", size:"<<cloud->points.size()<<std::endl;
        cloud->header.frame_id="/velodyne";
        cloud->header.seq=counter;
        cloud->header.stamp=timestamp_of_last_sensor_message;
        outbag.write(m.getTopic() ,timestamp_of_last_sensor_message,cloud);

        //cld.header.frame_id=cloud.header.frame_id;
        //cld.header.stamp=timestamp_of_last_sensor_message;
        //outbag.write("/sensor_lidar",timestamp_of_last_sensor_message,  cld);
      }
    }
    else{
      // std::cout<<"wrinting topic="<<m.getTopic()<<std::endl;
      outbag.write(m.getTopic(),m.getTime(),  m);
    }


    I++;
    return true;
  }


  bool getNextScanMsg(){
    if(I == view->end()){
      fprintf(stderr,"End of measurement file Reached!!\n");
      return false;
    }
    bool done = false;
    while(!done){
      rosbag::MessageInstance const m = *I;

      {
        outbag.write(m.getTopic(),m.getTime(),  m);
      }
      global_scan = m.instantiate<velodyne_msgs::VelodyneScan>();


      I++;
      if(I == view->end()) done = true;
      if(global_scan != NULL){
        //                        fprintf(stderr,"GOT %s \n",m.getTopic().c_str());
        done = true;
      }
    }

    if(I == view->end()) {
      return false;
    }
    return true;
  }


  int addToImages(const velodyne_rawdata::VPointCloud &pnts, int width, int height, int startpixelx, int startpixely, const tf::Transform &T) {

    double resolution_factor = width / view_width_;
    for(size_t i = 0;i<pnts.size();i++){
      int y = height-pnts.points[i].ring-1;
      tf::Point pt;
pt.setValue(pnts.points[i].x - T.getOrigin()[0], pnts.points[i].y - T.getOrigin()[1], pnts.points[i].z - T.getOrigin()[2]);

      double th = atan2(pt.y(), pt.x());
      th = angles::normalize_angle_positive(th);
      int x = static_cast<int>(resolution_factor*th);

      float depth = sqrt(pt.x()*pt.x() + pt.y()*pt.y() + pt.z()*pt.z() );
      bool scale_depth = false;
      if (scale_depth) {
          if (depth < 0. || depth > 1.) {
            depth = 1.;
          }
          depth = (depth - velodyne_min_range_)/velodyne_max_range_;
      }
      depth_img_.at<float>(y,x) = depth;

      float intensity = pnts[i].intensity;
      bool scale_intensity = false;
      if (scale_intensity)
        intensity = intensity/255.;
      intensity_img_.at<float>(y,x) = intensity;
    }

    return 0;
  }

  bool ConvertToCompleteImages(tf::Transform &sensor_pose) {

    if (image_clean_) {
      getNextScanMsg();
      depth_img_ = cv::Scalar::all(0);
      intensity_img_ = cv::Scalar::all(0);

      tf::Transform T;
      ros::Time t0=global_scan->header.stamp + sensor_time_offset_;
      timestamp_of_last_sensor_message=t0;
      velodyne_rawdata::VPointCloud pnts,conv_points;

      if(odosync->getTransformationForTime(t0, tf_pose_id_, sensor_pose)){
      for (size_t next = 0; next < global_scan->packets.size(); ++next){
         dataParser.unpack(global_scan->packets[next], pnts); // unpack the raw data
         ros::Time t1=global_scan->packets[next].stamp + sensor_time_offset_;

         if(odosync->getTransformationForTime(t0,t1,tf_pose_id_,T)){
           pcl_ros::transformPointCloud(pnts,conv_points,T);
           this->addToImages(conv_points, width_, height_, 0, 0, T);
         }
         else{
              fprintf(stderr,"No transformation (velodyne images complete #1)\n");
         }
         pnts.clear();
       }
      }
      else {
        fprintf(stderr,"No transformation (velodyne images complete #00)\n");

      }

      image_clean_ = false;
      return true; // Return true if there is more to process...
    }
    else {
      getNextScanMsg();

      tf::Transform T;
      velodyne_rawdata::VPointCloud pnts,conv_points;

      for (size_t next = 0; next < global_scan->packets.size(); ++next){
         dataParser.unpack(global_scan->packets[next], pnts); // unpack the raw data
         ros::Time t1=global_scan->packets[next].stamp + sensor_time_offset_;

         if(odosync->getTransformationForTime(timestamp_of_last_sensor_message,t1,tf_pose_id_,T)){
           pcl_ros::transformPointCloud(pnts,conv_points,T);
           this->addToImages(conv_points, width_, height_, 0, 0, T);
         }
         else{
              fprintf(stderr,"No transformation (velodyne images complete #2)\n");
         }
         pnts.clear();
       }
      image_clean_ = true;
    }

    std::cout<<"Frame (images):"<<++counter<<std::endl;

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = timestamp_of_last_sensor_message;
    out_msg.header.frame_id = global_scan->header.frame_id;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

   {
      cv::Mat dst;
      cv::normalize(depth_img_, dst, 0, 1, cv::NORM_MINMAX);
      cv::imshow("depth_img", dst);
    }
    {
      cv::Mat dst;
      cv::normalize(intensity_img_, dst, 0, 1, cv::NORM_MINMAX);
      cv::imshow("intensity_img", dst);
    }
    cv::waitKey(1);
    out_msg.image = depth_img_;
    outbag.write("/velodyne_depth_img",timestamp_of_last_sensor_message,out_msg);

    out_msg.image = intensity_img_;
    outbag.write("/velodyne_intensity_img",timestamp_of_last_sensor_message,out_msg);
    return true;

  }

  // Take one set of packages and create one image (will not be complete)
  bool ConvertToImages(tf::Transform &sensor_pose){

    double resolution_factor = width_ / view_width_;
    depth_img_ = cv::Scalar::all(0);
    intensity_img_ = cv::Scalar::all(0);

    sensor_msgs::Image depth_img_msg, intensity_img_msg;

    //if(odosync == NULL) return true;
    rosbag::MessageInstance const m = *I;
    if(m.getTopic()==velodynetopic_){
      velodyne_msgs::VelodyneScan::ConstPtr scan = m.instantiate<velodyne_msgs::VelodyneScan>();
      if (scan != NULL){
        velodyne_rawdata::VPointCloud pnts,conv_points;
        // process each packet provided by the driver
        tf::Transform T;
        ros::Time t0=scan->header.stamp + sensor_time_offset_;
        timestamp_of_last_sensor_message=t0;
        if(odosync->getTransformationForTime(t0, tf_pose_id_, sensor_pose)){
          for (size_t next = 0; next < scan->packets.size(); ++next){
            dataParser.unpack(scan->packets[next], pnts); // unpack the raw data
            ros::Time t1=scan->packets[next].stamp + sensor_time_offset_;

            if(odosync->getTransformationForTime(t0,t1,tf_pose_id_,T)){
              pcl_ros::transformPointCloud(pnts,conv_points,T);
              this->addToImages(conv_points, width_, height_, 0, 0, T);

            }else{
              fprintf(stderr,"No transformation (velodyne images)\n");
            }
            pnts.clear();
          }


        }else{
          fprintf(stderr,"No transformation\n");
        }

        std::cout<<"Frame (images):"<<++counter<<std::endl;

        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = t0;
        out_msg.header.frame_id = scan->header.frame_id;
        out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;


        cv::imshow("depth_img", depth_img_);
        cv::imshow("intensity_img", intensity_img_);
        cv::waitKey(1);
        out_msg.image = depth_img_;
        outbag.write("/velodyne_depth_img",timestamp_of_last_sensor_message,out_msg);

        out_msg.image = intensity_img_;
        outbag.write("/velodyne_intensity_img",timestamp_of_last_sensor_message,out_msg);
      }
    }

    {
      outbag.write(m.getTopic(),m.getTime(),  m);
    }


    I++;

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
  std::string velodynetopic_;
  std::string tf_pose_id_;
  velodyne_msgs::VelodyneScan::ConstPtr global_scan;
  ros::Time timestamp_of_last_sensor_message;
  ros::Duration sensor_time_offset_;
  double view_width_;
  double velodyne_max_range_;
  double velodyne_min_range_;
  ros::Time image_start_time_;
  cv::Mat depth_img_;
  cv::Mat intensity_img_;
  bool image_clean_;
  int width_;
  int height_;
};

#endif // CONVERTVELODYNEBAGS_H
