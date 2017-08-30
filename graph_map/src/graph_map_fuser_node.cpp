
#include "graph_map_fuser.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ndt_map/ndt_conversions.h>
#include "ndt_generic/utils.h"
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>


#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"

#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>

#include <boost/foreach.hpp>
#include <ndt_map/NDTMapMsg.h>
//#include "gnuplot-iostream.h"
#include "lidarUtils/lidar_utilities.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <time.h>
#include <fstream>
#include <cstdio>
#include "tf_conversions/tf_eigen.h"
#ifndef SYNC_FRAMES
#define SYNC_FRAMES 20
#define MAX_TRANSLATION_DELTA 2.0
#define MAX_ROTATION_DELTA 0.5
#endif
/** \brief A ROS node which implements an NDTFuser or NDTFuserHMT object
 * \author Daniel adolfsson based on code from Todor Stoyanov
 *
 */
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> LaserOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::PoseStamped> LaserPoseSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> PointsOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> PointsGTOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> PointsPoseSync;
using namespace libgraphMap;
class GraphMapFuserNode {

protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  GraphMapFuser *fuser_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *points2_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> *laser_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
  plotmarker plot_marker;

  message_filters::Subscriber<nav_msgs::Odometry> *gt_fuser_sub_;
  ros::Subscriber gt_sub,points2OdomTfSub;

  // Components for publishing
  tf::TransformBroadcaster tf_;
  tf::TransformListener tf_listener_;
  ros::Publisher output_pub_;
  Eigen::Affine3d pose_, T, sensorPose_;


  unsigned int frame_nr_;
  double varz;
  tf::Transform tf_sensor_pose_;
  std::string map_type_name,reg_type_name;
  std::string map_name="graph_map";
  std::string points_topic, laser_topic, map_dir, odometry_topic,odometry_adjusted_topic;
  std::string world_link_id, odometry_link_id, fuser_base_link_id,laser_link_id, init_pose_frame, gt_topic, bag_name,state_base_link_id;
  double size_x, size_y, size_z, resolution, sensor_range, min_laser_range_;
  bool visualize, match2D, matchLaser, beHMT, useOdometry,
  initPoseFromGT, initPoseFromTF, initPoseSet, gt_mapping;

  double pose_init_x,pose_init_y,pose_init_z,
  pose_init_r,pose_init_p,pose_init_t;
  double sensor_pose_x,sensor_pose_y,sensor_pose_z,
  sensor_pose_r,sensor_pose_p,sensor_pose_t;
  double sensor_offset_t_;

  laser_geometry::LaserProjection projector_;
  message_filters::Synchronizer< LaserOdomSync > *sync_lo_;
  message_filters::Synchronizer< LaserPoseSync > *sync_lp_;

  message_filters::Synchronizer< PointsGTOdomSync > *sync_GTodom_;
  message_filters::Synchronizer< PointsOdomSync > *sync_po_;
  message_filters::Synchronizer< PointsPoseSync > *sync_pp_;
  ros::ServiceServer save_map_;
  ros::Time time_now,time_last_itr;
  ros::Publisher map_publisher_,laser_publisher_,point2_publisher_,odom_publisher_,adjusted_odom_publisher_,fuser_odom_publisher_;
  nav_msgs::Odometry fuser_odom,adjusted_odom_msg;
  Eigen::Affine3d last_odom, this_odom,last_gt_pose;

  bool use_tf_listener_;
  Eigen::Affine3d last_tf_frame_;
  lslgeneric::MotionModel2d::Params motion_params;
  boost::mutex m;
public:
  // Constructor
  GraphMapFuserNode(ros::NodeHandle param_nh) : frame_nr_(0)
  {
    ///if we want to build map reading scans directly from bagfile


    ///topic to wait for point clouds, if available
    param_nh.param<std::string>("points_topic",points_topic,"points");
    ///topic to wait for laser scan messages, if available
    param_nh.param<std::string>("laser_topic",laser_topic,"laser_scan");

    ///only match 2ith 3dof
    param_nh.param("match2D",match2D,true);
    ///enable for LaserScan message input
    param_nh.param("matchLaser",matchLaser,true);


    param_nh.param<std::string>("registration_type",reg_type_name,"default_reg");
    ///range to cutoff sensor measurements
    ///
    param_nh.param("sensor_range",sensor_range,3.);

    ///visualize in a local window
    param_nh.param("visualize",visualize,true);

    std::string marker_str;
    param_nh.param<std::string>("plot_marker",marker_str,"sphere");
    if(marker_str.compare("sphere")==0)
      plot_marker=plotmarker::sphere;
    else if(marker_str.compare("point")==0)
      plot_marker=plotmarker::point;
    else
      plot_marker=plotmarker::sphere;



    ///range to cutoff sensor measurements
    param_nh.param("min_laser_range",min_laser_range_,0.1);


    ///if using the HMT fuser, NDT maps are saved in this directory.
    ///a word of warning: if you run multiple times with the same directory,
    ///the old maps are loaded automatically
    param_nh.param<std::string>("map_directory",map_dir,"/map/");
    param_nh.param<std::string>("map_type",map_type_name,"default_map");


    ///initial pose of the vehicle with respect to the map
    param_nh.param("pose_init_x",pose_init_x,0.);
    param_nh.param("pose_init_y",pose_init_y,0.);
    param_nh.param("pose_init_z",pose_init_z,0.);
    param_nh.param("pose_init_r",pose_init_r,0.);
    param_nh.param("pose_init_p",pose_init_p,0.);
    param_nh.param("pose_init_t",pose_init_t,0.);

    ///pose of the sensor with respect to the vehicle odometry frame
    param_nh.param("sensor_pose_x",sensor_pose_x,0.);
    param_nh.param("sensor_pose_y",sensor_pose_y,0.);
    param_nh.param("sensor_pose_z",sensor_pose_z,0.);
    param_nh.param("sensor_pose_r",sensor_pose_r,0.);
    param_nh.param("sensor_pose_p",sensor_pose_p,0.);
    param_nh.param("sensor_pose_t",sensor_pose_t,0.);
    param_nh.param("sensor_offset_t",sensor_offset_t_,0.);
    ///size of the map in x/y/z. if using HMT, this is the size of the central tile
    param_nh.param("size_x_meters",size_x,10.);
    param_nh.param("size_y_meters",size_y,10.);
    param_nh.param("size_z_meters",size_z,10.);

    param_nh.param<double>("motion_params_Cd", motion_params.Cd, 0.005);
    param_nh.param<double>("motion_params_Ct", motion_params.Ct, 0.01);
    param_nh.param<double>("motion_params_Dd", motion_params.Dd, 0.001);
    param_nh.param<double>("motion_params_Dt", motion_params.Dt, 0.01);
    param_nh.param<double>("motion_params_Td", motion_params.Td, 0.001);
    param_nh.param<double>("motion_params_Tt", motion_params.Tt, 0.005);

    bool do_soft_constraints;
    param_nh.param<bool>("do_soft_constraints", do_soft_constraints, false);
    param_nh.param("laser_variance_z",varz,resolution/4);

    param_nh.param<std::string>("bagfile_name",bag_name,"data.bag");
    cout<<"bagfile_name"<<points_topic<<endl;


    ///if we want to create map based on GT pose
    param_nh.param("renderGTmap",gt_mapping,false);
    param_nh.param<std::string>("gt_topic",gt_topic,"groundtruth");
    ///if we want to get the initial pose of the vehicle relative to a different frame
    param_nh.param("initPoseFromGT",initPoseFromGT,false);
    //plot the map from the GT track if available


    ///topic to wait for laser scan messages, if available
    param_nh.param<std::string>("odometry_topic",odometry_topic,"odometry");

    param_nh.param<std::string>("odometry_adjusted",odometry_adjusted_topic,"odometry_adjusted");
    //get it from TF?
    param_nh.param("initPoseFromTF",initPoseFromTF,false);

    //the frame to initialize to

    param_nh.param<std::string>("world_frame",world_link_id,"/world");
    //our frame
    param_nh.param<std::string>("fuser_frame_id",fuser_base_link_id,"/fuser_base_link");
    param_nh.param<std::string>("laser_frame_id",laser_link_id,"/velodyne");

    param_nh.param<std::string>("state_base_link_id",state_base_link_id,"/state_base_link");

    ///use standard odometry messages for initialuess
    param_nh.param("useOdometry",useOdometry,true);

    param_nh.param<bool>("use_tf_listener", use_tf_listener_, false);
    param_nh.param<std::string>("odometry_frame_id", odometry_link_id, std::string("/odom_base_link"));

    initPoseSet = false;
    param_nh.param<bool>("do_soft_constraints", do_soft_constraints, false);
    fuser_odom.header.frame_id="/world";
    adjusted_odom_msg.header.frame_id="/world";
    laser_publisher_ =param_nh.advertise<sensor_msgs::LaserScan>("laserscan_in_fuser_frame",50);

    point2_publisher_ =param_nh.advertise<sensor_msgs::PointCloud2>("point2_fuser",15);
    fuser_odom_publisher_=param_nh.advertise<nav_msgs::Odometry>("fuser",50);
    adjusted_odom_publisher_=param_nh.advertise<nav_msgs::Odometry>("odom_gt_init",50);

    if(gt_mapping)
      use_tf_listener_= use_tf_listener_ && state_base_link_id != std::string("");// check if odometry topic exists
    else
      use_tf_listener_= use_tf_listener_ && odometry_link_id != std::string("");// check if odometry topic exists


    sensorPose_ =  Eigen::Translation<double,3>(sensor_pose_x,sensor_pose_y,sensor_pose_z)*
        Eigen::AngleAxis<double>(sensor_pose_r,Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxis<double>(sensor_pose_p,Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxis<double>(sensor_pose_t,Eigen::Vector3d::UnitZ()) ;

    tf::poseEigenToTF(sensorPose_,tf_sensor_pose_);

    if(!initPoseFromGT){
      pose_ =  Eigen::Translation<double,3>(pose_init_x,pose_init_y,pose_init_z)*
          Eigen::AngleAxis<double>(pose_init_r,Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxis<double>(pose_init_p,Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxis<double>(pose_init_t,Eigen::Vector3d::UnitZ()) ;
      initPoseSet=true;

      fuser_=new GraphMapFuser(map_type_name,reg_type_name,pose_,sensorPose_);
      cout<<"set fuser viz="<<visualize<<endl;
      fuser_->Visualize(visualize);

    }


    cout<<"node: initial pose =\n"<<pose_.translation()<<endl;


    if(!matchLaser) {
      points2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,points_topic,2);
      if(useOdometry) {
        if(gt_mapping){
          if(!use_tf_listener_){
            gt_fuser_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,gt_topic,10);
            sync_GTodom_ = new message_filters::Synchronizer< PointsGTOdomSync >(PointsGTOdomSync(SYNC_FRAMES), *points2_sub_, *gt_fuser_sub_);
            sync_GTodom_->registerCallback(boost::bind(&GraphMapFuserNode::GTLaserPointsOdomCallback, this, _1, _2));
          }
          else
            points2OdomTfSub=nh_.subscribe <sensor_msgs::PointCloud2>(points_topic,10,&GraphMapFuserNode::GTLaserPointsOdomCallbackTF,this);

        }
        else{
          if(!use_tf_listener_){
            odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
            sync_po_ = new message_filters::Synchronizer< PointsOdomSync >(PointsOdomSync(SYNC_FRAMES), *points2_sub_, *odom_sub_);
            sync_po_->registerCallback(boost::bind(&GraphMapFuserNode::points2OdomCallback, this,_1, _2));
          }
          else
            points2OdomTfSub=nh_.subscribe <sensor_msgs::PointCloud2>(points_topic,10,&GraphMapFuserNode::points2OdomCallbackTF,this);
        }
      }
    }
    else
    {
      laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_,laser_topic,2);
      if(useOdometry) {
        odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
        sync_lo_ = new message_filters::Synchronizer< LaserOdomSync >(LaserOdomSync(SYNC_FRAMES), *laser_sub_, *odom_sub_);
        sync_lo_->registerCallback(boost::bind(&GraphMapFuserNode::laserOdomCallback, this, _1, _2));
      }
      else{
        ((void)0); //Do nothing, seriously consider using a laser only callback (no odometry sync)
      }
    }
    if(initPoseFromGT) {
      gt_sub = nh_.subscribe<nav_msgs::Odometry>(gt_topic,10,&GraphMapFuserNode::gt_callback, this);
    }
    save_map_ = param_nh.advertiseService("save_map", &GraphMapFuserNode::save_map_callback, this);
    cout<<"init done"<<endl;
  }

  void processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d Tmotion) {

    if(!initPoseSet)
      return;

    frame_nr_++;
    cout<<"frame nr="<<frame_nr_<<endl;
    if((Tmotion.translation().norm() <0.005 && Tmotion.rotation().eulerAngles(0,1,2).norm()< 0.005) && useOdometry) {    //sanity check for odometry
      return;
    }
    (void)ResetInvalidMotion(Tmotion);

    cout<<"frame="<<frame_nr_<<"movement="<<(fuser_->GetPoseLastFuse().inverse()*pose_).translation().norm()<<endl;

    ros::Time tplot=ros::Time::now();
    plotPointcloud2(cloud,tplot);
    m.lock();
    fuser_->ProcessFrame<pcl::PointXYZ>(cloud,pose_,Tmotion);
    m.unlock();
    fuser_->plotMap();
    tf::Transform Transform;
    tf::transformEigenToTF(pose_,Transform);
    tf_.sendTransform(tf::StampedTransform(Transform, tplot, world_link_id, fuser_base_link_id));
    tf_.sendTransform(tf::StampedTransform(tf_sensor_pose_, tplot, fuser_base_link_id, laser_link_id));
    fuser_odom.header.stamp=tplot;

    tf::poseEigenToMsg( pose_,fuser_odom.pose.pose);
    fuser_odom_publisher_.publish(fuser_odom);
  }


  //bool save_map_callback(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res )
  bool ResetInvalidMotion(Eigen::Affine3d &Tmotion){
    if(Tmotion.translation().norm() > MAX_TRANSLATION_DELTA) {
      std::cerr<<"Ignoring Odometry (max transl)!\n";
      std::cerr<<Tmotion.translation().transpose()<<std::endl;
      Tmotion.setIdentity();
      return true;
    }
    else if(Tmotion.rotation().eulerAngles(0,1,2)(2) > MAX_ROTATION_DELTA) {
      std::cerr<<"Ignoring Odometry (max rot)!\n";
      std::cerr<<Tmotion.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
      Tmotion.setIdentity();
      return true;
    }
    else return false;
  }
  bool save_map_callback(std_srvs::Empty::Request  &req,
                         std_srvs::Empty::Response &res ) {
    char path[1000];
    string time=ndt_generic::currentDateTimeString();

    if(fuser_!=NULL){
      snprintf(path,999,"%s/%s_.MAP",map_dir.c_str(),time.c_str());
      m.lock();
      fuser_->SaveGraphMap(path);
      m.unlock();
      ROS_INFO("Current map was saved to path= %s", path);
      return true;
    }
    else
      ROS_INFO("No data to save");
    return false;
  }

  inline bool getAffine3dTransformFromTF(const ros::Time &time,const std::string &link_id,Eigen::Affine3d& ret,const ros::Duration &wait) {
    static tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    tf_listener_.waitForTransform(world_link_id, link_id,  time ,wait);
    try{
      tf_listener_.lookupTransform(world_link_id, link_id, time, transform);
      tf::poseTFToEigen(transform, ret);
      cout<<"found "<<ret.translation().transpose()<<endl;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return false;
    }
    return true;
  }

  // Callback
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
  {
    cout<<"laser callback"<<endl;
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_unfiltered, pcl_cloud;
    projector_.projectLaser(*msg_in, cloud);
    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

    pcl::PointXYZ pt;
    //add some variance on z
    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
      pt = pcl_cloud_unfiltered.points[i];
      if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_laser_range_) {
        pt.z += varz*((double)rand())/(double)INT_MAX;
        pcl_cloud.points.push_back(pt);
      }
    }
    T.setIdentity();
    cout<<"node: laser call back, process frame"<<endl;
    this->processFrame(pcl_cloud,T);


  }

  // Callback
  void laserOdomCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in,
                         const nav_msgs::Odometry::ConstPtr& odo_in)
  {
    cout<<"laser odom callback"<<endl;
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud, pcl_cloud_unfiltered;
    Eigen::Affine3d Tm;

    tf::poseMsgToEigen(odo_in->pose.pose,this_odom);
    if (frame_nr_  <= 1){
      Tm.setIdentity();
    }
    else
      Tm = last_odom.inverse()*this_odom;

    last_odom = this_odom;
    projector_.projectLaser(*msg_in, cloud);
    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);
    sensor_msgs::LaserScan msg_out=*msg_in;
    msg_out.header.stamp=ros::Time::now();
    msg_out.header.frame_id="/fuser_laser_link";
    laser_publisher_.publish(msg_out);
    pcl::PointXYZ pt;

    //add some variance on z
    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
      pt = pcl_cloud_unfiltered.points[i];
      if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_laser_range_) {
        pt.z += varz*((double)rand())/(double)INT_MAX;
        pcl_cloud.points.push_back(pt);
      }
    }
    this->processFrame(pcl_cloud,Tm);
    cout<< "publish fuser data"<<endl;
  }

  void plotPointcloud2(pcl::PointCloud<pcl::PointXYZ> & cloud,ros::Time time = ros::Time::now()){
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(cloud,msg_out);
    msg_out.header.frame_id=laser_link_id;
    msg_out.header.stamp=time;
    point2_publisher_.publish(msg_out);
  }
  void points2OdomCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in,
                           const nav_msgs::Odometry::ConstPtr& odo_in)//callback is used in conjunction with odometry time filter.
  {
    ros::Time tstart=ros::Time::now();

    tf::poseMsgToEigen(odo_in->pose.pose,this_odom);

    Eigen::Affine3d Tm;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    if (frame_nr_ == 0)
      Tm.setIdentity();
    else {
      Tm = last_odom.inverse()*this_odom;
    }
    last_odom = this_odom;
    pcl::fromROSMsg (*msg_in, cloud);
    this->processFrame(cloud,Tm);
    ros::Time tend=ros::Time::now();
    cout<<"Total execution time= "<<tend-tstart<<endl;
  }
  void points2OdomCallbackTF(const sensor_msgs::PointCloud2::ConstPtr& msg_in){//this callback is used to look up tf transformation for scan data
    Eigen::Affine3d Tm;
    static bool last_odom_found=false;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*msg_in, cloud);
    bool found_odom= getAffine3dTransformFromTF((msg_in->header.stamp-ros::Duration(sensor_offset_t_)),odometry_link_id,this_odom,ros::Duration(0.1));
    if (frame_nr_ =0 || !found_odom||!last_odom_found)
      Tm.setIdentity();
    else {
      Tm = last_odom.inverse()*this_odom;
    }
    last_odom_found=found_odom;

    last_odom = this_odom;
    this->processFrame(cloud,Tm);
    cout<<"TF callback Point2Odom"<<endl;
  }

  void GTLaserPointsOdomCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in,
                                 const nav_msgs::Odometry::ConstPtr& odo_in)//this callback is used for GT based mapping
  {
    cout<<"GT diff:"<<(msg_in->header.stamp-odo_in->header.stamp).toSec()<<endl;
    Eigen::Affine3d Tmotion;
    if(frame_nr_==0){
      Tmotion=Eigen::Affine3d::Identity();
    }
    Eigen::Affine3d GT_pose;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    tf::poseMsgToEigen(odo_in->pose.pose,pose_);
    pcl::fromROSMsg (*msg_in, cloud);
    ros::Time t_stamp=ros::Time::now();//msg_in->header.stamp;
    tf::Transform gt_base;
    tf::poseMsgToTF(odo_in->pose.pose,gt_base);
    tf_.sendTransform(tf::StampedTransform(gt_base, t_stamp, world_link_id, std::string("online_")+state_base_link_id));
    tf_.sendTransform(tf::StampedTransform(tf_sensor_pose_, t_stamp, std::string("online_")+state_base_link_id, laser_link_id));
    plotPointcloud2(cloud);
    m.lock();
    fuser_->ProcessFrame(cloud,pose_,Tmotion);
    fuser_->plotMap();
    m.unlock();
  }
  void GTLaserPointsOdomCallbackTF(const sensor_msgs::PointCloud2::ConstPtr& msg_in)//this callback is used for GT based mapping with TF lookup
  {
    cout<<"GT-TF toffset:"<<sensor_offset_t_<<endl;
    Eigen::Affine3d tmp_pose;
    Eigen::Affine3d Tmotion=Eigen::Affine3d::Identity();
    bool found_odom= getAffine3dTransformFromTF((msg_in->header.stamp-ros::Duration(sensor_offset_t_)),state_base_link_id,tmp_pose,ros::Duration(0.1));

    if(found_odom){
      pose_=tmp_pose;
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg (*msg_in, cloud);
      ros::Time t_stamp=ros::Time::now();//msg_in->header.stamp;
      tf::Transform tf_gt_base;
      tf::poseEigenToTF(pose_,tf_gt_base);
      tf_.sendTransform(tf::StampedTransform(tf_gt_base, t_stamp, world_link_id, std::string("online_")+state_base_link_id));
      tf_.sendTransform(tf::StampedTransform(tf_sensor_pose_, t_stamp, std::string("online_")+state_base_link_id, laser_link_id));
      plotPointcloud2(cloud,t_stamp);
      fuser_->ProcessFrame(cloud,pose_,Tmotion);
      fuser_->plotMap();
      m.unlock();
    }

  }
  // Callback
  void gt_callback(const nav_msgs::Odometry::ConstPtr& msg_in)//This callback is used to set initial pose from GT data.
  {

    Eigen::Affine3d gt_pose;
    tf::poseMsgToEigen(msg_in->pose.pose,gt_pose);

    if(initPoseFromGT && !initPoseSet) {
      pose_ = gt_pose;
      ROS_INFO("Set initial pose from GT track");
      fuser_=new GraphMapFuser(map_type_name,reg_type_name,pose_,sensorPose_);
      cout<<"----------------------------FUSER------------------------"<<endl;
      cout<<fuser_->ToString()<<endl;
      fuser_->Visualize(visualize,plot_marker);
      cout<<"---------------------------------------------------------"<<endl;
      initPoseSet = true;

    }
  }

public:
  // map publishing function
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graph_map_fuser_node");
  ros::NodeHandle param("~");
  GraphMapFuserNode t(param);
  ros::spin();

  return 0;
}

