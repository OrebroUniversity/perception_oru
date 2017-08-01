
#include "graph_map_fuser.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ndt_map/ndt_conversions.h>
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
#include "gnuplot-iostream.h"
#include "lidarUtils/lidar_utilities.h"
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
  message_filters::Subscriber<nav_msgs::Odometry> *gt_fuser_sub_;
  ros::Subscriber gt_sub;

  // Components for publishing
  tf::TransformBroadcaster tf_;
  tf::TransformListener tf_listener_;
  ros::Publisher output_pub_;
  Eigen::Affine3d pose_, T, sensorPose_;
  unsigned int frame_nr_;
  double varz;

  boost::mutex m, message_m;
  std::string points_topic, laser_topic, map_dir, map_type_name,reg_type_name, odometry_topic,
  world_frame, fuser_frame, init_pose_frame, gt_topic, bag_name;
  double size_x, size_y, size_z, resolution, sensor_range, min_laser_range_;
  bool visualize, match2D, matchLaser, beHMT, useOdometry,
  initPoseFromGT, initPoseFromTF, initPoseSet, renderGTmap;

  double pose_init_x,pose_init_y,pose_init_z,
  pose_init_r,pose_init_p,pose_init_t;
  double sensor_pose_x,sensor_pose_y,sensor_pose_z,
  sensor_pose_r,sensor_pose_p,sensor_pose_t;
  laser_geometry::LaserProjection projector_;
  ScanPlot plotter_;
  message_filters::Synchronizer< LaserOdomSync > *sync_lo_;
  message_filters::Synchronizer< LaserPoseSync > *sync_lp_;


  message_filters::Synchronizer< PointsGTOdomSync > *sync_GTodom_;
  message_filters::Synchronizer< PointsOdomSync > *sync_po_;
  message_filters::Synchronizer< PointsPoseSync > *sync_pp_;
  ros::ServiceServer save_map_;
  ros::Time time_now,time_last_itr;
  ros::Publisher map_publisher_,laser_publisher_,fuser_odom_publisher_;
  nav_msgs::Odometry fuser_odom;
  Eigen::Affine3d last_odom, this_odom,last_gt_pose;
  std::string tf_pose_frame_;
  bool use_tf_listener_;
  Eigen::Affine3d last_tf_frame_;
  lslgeneric::MotionModel2d::Params motion_params;
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
    ///range to cutoff sensor measurements
    param_nh.param("min_laser_range",min_laser_range_,0.1);

    ///use standard odometry messages for initial guess
    param_nh.param("useOdometry",useOdometry,true);
    ///topic to wait for laser scan messages, if available
    param_nh.param<std::string>("odometry_topic",odometry_topic,"odometry");

    ///if using the HMT fuser, NDT maps are saved in this directory.
    ///a word of warning: if you run multiple times with the same directory,
    ///the old maps are loaded automatically
    param_nh.param<std::string>("map_directory",map_dir,"map");
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

    ///visualize in a local window
    param_nh.param("visualize",visualize,true);
    param_nh.param<std::string>("bagfile_name",bag_name,"data.bag");
    cout<<"bagfile_name"<<points_topic<<endl;



    ///if we want to create map based on GT pose
    param_nh.param("renderGTmap",renderGTmap,false);
    param_nh.param<std::string>("gt_topic",gt_topic,"groundtruth");
    ///if we want to get the initial pose of the vehicle relative to a different frame
    param_nh.param("initPoseFromGT",initPoseFromGT,false);
    //plot the map from the GT track if available


    //get it from TF?
    param_nh.param("initPoseFromTF",initPoseFromTF,false);
    //the frame to initialize to
    param_nh.param<std::string>("init_pose_frame",init_pose_frame,"/state_base_link");
    //the world frame
    param_nh.param<std::string>("world_frame",world_frame,"/world");
    //our frame
    param_nh.param<std::string>("fuser_frame",fuser_frame,"/fuser");


    param_nh.param<std::string>("tf_pose_frame", tf_pose_frame_, std::string(""));

    initPoseSet = false;
    param_nh.param<bool>("do_soft_constraints", do_soft_constraints, false);
    fuser_odom.header.frame_id="/world";
    laser_publisher_=param_nh.advertise<sensor_msgs::LaserScan>("laserscan_in_fuser_frame",50);
    fuser_odom_publisher_=param_nh.advertise<nav_msgs::Odometry>("fuser_odom",50);
    use_tf_listener_ = false;
    if (tf_pose_frame_ != std::string("")) {
      use_tf_listener_ = true;
    }


    sensorPose_ =  Eigen::Translation<double,3>(sensor_pose_x,sensor_pose_y,sensor_pose_z)*
        Eigen::AngleAxis<double>(sensor_pose_r,Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxis<double>(sensor_pose_p,Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxis<double>(sensor_pose_t,Eigen::Vector3d::UnitZ()) ;

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
        if(renderGTmap){
          gt_fuser_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,gt_topic,10);
          sync_GTodom_ = new message_filters::Synchronizer< PointsGTOdomSync >(PointsGTOdomSync(SYNC_FRAMES), *points2_sub_, *gt_fuser_sub_);
          sync_GTodom_->registerCallback(boost::bind(&GraphMapFuserNode::GTLaserPointsOdomCallback, this, _1, _2));
        }
        else{
          odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
          sync_po_ = new message_filters::Synchronizer< PointsOdomSync >(PointsOdomSync(SYNC_FRAMES), *points2_sub_, *odom_sub_);
          sync_po_->registerCallback(boost::bind(&GraphMapFuserNode::points2OdomCallback, this,_1, _2));
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
    cout<<"init done"<<endl;
  }

  void processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud,
                    Eigen::Affine3d Tmotion) {
    if(!initPoseSet)
      return;

    frame_nr_++;
    if(frame_nr_==0)
      time_last_itr=ros::Time::now();
    else{
      time_now=ros::Time::now();
      //  cout<<"iteration time= "<<time_now-time_last_itr<<endl;
      time_last_itr=time_now;


      cout<<"frame nr="<<frame_nr_<<endl;
      if((Tmotion.translation().norm() <0.005 && Tmotion.rotation().eulerAngles(0,1,2)(2)< 0.005) && useOdometry) {    //sanity check for odometry
        //cout<<"norm:" <<Tmotion.translation().norm()<<endl;
        std::cerr<<"No motion, skipping Frame\n";
        return;
      }
      (void)ResetInvalidMotion(Tmotion);
      pose_=pose_*Tmotion; //currently being changed to increment pose inside fuser
      cout<<"frame="<<frame_nr_<<"movement="<<(fuser_->GetPoseLastFuse().inverse()*pose_).translation().norm()<<endl;
    //  if((fuser_->GetPoseLastFuse().inverse()*pose_).translation().norm()>=0.0 || fuser_->FramesProcessed()==0){

        cout<<"perform update"<<endl;
        fuser_->ProcessFrame(cloud,pose_,Tmotion);



      tf::Transform Transform;
      tf::transformEigenToTF(pose_,Transform);
      tf_.sendTransform(tf::StampedTransform(Transform, ros::Time::now(), world_frame, fuser_frame));
      fuser_odom.header.stamp=ros::Time::now();

      tf::poseEigenToMsg( pose_,fuser_odom.pose.pose);
      fuser_odom_publisher_.publish(fuser_odom);
    }
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
  inline bool getAffine3dTransformFromTF(const ros::Time &time, Eigen::Affine3d& ret) {
    tf::StampedTransform transform;
    tf_listener_.waitForTransform("/world", tf_pose_frame_, time,ros::Duration(1.0));
    try{
      tf_listener_.lookupTransform("/world", tf_pose_frame_, time, transform);
      tf::poseTFToEigen(transform, ret);
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
    if (frame_nr_  == 0){
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
    if(frame_nr_%3==0){
      //plotter_.plot(msg_out,ScanPlot::RangeAngle);
    }

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
  void points2OdomCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in,
                           const nav_msgs::Odometry::ConstPtr& odo_in)
  {
    ros::Time tstart=ros::Time::now();
    // cout<<"Point odom callback"<<endl;
    Eigen::Affine3d Tm;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    tf::poseMsgToEigen(odo_in->pose.pose,this_odom);

    if (frame_nr_ == 0)
      Tm.setIdentity();
    else {
      Tm = last_odom.inverse()*this_odom;
    }
    last_odom = this_odom;
    cout<<"odometry=\n"<<Tm.translation().norm()<<endl;
    pcl::fromROSMsg (*msg_in, cloud);
    this->processFrame(cloud,Tm);
    ros::Time tend=ros::Time::now();
    cout<<"Total execution time= "<<tend-tstart<<endl;
  }
  void GTLaserPointsOdomCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in,
                                 const nav_msgs::Odometry::ConstPtr& odo_in)
  {
    cout<<"GT 3d laser mapping"<<endl;
    Eigen::Affine3d Tmotion;
    if(frame_nr_==0){
      Tmotion=Eigen::Affine3d::Identity();
    }
    //cout<<"GT point callback"<<endl;
    Eigen::Affine3d GT_pose;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    tf::poseMsgToEigen(odo_in->pose.pose,pose_);
    pcl::fromROSMsg (*msg_in, cloud);
    fuser_->ProcessFrame(cloud,pose_,Tmotion);
  }
  // Callback
  void gt_callback(const nav_msgs::Odometry::ConstPtr& msg_in)
  {
      cout<<"GT odom callback"<<endl;
    Eigen::Affine3d gt_pose;
    tf::poseMsgToEigen(msg_in->pose.pose,gt_pose);

    if(initPoseFromGT && !initPoseSet) {
      pose_ = gt_pose;
      ROS_INFO("Set initial pose from GT track");
      fuser_=new GraphMapFuser(map_type_name,reg_type_name,pose_,sensorPose_);
      fuser_->Visualize(visualize);
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

