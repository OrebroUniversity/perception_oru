//#include <ndt_fuser.h>
#include <ndt_fuser/ndt_fuser_hmt.h>
#include <ndt_map/ndt_conversions.h>
#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/ndt_conversions.h>
#include "ndt_fuser/ndt_fuser_ros_wrappers/ros_fuser_init.hpp"


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include <cstdio>
#include <Eigen/Eigen>
#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>
#include <cstdio>
#include <Eigen/Geometry>
#include <boost/foreach.hpp>

#include "eigen_conversions/eigen_msg.h"
class view_bag {

protected:
  // Our NodeHandle
  ros::NodeHandle nh_;

  ros::Subscriber *odom_sub_;
  ros::Subscriber *gt_sub;

  ros::Publisher *gt_pub,*odom_pub;
  nav_msgs::Odometry gt_pose_msg,odom_pose_msg;
  std::string odometry_topic="",gt_topic="";
  Eigen::Affine3d Todom,Todom_prev,Todom_init;
  Eigen::Affine3d Tgt,Tgt_prev,Tgt_init;
  bool init_gt_set=false;
  bool first_odom_set=false;
  int t_merge_frames=1;

public:
  // Constructor
  view_bag(ros::NodeHandle param_nh)
  {

    param_nh.param<std::string>("odometry_topic",odometry_topic,"/vmc_navserver/odom");
    ///if we want to compare to a ground truth topic;
    param_nh.param<std::string>("gt_topic",gt_topic,"/vmc_navserver/state");
    param_nh.param<int>("t_merge_frames",t_merge_frames,1);
    odom_sub_=new ros::Subscriber();
    gt_sub=new ros::Subscriber();
    gt_pub=new ros::Publisher();
    odom_pub=new ros::Publisher();
    *odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(odometry_topic,10,&view_bag::odom_callback, this);
    *gt_sub = nh_.subscribe<nav_msgs::Odometry>(gt_topic,10,&view_bag::gt_callback, this);
    *odom_pub=param_nh.advertise<nav_msgs::Odometry>("odom_aligned_with_gt",100);
    *gt_pub=param_nh.advertise<nav_msgs::Odometry>("gt_odom",100);
    gt_pose_msg.header.frame_id="/world";
    odom_pose_msg.header.frame_id="/world";
  }

  // Callback
  void gt_callback(const nav_msgs::Odometry::ConstPtr& msg_in)
  {
    static unsigned int counter=0;
    Eigen::Quaterniond qd;
    Eigen::Affine3d gt_pose;

    qd.x() = msg_in->pose.pose.orientation.x;
    qd.y() = msg_in->pose.pose.orientation.y;
    qd.z() = msg_in->pose.pose.orientation.z;
    qd.w() = msg_in->pose.pose.orientation.w;

    gt_pose = Eigen::Translation3d (msg_in->pose.pose.position.x,
                                    msg_in->pose.pose.position.y,msg_in->pose.pose.position.z) * qd;


    if(!init_gt_set && counter==t_merge_frames) {
      Tgt_init = gt_pose;
      ROS_INFO("Set initial pose from GT track");
      init_gt_set = true;
    }

    gt_pose_msg.header.stamp=msg_in->header.stamp;
    tf::poseEigenToMsg(gt_pose, gt_pose_msg.pose.pose);
    gt_pub->publish(gt_pose_msg);
    counter++;
  }
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg_in)
  {
    Eigen::Quaterniond qd;


    qd.x() = msg_in->pose.pose.orientation.x;
    qd.y() = msg_in->pose.pose.orientation.y;
    qd.z() = msg_in->pose.pose.orientation.z;
    qd.w() = msg_in->pose.pose.orientation.w;

    Todom = Eigen::Translation3d (msg_in->pose.pose.position.x,
                                  msg_in->pose.pose.position.y,msg_in->pose.pose.position.z) * qd;
    std::cout<<"odom:"<<Todom.translation().transpose()<<std::endl;
    if(!first_odom_set && init_gt_set) {
      Todom_init = Todom;
      ROS_INFO("Set initial odom pose");
      first_odom_set = true;
    }
    if(init_gt_set){
      std::cout<<"Todom_init: "<<Todom_init.translation()<<std::endl;
      std::cout<<"Tgt_init: "<<Tgt_init.translation()<<std::endl;
      Eigen::Affine3d odom_pose_in_gt_frame=Tgt_init*Todom_init.inverse()*Todom;
      tf::poseEigenToMsg(odom_pose_in_gt_frame, odom_pose_msg.pose.pose);
      odom_pose_msg.header.stamp=msg_in->header.stamp;
      odom_pub->publish(odom_pose_msg);
    }
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndt_fuser_node");
  ros::NodeHandle param("~");
  view_bag t(param);
  while(ros::ok()){
    ros::spinOnce();
  }

  return 0;
}

