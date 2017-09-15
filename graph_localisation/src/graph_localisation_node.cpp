
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
class GraphLocalisatonNode {

protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  ros::Subscriber gt_sub;
  ros::Publisher fuser_odom_publisher_;
  nav_msgs::Odometry estimate_msg,gt_msg;
  string gt_topic="/vmc_navserver/state";

public:
  // Constructor
  GraphLocalisatonNode(ros::NodeHandle param_nh)
  {
      fuser_odom_publisher_=param_nh.advertise<nav_msgs::Odometry>("/estimate",50);
      gt_sub = nh_.subscribe<nav_msgs::Odometry>(gt_topic,10,&GraphLocalisatonNode::gt_callback, this);
  }

  void gt_callback(const nav_msgs::Odometry::ConstPtr& msg_in)//This callback is used to set initial pose from GT data.
  {
    fuser_odom_publisher_.publish(msg_in);
  }

public:
  // map publishing function
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graph_localisation_node");
  ros::NodeHandle param("~");
  GraphLocalisatonNode t(param);
  ros::spin();

  return 0;
}

