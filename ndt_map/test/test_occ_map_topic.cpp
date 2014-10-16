#include "ros/ros.h"
#include <ndt_map/ndt_conversions.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/lazy_grid.h>
#include <ndt_map/pointcloud_utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ndt_map/NDTMapMsg.h>

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>
#include <cstring>
#include <string>

int main(int argc, char** argv){
  ros::init(argc,argv,"occ_map_topic");
  ros::NodeHandle nh;
  ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("dummy_occ_map_pub", 1000);
  ros::Rate loop_rate(1);
  nav_msgs::OccupancyGrid msg;

  lslgeneric::NDTMap nd(new lslgeneric::LazyGrid(0.4));
  ROS_INFO("loading from jff...\n");
  if (nd.loadFromJFF("basement2d_map.jff") < 0)
    ROS_INFO("loading from jff failed\n");

  lslgeneric::toOccupancyGrid(&nd,msg,0.05,"/base");
  while (ros::ok()){
    map_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
