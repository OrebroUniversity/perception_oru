#include "ros/ros.h"
#include <ndt_map/ndt_conversions.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/lazy_grid.h>
#include <pointcloud_vrml/pointcloud_utils.h>

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>
#include <cstring>
#include <string>

int main(int argc, char** argv){
  ros::init(argc,argv,"map_topic");
  ros::NodeHandle nh;
  ros::Publisher map_pub = nh.advertise<ndt_map::NDTMap>("dummy_map_pub", 1000);
  ros::Rate loop_rate(10);
  ndt_map::NDTMap msg;
  lslgeneric::NDTMap<pcl::PointXYZ> nd(new lslgeneric::LazyGrid<pcl::PointXYZ>(0.2));
  if (nd.loadFromJFF("/home/maw/basement_04m.jff") < 0)
    ROS_INFO("loading from jff failed\n");
  lslgeneric::toMessage<pcl::PointXYZ>(&nd,msg,"base");
  while (ros::ok()){
    map_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
