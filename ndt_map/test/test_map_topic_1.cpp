#include "ros/ros.h"
#include <ndt_map/ndt_conversions.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/NDTCell.h>
#include <ndt_map/NDTMap.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/lazy_grid.h>
#include <pointcloud_vrml/pointcloud_utils.h>

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>
#include <cstring>
#include <string>



void mapCallback(const ndt_map::NDTMap::ConstPtr& msg)
{
  lslgeneric::NDTMap<pcl::PointXYZ> *nd;
  lslgeneric::LazyGrid<pcl::PointXYZ> *idx;
  std::string f;
  lslgeneric::fromMessage<pcl::PointXYZ>(idx,nd,*msg,f);
  ROS_INFO("%d",nd->getMyIndexInt());
  ros::shutdown();
  if (nd->writeToJFF("/home/maw/transported.jff") < 0)
    ROS_INFO("writing to jff failed\n");
  else
    ROS_INFO("SUCCESS!!!\n");

}

int main(int argc, char** argv){
  ros::init(argc,argv,"map_topic_1");
  ros::NodeHandle nh;
  ros::Subscriber map_sub = nh.subscribe("dummy_map_pub", 1000, mapCallback);
  //  while(ros::ok()){

  ros::spin();
  // }
  return 0;
}
