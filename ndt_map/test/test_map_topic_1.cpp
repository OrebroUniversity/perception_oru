#include "ros/ros.h"
#include <ndt_map/ndt_conversions.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/lazy_grid.h>
#include <ndt_map/pointcloud_utils.h>

#include <ndt_map/NDTCellMsg.h>
#include <ndt_map/NDTMapMsg.h>

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>
#include <cstring>
#include <string>


void mapCallback(const ndt_map::NDTMapMsg::ConstPtr& msg)
{
  lslgeneric::NDTMap *nd;
  lslgeneric::LazyGrid *idx;
  std::string f;
  lslgeneric::fromMessage(idx,nd,*msg,f);
  ros::shutdown();
  if (nd->writeToJFF("transported.jff") < 0)
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
