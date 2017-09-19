#include "ndt_map/ndt_map_hmt.h"
#include "ros/ros.h"
#include "ndt_map/ndt_conversions.h"
int main(int argc, char **argv){
  ros::init(argc, argv, "ndt_map_publisher");
  ros::NodeHandle nh, param("~");
  std::string mapFile;
  double resolution;
  double mapRate;
  ros::Publisher mapPub;

  param.param<std::string>("map_file",mapFile,"file.jff");
  param.param<double>("resolution",resolution,0.2);
  param.param<double>("map_rate",mapRate,1);

  // lslgeneric::LazyGrid *mapGrid = new lslgeneric::LazyGrid(resolution);
  lslgeneric::NDTMap *ndtMap = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution));
  if (ndtMap->loadFromJFF(mapFile.c_str())<0){
    return -1;
  }

  ndt_map::NDTMapMsg mapMsg;
  lslgeneric::toMessage(ndtMap,mapMsg,"/map");//add tf betwen map and world!!!!
  ros::Rate loop_rate(mapRate);
  mapPub = nh.advertise<ndt_map::NDTMapMsg>("ndt_map", 1);
  while (ros::ok()){
    mapPub.publish(mapMsg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
