#include "ndt_dl/ndtdl_map_type.h"
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(libgraphMap::NDTDL)
namespace libgraphMap{
using namespace std;


NDTDL::NDTDL( MapParamPtr paramptr) : MapType(paramptr){
  NDTDLMapParamPtr param = boost::dynamic_pointer_cast< NDTDLMapParam >(paramptr);//Should not be NULL
  if(param!=NULL){
    //Get parameters to this class from param
    cout<<"ndtdl: created ndtdl map"<<endl;
  }
  else
    cerr<<"templateMapType: Cannot create instance for \"templateMapType\""<<std::endl;
}
NDTDL::~NDTDL(){}

void NDTDL::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.

  cout<<"please implement map update for NDT-DL"<<endl;
  if(initialized_){
    //Initialize map
  }else{
    //Update map
    initialized_ = true;
  }
}

void NDTDL::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.

  cout<<"please implement map update for NDT-DL for point type PointXYZIR"<<endl;
  if(initialized_){
    //Initialize map
  }else{
    //Update map
    initialized_ = true;
  }
}

bool NDTDL::CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius){

  return true;
  cout<<"please implement map compound for improved usage of submaps"<<endl;
  if( NDTDLMapTypePtr targetPtr=boost::dynamic_pointer_cast<NDTDL>(target) ){

    cout<<"\"CompoundMapsByRadius\" not overrided by template but not implemented"<<endl;
  }
}



NDTDLMapParam::NDTDLMapParam(){}
void NDTDLMapParam::GetParametersFromRos(){
  MapParam::GetParametersFromRos();
  ros::NodeHandle nh("~");
  cout<<"reading parameters from ros inside GetRosParamNDT2D"<<endl;
  nh.param<std::string>("Super_important_map_parameter",SuperImportantMapParameter,"parhaps not so important...");
}



}

