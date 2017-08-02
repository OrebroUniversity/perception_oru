#include "graph_map/map_type.h"
namespace libgraphMap{


MapParam::~MapParam(){}

MapParam::MapParam(){}
void MapParam::GetParametersFromRos(){
  ros::NodeHandle nh("~");//base class parameters
  nh.param("sensor_range",max_range_,30.);
  nh.param("min_laser_range",min_range_,0.5);
  nh.param("size_x_meters",sizex_,20.);
  nh.param("size_y_meters",sizey_,20.);
  nh.param("size_z_meters",sizez_,20.);
  nh.param("enable_mapping",enable_mapping_,true);
  cout<<"read mapType parameters from ros"<<endl;
  cout<<ToString()<<endl;
}
string MapParam::ToString(){
  stringstream ss;
  ss<<"Base map parameters:"<<endl;
  ss<<"Range(max/min)=("<<max_range_<<"/"<<min_range_<<endl;
  ss<<"size(x,y,z)=("<<sizex_<<","<<sizey_<<","<<sizez_<<")";
  return ss.str();
}

MapType::MapType(){
  sizex_= 0;
  sizey_= 0;
  sizez_= 0;
  max_range_=0;
  min_range_=0;
  initialized_=false;
  enable_mapping_=true;
  mapName_="";
}

MapType::MapType(MapParamPtr param){
  initialized_=false;
  enable_mapping_=param->enable_mapping_;
  sizex_= param->sizex_;
  sizey_= param->sizey_;
  sizez_= param->sizez_;
  max_range_=param->max_range_;
  min_range_=param->min_range_;
  mapName_="";
}

bool MapType::CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius){
  cout<<"Compunding map not possible in base class"<<endl;
  return false;
}

}
