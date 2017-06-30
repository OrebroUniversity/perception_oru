#include "ndt/ndt_map_param.h"
using namespace std;
namespace libgraphMap{

NDTMapParam::NDTMapParam(){}

void NDTMapParam::GetParametersFromRos(){
  MapParam::GetParametersFromRos();
  ros::NodeHandle nh("~");
  cout<<"reading parameters from ros inside GetRosParamNDT2D"<<endl;
  nh.param("resolution",resolution_,1.0);
  //nh.param("laser_variance_z",varz,resolution/4);
}

template<class Archive>
void NDTMapParam::serialize(Archive & ar, const unsigned int version){
  ar & boost::serialization::base_object<MapParam>(*this);
  //ar & cenx_& ceny_ &cenz_;
}



}
