#include "ndt/ndt_map_param.h"
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(libgraphMap::NDTMapParam)
using namespace std;
namespace libgraphMap{



void NDTMapParam::GetParametersFromRos(){
  MapParam::GetParametersFromRos();
  ros::NodeHandle nh("~");
  cout<<"reading parameters from ros inside GetRosParamNDT2D"<<endl;
  nh.param("resolution",resolution_,1.0);
  //nh.param("laser_variance_z",varz,resolution/4);
}




}
