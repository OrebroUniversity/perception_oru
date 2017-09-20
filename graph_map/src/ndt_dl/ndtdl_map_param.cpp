#include "ndt_dl/ndtdl_map_param.h"
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(libgraphMap::NDTDLMapParam)
using namespace std;
namespace libgraphMap{


void NDTDLMapParam::GetParametersFromRos(){
  MapParam::GetParametersFromRos();
  ros::NodeHandle nh("~");
  cout<<"reading parameters from ros inside NDTDLMapParam::GetRosParametersFromRos()"<<endl;
  nh.param<std::string>("Super_important_map_parameter",SuperImportantMapParameter,"parhaps not so important...");
  nh.param("resolution",resolution_,1.0);
}





}
