#ifndef NDT2DMAPPARAM_H
#define NDT2DMAPPARAM_H
#include "graph_map/map_type.h"
#include <string.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include "ros/ros.h"
using namespace std;
namespace libgraphMap{

/*!
 * ... Parameter class for mapType. Class is by choice of design fully public.  ...
 */

class NDTMapParam : public MapParam{
public:
  ~NDTMapParam(){}
  NDTMapParam(){}
  void GetParametersFromRos();

  double resolution_=0.4;
  std::string directory_="";
  bool saveOnDelete_=false,match2D_=false,beHMT=false,matchLaser=false;
  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    ar & boost::serialization::base_object<MapParam>(*this);
    ar & resolution_;
    ar & saveOnDelete_ & match2D_ & beHMT & matchLaser;
  }


};


}
#endif // NDT2DMAPPARAM_H
