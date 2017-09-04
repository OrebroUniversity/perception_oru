#ifndef NDTDL_MAP_PARAM_H
#define NDTDL_MAP_PARAM_H

#include "graph_map/map_type.h"
#include <string.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include "ros/ros.h"

namespace libgraphMap{

class NDTDLMapParam : public MapParam{
public:
  ~NDTDLMapParam(){}
  NDTDLMapParam(){};
  void GetParametersFromRos();

  double resolution_=0.4;
  string SuperImportantMapParameter;
  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    ar & boost::serialization::base_object<MapParam>(*this);
  }
};


}
#endif
