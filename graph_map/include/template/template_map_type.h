#ifndef TEMPLATE_MAP_TYPE_H
#define TEMPLATE_MAP_TYPE_H

#include "graphfactory.h" //includes the full list of forward declarations
#include <graph_map/map_type.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include "ros/ros.h"
#include "ros/node_handle.h"


#define template_map_name "template"
namespace libgraphMap{

class TemplateMapType:public MapType{
public:
  //Mandatory
  ~TemplateMapType();
  TemplateMapType(MapParamPtr paramptr);
  virtual void update(const Eigen::Affine3d &Tsensor, pcl::PointCloud<pcl::PointXYZ> &cloud);//Mandatory, base method implemented as pure virtual
  //Optional
  virtual bool CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius);//Optional
private:
  friend class GraphFactory;// objects of type <template_map_type> are created by using teh factory design pattern, don't forget to register <template_map_type> for creation in factory

  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    //ar & map_ ...
  }
  /*-----End of Boost serialization------*/
};

class TemplateMapParam : public MapParam{
public:
  ~TemplateMapParam(){}
  void GetParametersFromRos();
  string SuperImportantMapParameter;
protected:
  TemplateMapParam();
private:
  friend class GraphFactory;
  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version);
  /*-----End of Boost serialization------*/

};



}


#endif // TEMPLATE_MAP_TYPE_H
