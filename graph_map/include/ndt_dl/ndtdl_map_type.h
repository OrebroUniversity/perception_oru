#ifndef NDTDL_MAP_TYPE_H
#define NDTDL_MAP_TYPE_H

#include "graphfactory.h" //includes the full list of forward declarations
#include <graph_map/map_type.h>
#include <ndt_map/ndt_map_hmt.h>
#include <ndt_dl/ndtdl_map_param.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <ndt_map/ndt_map.h>
#include "ros/ros.h"
#include "ros/node_handle.h"
#include <ndt_map/pointcloud_utils.h>
#include "ndt_generic/serialization.h"
#include "stdio.h"
#include "sstream"

#define ndtdl_map_type_name "ndt_dl_map"
namespace libgraphMap{
using namespace lslgeneric;

class NDTDLMapType:public MapType{
public:
  //Mandatory
  ~NDTDLMapType();
  virtual void update(const Eigen::Affine3d &Tsensor, pcl::PointCloud<pcl::PointXYZ> &cloud, bool simple = false);//Mandatory, base method implemented as pure virtual
  virtual void update(const Eigen::Affine3d &Tsensor, pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, bool simple = false);//Mandatory, base method implemented as pure virtual
  //Optional
  NDTMap* GetNDTMapFlat() { return map_flat_; }
  NDTMap* GetNDTMapEdge() { return map_edge_; }
  virtual bool CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius);//Optional
  double GetResolution() const{return resolution_;}
  NDTDLMapType(MapParamPtr paramptr);
  NDTDLMapType(){}
  NDTMap *map_flat_=NULL;
  NDTMap *map_edge_=NULL;
protected:
  double resolution_=0.4;

  friend class GraphFactory;// objects of type <template_map_type> are created by using teh factory design pattern, don't forget to register <template_map_type> for creation in factory
  void InitializeMap(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloudFlat, pcl::PointCloud<pcl::PointXYZ> &cloudEdge);

  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
     ar & boost::serialization::base_object<MapType>(*this);
     ar & map_flat_;
     ar & map_edge_;
     ar & resolution_;
  }

  /*-----End of Boost serialization------*/
};


}


#endif // NDTDL_MAP_TYPE_H
