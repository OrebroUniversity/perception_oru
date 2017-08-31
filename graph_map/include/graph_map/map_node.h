#ifndef MAP_NODE_H
#define MAP_NODE_H
#include "graphfactory.h"
#include "graph_map/map_type.h"
#include "boost/shared_ptr.hpp"
#include "Eigen/Dense"
#include "stdio.h"
#include <iostream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include "boost/serialization/shared_ptr.hpp"
#include "ndt_generic/serialization.h"
#include <velodyne_pointcloud/point_types.h>

namespace libgraphMap{
/*!
* ... Class to represent a node ...
*/

class Node{
protected:
public:
  bool operator ==(const Node& node_compare);
  virtual string ToString(){return "base node";}
  virtual Affine3d GetPose() const;
  virtual bool WithinRadius(const Affine3d &pose, const double &radius);
  virtual unsigned int GetId()const{return id_;}
  Node();
protected:
  unsigned int id_;
  Eigen::Affine3d pose_;
private:
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    ar & id_;
    ar & pose_;
  }
};
/*!
* ... Class to represent a map node ...
*/
class MapNode:public Node{

public:
//  template<class PointT> void updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<PointT> &cloud);
  void updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud);
  void updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud);
  virtual bool Initialized(){return initialized_;}
  virtual MapTypePtr GetMap(){return map_;}
  virtual string ToString();
  MapNode();
protected:
  MapNode(const Eigen::Affine3d &pose,const MapParamPtr &mapparam);
  MapTypePtr map_=NULL;
  bool initialized_=false;
private:
  friend class GraphFactory;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    ar & boost::serialization::base_object<Node>(*this);
    ar & map_;
    ar & initialized_;
  }


};


}
#endif // MAP_NODE_H
