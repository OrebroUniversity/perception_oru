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
protected:
  Node();
  unsigned int id_;
  Eigen::Affine3d pose_;
};
/*!
* ... Class to represent a map node ...
*/
class MapNode:public Node{

public:
  virtual void updateMap(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud);
  virtual bool Initialized(){return initialized_;}
  virtual MapTypePtr GetMap(){return map_;}
  virtual string ToString();
protected:
  MapTypePtr map_;
  MapNode(const Eigen::Affine3d &pose,const MapParamPtr &mapparam);
  bool initialized_=false;
private:
  friend class GraphFactory;
  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version);
  /*-----End of Boost serialization------*/

};


}
#endif // MAP_NODE_H
