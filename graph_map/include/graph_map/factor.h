#ifndef GRAPHFACTOR_H
#define GRAPHFACTOR_H
#include "graphfactory.h"
#include "graph_map/map_node.h"
#include "graph_map/map_type.h"
#include "Eigen/Dense"
#include "boost/serialization/serialization.hpp"
#include "ndt_generic/serialization.h"
namespace perception_oru{
namespace libgraphMap{
typedef enum factoryype{observationFactor=0,poseFactor=1}factorType;
class factor{
public:

  virtual bool Connects(NodePtr node);
  virtual void GetNodes(NodePtr prev, NodePtr next){prev=prev_;next=next_;}
  //factor();
  //factor(mapNodePtr prev, mapNodePtr next){prev_=prev; next_=next;}
  factor(MapNodePtr prev, NodePtr next,const Eigen::Affine3d &diff,const Matrix6d &cov){

    prev_=prev;
    next_=next;
    diff_=diff;
    covar_=cov;
  }
  factor();
protected:
  //factorType factortype_;
  Eigen::Affine3d diff_;
  NodePtr prev_,next_;
  Matrix6d covar_;
private:
  friend class GraphFactory;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
  //ar & factortype_;
    ar & diff_;
    ar & prev_;
    ar & next_;
    ar & covar_;
  }


};
}
}

#endif // FACTOR_H
