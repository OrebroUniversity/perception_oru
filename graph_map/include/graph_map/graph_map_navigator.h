#ifndef GRAPH_NAVIGATOR_INTERFACE_H
#define GRAPH_NAVIGATOR_INTERFACE_H
#include "graph_map/graph_map.h"
#include "graph_map/map_node.h"
#include "graph_map/map_type.h"
#include "graphfactory.h"
#include "Eigen/Dense"

namespace libgraphMap{

using namespace lslgeneric;
class GraphMapNavigator:public GraphMap{
public:
  GraphMapNavigator(const Eigen::Affine3d &nodepose, const MapParamPtr &mapparam, const GraphParamPtr graphparam);
  bool SwitchToClosestMapNode(Affine3d &Tnow, const Matrix6d &cov, Affine3d & T_world_to_local_map,const double radius);
  bool AutomaticMapInterchange(Affine3d &Tnow, const Matrix6d &cov, Affine3d & T_world_to_local_map,bool &changed_map_node,bool &created_map_node);

};

}

#endif // GRAPH_NAVIGATOR_INTERFACE_H
