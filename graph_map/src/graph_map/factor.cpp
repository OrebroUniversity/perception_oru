#include "graph_map/factor.h"
namespace perception_oru{
namespace libgraphMap{
bool factor::Connects(NodePtr node){
  if(node==prev_ || node==next_){
    return true;
  }
}
factor::factor(){
  diff_=Eigen::Affine3d::Identity();
  prev_=NULL;
  next_=NULL;
  covar_=unit_covar;
}

}

}
