#include "graph_map/factor.h"
namespace libgraphMap{
bool factor::Connects(NodePtr node){
  if(node==prev_ || node==next_){
    return true;
  }
}




}
