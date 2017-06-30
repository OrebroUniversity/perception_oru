#include "graph_map/graph_map_navigator.h"
namespace libgraphMap{
GraphMapNavigator::GraphMapNavigator(const Eigen::Affine3d &nodepose, const MapParamPtr &mapparam, const GraphParamPtr graphparam): GraphMap(nodepose, mapparam ,graphparam){}

bool GraphMapNavigator::SwitchToClosestMapNode(Affine3d &Tnow, const Matrix6d &cov, Affine3d & T_world_to_local_map,const double radius){
  bool node_found=false;
  MapNodePtr closest_map_node=NULL;
  double closest_distance=-1.0;
  cout<<"currently at pose="<<Tnow.translation()<<endl;
  for(std::vector<NodePtr>::iterator itr_node = nodes_.begin(); itr_node != nodes_.end(); ++itr_node) { //loop thorugh all existing nodes
    if( (*itr_node)->WithinRadius(Tnow,radius) ){ //if a node is within radius of pose and the node is of type "map type"
      if(  MapNodePtr found_map_node_ptr = boost::dynamic_pointer_cast< MapNode >(*itr_node) ){ //A map node has now been found within radius)
        double found_distance= Eigen::Vector3d(Tnow.translation()-(*itr_node)->GetPose().translation()).norm();//Euclidian distance between robot pose and map node pose;
        cout<<"Node is within range of previously created map, distance="<<found_distance<<endl;
        if(closest_distance==-1.0|| found_distance<closest_distance){
          node_found=true;
          closest_map_node=found_map_node_ptr;
          closest_distance=found_distance;
          cout<<"closest node found at pose=\n"<<closest_map_node->GetPose().translation()<<endl;
        }
      }
      else
        cout<<"wrong type of previously created node"<<endl;
    }
  }
  if(node_found){//if any node at all was found, switch to the closest and update Tnow & T_world_to_local_map (transformation to current node)
    prevNode_=currentNode_;
    currentNode_=closest_map_node; //switch to that node
    AddFactor(prevNode_,currentNode_,prevNode_->GetPose().inverse()*currentNode_->GetPose(),cov);
  }
  return node_found;
}
bool GraphMapNavigator::AutomaticMapInterchange(Affine3d &Tnow, const Matrix6d &cov_incr, Affine3d & T_world_to_local_map,bool &changed_map_node,bool &created_map_node){

  created_map_node=false;
  changed_map_node=false;
  static Matrix6d pose_covar=unit_covar;
  if(use_submap_!=false){
    Tnow=T_world_to_local_map.inverse()*Tnow;//map Tnow to world frame
    if(! currentNode_->WithinRadius(Tnow,interchange_radius_)){ //No longer within radius of node
      cout<<"Left boundries of previous map, will  search through "<<nodes_.size()<<" node(s) to find a map node within range of"<<interchange_radius_<<"m"<<endl;
      if( changed_map_node=SwitchToClosestMapNode(Tnow,cov_incr,T_world_to_local_map,interchange_radius_)){
        cout<<"switched to node="<<currentNode_->GetPose().translation()<<endl;
        //Reset covariance
      }
      else{
        cout<<"No node was found, will create a new map pose."<<endl;
        prevNode_=currentNode_;
        //NDT2DMapPtr prev_ndt_map = boost::dynamic_pointer_cast< NDTMapType >(prevNode_->GetMap());
        AddMapNode(mapparam_,T_world_to_local_map*Tnow,cov_incr); //if no node already exists, create a new node
        prevNode_->GetMap()->CompoundMapsByRadius(currentNode_->GetMap(),prevNode_->GetPose(),currentNode_->GetPose(),compound_radius_);
        created_map_node=true;
      }
    }
    T_world_to_local_map=currentNode_->GetPose().inverse();
    Tnow=T_world_to_local_map*Tnow;//map Tnow from global to new local frame
  }
  return created_map_node || changed_map_node;
}

}
