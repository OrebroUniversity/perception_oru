#include "graph_map/graph_map.h"

using namespace std;
namespace libgraphMap{

GraphMap::GraphMap(const Affine3d &nodepose,const MapParamPtr &mapparam,const GraphParamPtr graphparam){
  prevNode_=NULL;
  currentNode_=GraphFactory::CreateMapNode(nodepose,mapparam);//The first node to be added
  nodes_.push_back(currentNode_);
  mapparam_=mapparam;
  use_submap_=graphparam->use_submap_;
  interchange_radius_=graphparam->interchange_radius_;
  compound_radius_=graphparam->compound_radius_;

  cout<<"interchange_radius_ set to: "<<interchange_radius_<<endl;
  cout<<"use_submap_ set to: "<<use_submap_<<endl;

  //double min_size=mapparam->sizex_< mapparam->sizey_? mapparam->sizex_ : mapparam->sizey_; //select min map length
  //interchange_radius_=min_size/2-mapparam_->max_range_;//

}
MapNodePtr GraphMap::GetCurrentNode(){
  return currentNode_;
}
MapNodePtr GraphMap::GetPreviousNode(){
  return prevNode_;
}
void GraphMap::AddMapNode(const MapParamPtr &mapparam, const Affine3d &diff, const Matrix6d &cov){ //Add node with link uncertainty

  Affine3d newNodePose=Affine3d::Identity();
  newNodePose= currentNode_->GetPose()*diff;
  MapNodePtr newNode=GraphFactory::CreateMapNode(newNodePose,mapparam);
  FactorPtr sd=GraphFactory::CreateMapNodeFactor(currentNode_,newNode,diff,cov);
  factors_.push_back(sd);//Add connection between current and new node with link diff and covariance
  nodes_.push_back(newNode);
  currentNode_=newNode;
}
string GraphMap::ToString(){
  string s="Graph map: \n";
  for(int i=0;i<nodes_.size();i++){
    NodePtr ptr=nodes_[i];
    s=s+ptr->ToString()+"\n";
  }
  return s;
}
Affine3d GraphMap::GetNodePose(int nodeNr){
  if(nodeNr<GraphSize())
    return nodes_[nodeNr]->GetPose();
}
Affine3d GraphMap::GetCurrentNodePose(){
  return currentNode_->GetPose();
}
Affine3d GraphMap::GetPreviousNodePose(){
  return prevNode_->GetPose();
}
NodePtr GraphMap::GetNode(int nodeNr){
  if(nodeNr<GraphSize()){
    return nodes_[nodeNr];
  }
}
/*std::vector<FactorPtr> GraphMap::GetFactors(NodePtr node){
  std::vector<FactorPtr> factors;
  for(int i=0;i<factors_.size();i++){
    if(factors_[i]->Connects(node)){
      factors_[i]
    }
  }
  return factors;
}*/
void GraphMap::AddFactor(MapNodePtr prev, MapNodePtr next,Affine3d Tdiff,Matrix6d cov){
 factors_.push_back(GraphFactory::CreateMapNodeFactor(prev,next,Tdiff,cov));
}

GraphParam::GraphParam(){}
void GraphParam::GetParametersFromRos(){
  ros::NodeHandle nh("~");
  nh.param("use_submap",use_submap_,false);
  nh.param("interchange_radius",interchange_radius_,100.0);
  nh.param("compound_radius",compound_radius_,100.0);
  nh.param("use_keyframe",use_keyframe_,true);
  nh.param("min_keyframe_dist",min_keyframe_dist_,0.5);
  nh.param("min_keyframe_rot_deg",min_keyframe_rot_deg_,15.0);
}
}
