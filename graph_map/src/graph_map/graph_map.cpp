#include "graph_map/graph_map.h"

using namespace std;
namespace libgraphMap{

GraphMap::GraphMap(const Affine3d &nodepose,const MapParamPtr &mapparam,const GraphParamPtr graphparam){
  factors_.clear();
  prevNode_=NULL;
  currentNode_=GraphFactory::CreateMapNode(nodepose,mapparam);//The first node to be added
  nodes_.push_back(currentNode_);
  mapparam_=mapparam;
  use_submap_=graphparam->use_submap_;
  interchange_radius_=graphparam->interchange_radius_;
  compound_radius_=graphparam->compound_radius_;
  use_keyframe_=graphparam->use_keyframe_;
   min_keyframe_dist_=graphparam->min_keyframe_dist_;
   min_keyframe_rot_deg_=graphparam->min_keyframe_rot_deg_;

  //double min_size=mapparam->sizex_< mapparam->sizey_? mapparam->sizex_ : mapparam->sizey_; //select min map length
  //interchange_radius_=min_size/2-mapparam_->max_range_;//

}
GraphMap::GraphMap(){
  currentNode_=NULL;
  prevNode_=NULL;
  nodes_.clear();//Vector of all nodes in graph
  factors_.clear();
  mapparam_=NULL;//
  bool use_submap_=false;
  double interchange_radius_=0;
  double compound_radius_=0;
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
  std::stringstream ss;
  ss<<"GraphMap:"<<endl;
  ss <<"Graph size="<<nodes_.size()<<endl;
  ss <<"Interchange: "<<interchange_radius_<<endl;
  ss <<"Compound_radius: "<<compound_radius_<<endl;
  ss <<"use keyframe: "<<boolalpha<<use_keyframe_<<endl;
  for(int i=0;i<nodes_.size();i++){
    if(i==0)
      ss <<"Node positions:"<<endl;

    NodePtr ptr=nodes_[i];
    Eigen::Vector3d position=ptr->GetPose().translation();
    ss<<"node "<<i<<" (x,y,z):("<<position(0)<<","<<position(1)<<","<<position(2)<<")"<<endl;
  }
  if(currentNode_!=NULL)
    ss<<"Detailed info node 0:"<<endl<<currentNode_->ToString();

  return ss.str();
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
