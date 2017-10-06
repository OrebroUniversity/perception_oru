#include "graph_map/graph_map_fuser.h"
namespace perception_oru{
namespace libgraphMap{
using namespace std;
GraphMapFuser::GraphMapFuser(string maptype, string registratorType, const Eigen::Affine3d &init_pose, const Eigen::Affine3d &sensorPose){
  graph_param_=GraphFactory::CreateGraphParam();
  graph_param_->GetParametersFromRos();
  regParam_=GraphFactory::CreateRegParam(registratorType);
  cout<<"started reading reg par from ros"<<endl;
  regParam_->GetParametersFromRos();
  cout<<"finished reading reg par from ros"<<endl;
  mapParam_=GraphFactory::CreateMapParam(maptype);
  cout<<"started reading map par from ros"<<endl;
  mapParam_->GetParametersFromRos();
  cout<<"time to create graph inside fuser"<<endl;
  graph_map_ =GraphFactory::CreateGraphNavigator(init_pose,mapParam_,graph_param_);
  registrator_=GraphFactory::CreateRegistrationType(sensorPose,regParam_);
  sensorPose_=sensorPose;
  use_keyframe_=graph_param_->use_keyframe_;
  min_keyframe_dist_=graph_param_->min_keyframe_dist_;
  min_keyframe_rot_deg_=graph_param_->min_keyframe_rot_deg_;
  nr_frames_=0;
  initialized_=true;
}
GraphMapFuser::GraphMapFuser(  RegParamPtr regParam,  MapParamPtr mapParam, GraphParamPtr graph_param, const Eigen::Affine3d &init_pose, const Eigen::Affine3d &sensorPose){
  mapParam_=mapParam;
  regParam_=regParam;
  graph_param_=graph_param;
  graph_map_ =GraphFactory::CreateGraphNavigator(init_pose,mapParam_,graph_param_);
  registrator_=GraphFactory::CreateRegistrationType(sensorPose,regParam_);
  sensorPose_=sensorPose;
  nr_frames_=0;
  use_keyframe_=graph_param->use_keyframe_;
  min_keyframe_dist_=graph_param->min_keyframe_dist_;
  min_keyframe_rot_deg_=graph_param->min_keyframe_rot_deg_;
  initialized_=true;
  pose_last_fuse_=init_pose;
}
void GraphMapFuser::SaveGraphMap(const std::string &filename){
  cout<<"-----------------------------Saving---------------------------------\n"<<graph_map_->ToString()<<endl;
  cout<<"----------------------------------------------------------------------\nTo file path:"<<filename<<endl;
  std::ofstream ofs(filename);
  boost::archive::text_oarchive ar(ofs);
  ar << graph_map_;
  ofs.close();
}
void GraphMapFuser::SaveCurrentNodeAsJFF(const std::string &filename){
  cout<<"-----------------------------Saving JFF---------------------------------\n"<<graph_map_->ToString()<<endl;
  cout<<"----------------------------------------------------------------------\nTo file path:"<<filename<<endl;
  NDTMap * map=boost::dynamic_pointer_cast<NDTMapType> (graph_map_->GetCurrentNode()->GetMap())->GetNDTMap();
  std::string name=filename+std::string(".JFF");
  map->writeToJFF(name.c_str());
}
void GraphMapFuser::Visualize(bool enableVisualOutput,plotmarker marker){
  visualize_=enableVisualOutput;
  marker_=marker;
}
//!
//! \brief GraphMapFuser::KeyFrameBasedFuse
//! \param Tnow pose of base specified in the GLOBAL world frame
//! \return true if base has moved outside bounds and can fuse the current frame, otherwise return false. Will always return true the first frame
//!
bool GraphMapFuser::KeyFrameBasedFuse(const Affine3d &Tnow ){
  bool ret = false;
  if(nr_frames_==0) {
    frame_idx_ = 0;
    return true;
  }
  else{
    Affine3d diff=pose_last_fuse_.inverse()*Tnow;
    Eigen::Vector3d Tmotion_euler = diff.rotation().eulerAngles(0,1,2);
    ndt_generic::normalizeEulerAngles(Tmotion_euler);
    if(use_keyframe_ ){
      if(diff.translation().norm()>min_keyframe_dist_ || Tmotion_euler.norm()>(min_keyframe_rot_deg_*M_PI/180.0)) {
        ret = true;
      }
      else {
        ret = false;
      }
    }
    else {
      ret = true;
    }
  }
  frame_idx_++;
  if (ret == true) {
    if (frame_idx_ % 4 == 0) {// Avoid that the scanner covers the same region (one scan covers ~270 degrees angle and the same area is repeaded every 4rd frame)
      ret = false;
    }
    else {
      frame_idx_ = 0;
    }
  }
  return ret;
}

void GraphMapFuser::PlotMapType(){
/*  NDTMapPtr curr_node = boost::dynamic_pointer_cast< NDTMapType >(graph_map_->GetCurrentNode()->GetMap());
  GraphPlot::SendGlobalMapToRviz(curr_node->GetNDTMap(),1,graph_map_->GetCurrentNodePose());*/
  GraphPlot::PlotMap(graph_map_->GetCurrentNode()->GetMap(),-1,graph_map_->GetCurrentNodePose(),marker_);
  GraphPlot::PlotPoseGraph(graph_map_);
}

void GraphMapFuser::plotGTCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud){
  perception_oru::NDTMap ndlocal(new perception_oru::LazyGrid(0.4));
  // for(int i=0;i<cloud.size();i+=500){
  // cout<<cloud[i].x<<","<<cloud[i].y<<","<<cloud[i].z<<endl;
  // }
  ndlocal.guessSize(0,0,0,100,100,8);
  ndlocal.loadPointCloud(cloud,30.0);
  ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
  cout<<"CLOUD SIZE IS="<<cloud.size()<<endl;
  cout<<"ndt cloud size="<<ndlocal.getAllCells().size()<<endl;
  NDTMap * ptrmap=&ndlocal;
  GraphPlot::SendLocalMapToRviz(ptrmap,1);
}

bool GraphMapFuser::ErrorStatus(string status){
  if(graph_map_!=NULL && registrator_!=NULL){
    return false;
  }
  else{
    status="No object instance found for graph or registrator";
    return true;
  }
}
//void GetCovarianceFromMotion(Matrix6d &cov,const Affine3d &Tm){
//}
std::string GraphMapFuser::ToString(){
  std::stringstream ss;
  ss<<"fuser:"<<endl<<"initialized:"<<initialized_<<endl;
  ss<<"visualize:"<<visualize_<<endl;
  ss<<"Key frame based update:"<<std::boolalpha<<use_keyframe_<<endl;
  if(use_keyframe_)
    ss<<"minimum keyframe distance(meter/deg):("<<min_keyframe_dist_<<","<<min_keyframe_rot_deg_<<")"<<endl;
  ss<<registrator_->ToString();
  ss<<graph_map_->ToString();
  return ss.str();
}
}
}
