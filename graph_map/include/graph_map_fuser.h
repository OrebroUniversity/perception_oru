#ifndef GRAPH_MAP_FUSER_H
#define GRAPH_MAP_FUSER_H
#include "stdio.h"
#include "iostream"
#include "graphfactory.h"
#include "graph_map/map_type.h"
#include "graph_map/graph_map_navigator.h"
#include "graph_map/reg_type.h"
#include "ndt/ndtd2d_reg_type.h"
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"
//#include "gnuplot-iostream.h"
#include "ndt_map/ndt_map.h"
#include "ndt_generic/motion_model_2d.h"
#include "ndt_generic/eigen_utils.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include "boost/serialization/shared_ptr.hpp"


namespace libgraphMap{
using namespace std;
using namespace lslgeneric;
using lslgeneric::MotionModel2d;
using Eigen::Affine3d;
class GraphMapFuser{
public:
  GraphMapFuser(){}
  GraphMapFuser(string maptype, string registratorType, const Eigen::Affine3d &init_pose, const Affine3d &sensorPose);//Ros friendly constructor to read parameters from ros-par-server
  GraphMapFuser(  RegParamPtr regParam,  MapParamPtr mapParam, GraphParamPtr graph_param, const Eigen::Affine3d &init_pose, const Eigen::Affine3d &sensorPose);
  virtual void SaveGraphMap(const std::string &filename);
  virtual void SetMotionParameters(const MotionModel2d &motion_param){motion_model_2d_=motion_param;}
  virtual void ProcessFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d &Tnow, const Eigen::Affine3d &Tmotion); //cloud is the current scan in robot frame,  Tnow is the current pose in world frame
  virtual bool ErrorStatus(string status="");
  virtual Affine3d GetPoseLastFuse() const{return pose_last_fuse_;}
  unsigned int FramesProcessed() const{return nr_frames_;}
  virtual void Visualize(bool enableVisualOutput){visualize_=enableVisualOutput;}
protected:
  //virtual void GetParameters();//Get fuser specific parameters, map specific parameters are preferably read inside a class derived from map_paramers
  bool KeyFrameBasedFuse(const Affine3d &Tnow );
  void plotGTCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud);
  ros::NodeHandle n_;
  string maptype_,registratorType_;
  Eigen::Affine3d initPose_,sensorPose_,pose_last_fuse_;
  MapParamPtr mapParam_;
  GraphMapNavigatorPtr graph_map_;
  GraphParamPtr graph_param_;
  RegParamPtr regParam_;
  RegTypePtr registrator_;
  unsigned int nr_frames_;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  MotionModel2d motion_model_2d_;
  bool initialized_=false;
  bool visualize_=false;
  bool use_keyframe_=true;
  double min_keyframe_dist_=0.5;
  double min_keyframe_rot_deg_=15;

};

}
#endif // GRAPH_MAP_FUSER_H
