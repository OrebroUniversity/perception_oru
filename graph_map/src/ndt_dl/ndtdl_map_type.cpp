#include "ndt_dl/ndtdl_map_type.h"
#include <boost/serialization/export.hpp>
#include <ndt_dl/point_curv.h>

BOOST_CLASS_EXPORT(libgraphMap::NDTDL)
namespace libgraphMap{
  using namespace std;


  NDTDL::NDTDL( MapParamPtr paramptr) : MapType(paramptr){
    NDTDLMapParamPtr param = boost::dynamic_pointer_cast< NDTDLMapParam >(paramptr);//Should not be NULL
    if(param!=NULL){
      resolution_=param ->resolution_;
      map_flat_ = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution_));
      map_flat_->initialize(0.0,0.0,0.0,param->sizex_,param->sizey_,param->sizez_);
      map_edge_ = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution_));
      map_edge_->initialize(0.0,0.0,0.0,param->sizex_,param->sizey_,param->sizez_);
      cout<<"created ndtdlmap"<<endl;
    }
    else
      cerr<<"templateMapType: Cannot create instance for \"templateMapType\""<<std::endl;
  }
  NDTDL::~NDTDL(){}

  void NDTDL::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.

    cout<<"The NDT-DL update with PointXYZ is please implement map update for NDT-DL"<<endl;
    if(initialized_){
      //Initialize map
    }else{
      //Update map
      initialized_ = true;
    }
  }

  void NDTDL::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.


    // Segment the point based on curvature


    pcl::PointCloud<pcl::PointXYZ> cornerPointsSharp;
    pcl::PointCloud<pcl::PointXYZ> cornerPointsLessSharp;
    pcl::PointCloud<pcl::PointXYZ> surfPointsFlat;
    pcl::PointCloud<pcl::PointXYZ> surfPointsLessFlat;

    segmentPointCurvature(cloud, cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat);
    // Add the different point clouds into different maps for now only use the flat ones.

    ROS_INFO_STREAM("flatpoints size : " << surfPointsLessFlat.size());
    ROS_INFO_STREAM("edgepoints size : " << cornerPointsLessSharp.size());


    if(initialized_ && enable_mapping_){
      ROS_ERROR_STREAM("update");
      Eigen::Vector3d localMapSize(max_range_,max_range_,sizez_);
      map_flat_->addPointCloudMeanUpdate(Tsensor.translation(),surfPointsLessFlat,localMapSize, 1e5, 25, 2*sizez_, 0.06);
      map_edge_->addPointCloudMeanUpdate(Tsensor.translation(),cornerPointsLessSharp,localMapSize, 1e5, 25, 2*sizez_, 0.06);
    }
    else if(!initialized_){
      InitializeMap(Tsensor,surfPointsLessFlat, cornerPointsSharp);
      initialized_ = true;
    }
  }

  void NDTDL::InitializeMap(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloudFlat, pcl::PointCloud<pcl::PointXYZ> &cloudEdge){
    cout<<"initialize map"<<endl;
    map_flat_->addPointCloud(Tsensor.translation(),cloudFlat, 0.1, 100.0, 0.1);
    map_flat_->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tsensor.translation(), 0.1);

    map_edge_->addPointCloud(Tsensor.translation(),cloudEdge, 0.1, 100.0, 0.1);
    map_edge_->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tsensor.translation(), 0.1);
  }

  bool NDTDL::CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius){

    return true;
    cout<<"please implement map compound for improved usage of submaps"<<endl;
    if( NDTDLMapTypePtr targetPtr=boost::dynamic_pointer_cast<NDTDL>(target) ){

      cout<<"\"CompoundMapsByRadius\" not overrided by template but not implemented"<<endl;
    }
  }




}

