/**
 *  file map_type.h.
 */
#ifndef MAPTYPE_H
#define MAPTYPE_H
#include "graphfactory.h"
#include "eigen3/Eigen/Dense"
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include "boost/serialization/serialization.hpp"
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include <velodyne_pointcloud/point_types.h>

using namespace std;
namespace libgraphMap{

/*!
 * ... Abstract class to present parameters for "mapType". this class contain generic parameters forr all map types.  ...
 */





/*!
 * ... Abstract class to implement local maps.  ...
 */

class MapType{

public:
  /*!
   * \brief update attempts to update the map based on point cloud data and the pose where the scan was taken from(scanner) in the world fram
   * \param Tnow transformation from world to sensor frame
   * \param cloud data to update map with
   */
  virtual bool Initialized() const{return initialized_;}
  virtual void update(const Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud)=0;
  virtual void update(const Eigen::Affine3d &Tnow,pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud)=0;
  virtual bool CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius=5);
  virtual string GetMapName()const{return mapName_;}
  virtual string ToString();
  MapType();
protected:
  MapType(MapParamPtr param);
  double sizex_=0;
  double sizey_=0;
  double sizez_=0;
  double max_range_=30;
  double min_range_=0.6;
  bool initialized_=false;
  bool enable_mapping_=true;
  string mapName_="";

  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    ar & sizex_ & sizey_ & sizez_;
    ar & max_range_ & min_range_;
    ar & initialized_;
    ar & enable_mapping_;
    ar & mapName_;
  }
};

class MapParam{
public:
  virtual ~MapParam()=0;
  string getMapName() const{return mapName_;}
  virtual void GetParametersFromRos();
  virtual string ToString();
  double sizex_=0;
  double sizey_=0;
  double sizez_=0;
  double max_range_=30;
  double min_range_=0.6;
  bool enable_mapping_=true;
protected:
  MapParam(){}
  string mapName_="";
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & sizex_;
    ar & sizey_;
    ar & sizez_;
    ar & max_range_;
    ar & min_range_;
    ar & enable_mapping_;
  }
};
}
#endif // MAPTYPE_H
