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
#include <stdio.h>
#include "ros/ros.h"


using namespace std;
namespace libgraphMap{


/*!
 * ... Abstract class to present parameters for "mapType". this class contain generic parameters forr all map types.  ...
 */

class MapParam{
public:
  virtual ~MapParam()=0;
  string getMapName() const{return mapName_;}
  virtual void GetParametersFromRos();
  virtual string ToString();
  double sizex_;
  double sizey_;
  double sizez_;
  double max_range_;
  double min_range_;
  bool enable_mapping_;
protected:
  MapParam();
  string mapName_;
  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){}
  /*-----End of Boost serialization------*/
};




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
  virtual bool CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius=5);
  virtual string GetMapName()const{return mapName_;}
protected:
  MapType(MapParamPtr param);
  //double radius_;
  double sizex_;
  double sizey_;
  double sizez_;
  double max_range_;
  double min_range_;
  bool initialized_;
  bool enable_mapping_=true;
  string mapName_;


  /*-----Boost serialization------*/
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version){
    ar & sizex_ & sizey_ & sizez_;
    ar & max_range_ & max_range_;
    ar & initialized_;
    ar & mapName_;
  }
  /*-----End of Boost serialization------*/
};





}
#endif // MAPTYPE_H
