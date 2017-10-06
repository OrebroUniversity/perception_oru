#ifndef LOCALISATIONTYPE_H
#define LOCALISATIONTYPE_H
#include "graph_localisation/localisation_factory.h"//must be included first
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "graph_map/graph_map_navigator.h"
#include "graph_map/graph_map.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <velodyne_pointcloud/point_types.h>
namespace perception_oru{
namespace GraphMapLocalisation{
using namespace libgraphMap;

class LocalisationParam{
public:
  LocalisationParam();
  ~LocalisationParam();
  virtual void GetParamFromRos(){}
  GraphMapNavigatorPtr graph_map_;
private:
  friend class LocalisationFactory;
};

class LocalisationType
{
public:
  LocalisationType(LocalisationParamPtr param){
    param_=param;
    graph_map_=param->graph_map_;
    initialized_=false;
  }
  virtual void InitializeLocalization(const Eigen::Affine3d &pose,const Vector6d &variance)=0; //Vector3d variance={Vx, Vy, Vz Vp Vr Vy}
  virtual void UpdateAndPredict(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tmotion)=0;
  virtual void UpdateAndPredict(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, const Eigen::Affine3d &Tmotion)=0;
  virtual ~LocalisationType()=0;
  virtual Eigen::Affine3d GetPose()=0;//return pose of robot in world frame
  virtual std::string ToString();
  protected:
  bool initialized_=false;
  GraphMapNavigatorPtr graph_map_;
  Eigen::Affine3d pose_=Eigen::Affine3d::Identity();
  LocalisationParamPtr param_;//holds the parameters for localisation, can be used to initialize or Reinitialize
private:
    friend class LocalisationFactory;
};
}

}
#endif // LOCALISATIONTYPE_H
