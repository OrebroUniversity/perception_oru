#ifndef MCL_NDT_H
#define MCL_NDT_H
#include "graph_localisation/localisation_type.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "graph_localisation/localisation_factory.h"
namespace GraphMapLocalisation{

class MCLNDTType:public LocalisationType{
public:
  MCLNDTType(LocalisationParamPtr param);
  MCLNDTType();
  ~MCLNDTType();
  void UpdateAndPredict(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tmotion);
private:
  friend class LocalisationFactory;
};
class MCLNDTParam:public LocalisationParam{
public:
  void GetParamFromRos(){}
  MCLNDTParam();
  ~MCLNDTParam();
private:
  friend class LocalisationFactory;
};
}

#endif // MCL_NDT_H
