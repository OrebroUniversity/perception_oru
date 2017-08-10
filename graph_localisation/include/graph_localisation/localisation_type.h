#ifndef LOCALISATIONTYPE_H
#define LOCALISATIONTYPE_H
#include "graph_localisation/localisation_factory.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
namespace GraphMapLocalisation{
class LocalisationType
{
public:
  LocalisationType();
  LocalisationType(LocalisationParamPtr param);
  virtual void UpdateAndPredict(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tmotion)=0;
  //  ~LocalisationType(){}
};

class LocalisationParam{
public:
  LocalisationParam();
  virtual ~LocalisationParam()=0;
  void GetParamFromRos(){}
private:
  friend class LocalisationFactory;

};
}
#endif // LOCALISATIONTYPE_H
