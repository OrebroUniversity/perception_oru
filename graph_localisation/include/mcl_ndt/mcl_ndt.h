#ifndef MCL_NDT_H
#define MCL_NDT_H
#include "graph_localisation/localisation_factory.h"//must be included first
#include "graph_localisation/localisation_type.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ndt_mcl/3d_ndt_mcl.h"
#include <angles/angles.h>
#include <velodyne_pointcloud/point_types.h>

namespace GraphMapLocalisation{

class MCLNDTType:public LocalisationType{
public:
  MCLNDTType(LocalisationParamPtr param);
  ~MCLNDTType(){}
   Eigen::Affine3d GetPose(){ return pose_;}
  void InitializeLocalization(const Eigen::Affine3d &pose,const Vector6d &variance); //Vector3d variance={Vx, Vy, Vz Vp Vr Vy}
  void UpdateAndPredict(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tmotion);
  void UpdateAndPredict(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, const Eigen::Affine3d &Tmotion);
  std::string ToString();
protected:
  NDTMap* GetCurrentNodeNDTMap();
  ParticleFilter3D pf; 						///<This is the particle filter
  NDTMap * map_;
  double resolution=0.5;
  double resolution_sensor=0.5;//ok
  int counter=0;//ok
  bool forceSIR=false;//ok
  double SIR_varP_threshold=0.6;
  int SIR_max_iters_wo_resampling_=25;
  unsigned int n_particles_=250;
  std::vector<double> motion_model, motion_model_offset;
  bool initialized_=false;
  int sinceSIR_=0; //ok
  double subsample_level_=1.0;
  double z_filter_min=-10000.0;
  double score_cell_weight=0.1;
  inline double getDoubleTime();
  inline void normalizeEulerAngles(Eigen::Vector3d &euler);
  private:
  friend class LocalisationFactory;
};
class MCLNDTParam:public LocalisationParam{
public:
  void GetParamFromRos(){}
  MCLNDTParam();
  ~MCLNDTParam(){}
  bool forceSIR=false;
  double SIR_varP_threshold=0.6;
  int SIR_max_iters_wo_resampling_=30;
  unsigned int n_particles_=250;
  int SIR_max_iters_wo_resampling=25;
  double z_filter_min=-10000.0;
  double score_cell_weight=0.1;
  double resolution=0;
  std::vector<double> motion_model, motion_model_offset;
private:
  friend class LocalisationFactory;
};
}

#endif // MCL_NDT_H
