#ifndef MCL_NDT_H
#define MCL_NDT_H
#include "graph_localisation/localisation_factory.h"//must be included first
#include "graph_localisation/localisation_type.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ndt_mcl/3d_ndt_mcl.h"
#include <angles/angles.h>

namespace GraphMapLocalisation{

class MCLNDTType:public LocalisationType{
public:
  MCLNDTType(LocalisationParamPtr param);
  ~MCLNDTType(){}
   Eigen::Affine3d GetPose(){ return pf.getMean ();}
  void InitializeLocalization(const Eigen::Affine3d &pose,const Vector6d &variance); //Vector3d variance={Vx, Vy, Vz Vp Vr Vy}
  void UpdateAndPredict(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tmotion);
protected:
  ParticleFilter3D pf; 						///<This is the particle filter
  NDTMap * map_;
  double resolution;
  double resolution_sensor;//ok
  int counter;//ok
  bool forceSIR;//ok
  double SIR_varP_threshold;//ok
  int SIR_max_iters_wo_resampling;//ok
  unsigned int n_particles_=250;//ok
  std::vector<double> motion_model, motion_model_offset; //pl
  bool initialized_=false;//ok
  int sinceSIR; //ok
  double subsample_level=1.0;
private:
 inline double getDoubleTime();
 inline void normalizeEulerAngles(Eigen::Vector3d &euler);
  friend class LocalisationFactory;
};
class MCLNDTParam:public LocalisationParam{
public:
  void GetParamFromRos(){}
  MCLNDTParam(){}
  ~MCLNDTParam(){}
  double resolution;
  int counter;
  bool forceSIR;
  double SIR_varP_threshold;
  int SIR_max_iters_wo_resampling;
  std::vector<double> motion_model, motion_model_offset;
  int sinceSIR;
private:
  friend class LocalisationFactory;
};
}

#endif // MCL_NDT_H
