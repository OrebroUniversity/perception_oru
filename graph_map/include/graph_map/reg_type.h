#ifndef REGISTRATIONTYPE_H
#define REGISTRATIONTYPE_H
#include "graphfactory.h"
#include "boost/shared_ptr.hpp"
#include "Eigen/Dense"
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"

namespace libgraphMap{

class registrationParameters{
public:
  virtual ~registrationParameters()=0;
  virtual void GetParametersFromRos();
  bool enableRegistration_;
  bool registration2d_;
  bool do_soft_constraints_;
  bool checkConsistency_;
  double maxTranslationNorm_,maxRotationNorm_;
  double translationRegistrationDelta_, rotationRegistrationDelta_;
  double sensorRange_;
  double mapSizeZ_;
 // lslgeneric::MotionModel2d motion_model_2d_;
//  lslgeneric::MotionModel3d motion_model_3d_;
protected:
  registrationParameters();
private:
  friend class GraphFactory;
};

class registrationType{
public:

  virtual ~registrationType()=0;
  virtual bool Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<pcl::PointXYZ> &cloud,Matrix6d covar=unit_covar){}//This methods attempts to register the point cloud versus the map using the affine transformation guess "Tm"
  virtual bool RegisterMap2Map(MapTypePtr map_prev,MapTypePtr map_next, Eigen::Affine3d &Tdiff,double match_score){}
  bool enableRegistration_;
  bool registration2d_;
  bool do_soft_constraints_;
  bool checkConsistency_;
  double maxTranslationNorm_,maxRotationNorm_;
  double translationRegistrationDelta_, rotationRegistrationDelta_;
  double sensorRange_;
  double mapSizeZ_;
  unsigned int  failed_registrations_;
  unsigned int  succesfull_registrations_;
  Eigen::Affine3d sensorPose_;//Translation between robot and sensor frame
protected:
  registrationType(const Affine3d &sensor_pose,RegParamPtr regparam);
private:
  friend class GraphFactory;
};





}
#endif // REGISTRATIONTYPE_H
