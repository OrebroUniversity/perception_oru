#ifndef REGISTRATIONTYPE_H
#define REGISTRATIONTYPE_H
#include "graphfactory.h"
#include "boost/shared_ptr.hpp"
#include "Eigen/Dense"
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"
#include "iostream"
#include "string"
#include "stdio.h"

namespace libgraphMap{

class registrationParameters{
public:
  virtual ~registrationParameters()=0;
  virtual void GetParametersFromRos();
  bool enableRegistration_=false;
  bool registration2d_=false;
  bool do_soft_constraints_=false;
  bool checkConsistency_=false;
  double maxTranslationNorm_=0,maxRotationNorm_=0;
  double translationRegistrationDelta_=0, rotationRegistrationDelta_=0;
  double sensorRange_=0;
  double mapSizeZ_=0;
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
  virtual std::string ToString();
  bool enableRegistration_=false;
  bool registration2d_=false;
  bool do_soft_constraints_=false;
  bool checkConsistency_=false;
  double maxTranslationNorm_=0,maxRotationNorm_=0;
  double translationRegistrationDelta_=0, rotationRegistrationDelta_=0;
  double sensorRange_=0;
  double mapSizeZ_=0;
  unsigned int  failed_registrations_=0;
  unsigned int  succesfull_registrations_=0;
  Eigen::Affine3d sensorPose_=Eigen::Affine3d::Identity();//Translation between robot and sensor frame
protected:
  registrationType(const Affine3d &sensor_pose,RegParamPtr regparam);
private:
  friend class GraphFactory;
};





}
#endif // REGISTRATIONTYPE_H
