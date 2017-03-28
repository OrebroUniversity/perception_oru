#pragma once

#include <angles/angles.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <iostream>

#include <ndt_generic/eigen_utils.h>
#include <ndt_generic/motion_model_2d.h>

//! Motion model (incremental).
/*!
  The only motion model that are really useful is the relative  incremental one.
  That is, given two measurement at t0 and t1, obtain the relative
  odometry pose between t0 and t1 (for example using the tf ROS package).
  This relative incremental pose can then directly be used to get the incremental motion (need to adjust it with the current heading), and to get the covariance of the motion.
*/
namespace lslgeneric
{
class MotionModel3d
{
public:
  //! Holds params for the motion models.
  class Params
  {
  public:
    //! Constructor, initiate to reasonable params.
    Params() 
      {
          // Variance forward from distance
          // Variance forward fom rotation
          // the abouve pairs for x (Dd, Dt), y (Cd, Ct) , z, roll, pitch, yaw (Td, Td) [( for the corresponding 2d motion model )]
          motion_model(0,0) = 0.005;
          motion_model(0,1) = 0.005;

          motion_model(1,0) = 0.005;
          motion_model(1,1) = 0.005;

          motion_model(2,0) = 0.005;
          motion_model(2,1) = 0.005;

          motion_model(2,0) = 0.005;
          motion_model(2,1) = 0.005;

          motion_model(3,0) = 0.005;
          motion_model(3,1) = 0.005;

          motion_model(4,0) = 0.005;
          motion_model(4,1) = 0.005;

          motion_model(5,0) = 0.005;
          motion_model(5,1) = 0.005;
      }
      
      Params(MotionModel2d::Params &params) {

          motion_model(0,0) = params.Dd;
          motion_model(0,1) = params.Dt;

          motion_model(1,0) = params.Cd;
          motion_model(1,1) = params.Ct;

          motion_model(2,0) = 1.;
          motion_model(2,1) = 1.;

          motion_model(2,0) = 1.;
          motion_model(2,1) = 1.;

          motion_model(3,0) = 1.;
          motion_model(3,1) = 1.;

          motion_model(4,0) = 1.;
          motion_model(4,1) = 1.;

          motion_model(5,0) = params.Td;
          motion_model(5,1) = params.Tt;
      }

      Eigen::Matrix<double, 6,2> motion_model;
      
      
      //! Return a one-line condensed string of the parameters
      std::string getDescString() const {
          std::ostringstream os;
          os << motion_model;
          return os.str();
      }
      
    //! Display the parameters.
      friend std::ostream& operator<<(std::ostream &os, const MotionModel3d::Params &obj)
      {
          os << "- motion model - ";
          os << "\n" << obj.getDescString();
          return os;
      }
  };
    
    MotionModel3d() { }
    MotionModel3d(const MotionModel3d::Params &p) : params(p) { }
    
    void setParams(const MotionModel3d::Params &p)
    {
        params = p;
    }
    
    //! Obtain the covariance for the provided relative incremental pose
    Eigen::MatrixXd getCovMatrix(const Eigen::Affine3d &rel) const
    {
        return getMeasurementCov(rel);
    }
    
    MotionModel3d::Params params;
  
private:
  Eigen::MatrixXd getMeasurementCov(const Eigen::Affine3d &rel) const
  {
      double dist = rel.translation().norm();
      Eigen::Vector3d euler = rel.rotation().eulerAngles(0,1,2);
      ndt_generic::normalizeEulerAngles(euler);
      double rot = euler.norm();
      
      Eigen::Matrix<double,2,1> incr;
      incr(0,0) = dist*dist;
      incr(1,0) = rot*rot;

      Eigen::MatrixXd diag = params.motion_model * incr;
      Eigen::MatrixXd R(6,6);
      R.setZero();
      for (int i = 0; i < 6; i++) {
          R(i,i) = diag(i);
      }
      return R;
  }
  
};

} // namespace

