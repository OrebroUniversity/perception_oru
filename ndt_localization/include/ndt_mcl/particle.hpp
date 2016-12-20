#ifndef PARTICLE_HPP
#define PARTICLE_HPP
//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace lslgeneric{
  class particle{
  
    Eigen::Affine3d pose;
    double r_,p_,t_,x_,y_,z_;

    double likelihood;
  public:

    double probability;
    particle();
    particle(double roll,double pitch,double yaw,double x, double y, double z);
    particle(Eigen::Affine3d pose_);

    double GetLikelihood();
    double GetProbability();
    void GetXYZ(double &x, double &y, double &z);
    void GetRPY(double &r,double &p,double &y);

    void Set(double roll,double pitch,double yaw,double x, double y, double z);
    void SetLikelihood(double l_);
    void SetProbability(double p_);

    Eigen::Affine3d GetAsAffine();
  };
}
#endif
