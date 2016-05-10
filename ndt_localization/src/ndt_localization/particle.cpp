#include "ndt_mcl/particle.hpp"
#include <iostream>
lslgeneric::particle::particle(){
  probability=0;
  likelihood=0;
  pose=Eigen::Affine3d::Identity();
}


lslgeneric::particle::particle(double roll,double pitch,double yaw,double x, double y, double z){
   this->Set(roll,pitch,yaw,x,y,z);
   //r_=roll;
  // p_=pitch;
  // t_=yaw;
  // x_=x;
  // y_=y;
  // z_=z;
  probability=0;
  likelihood=0;
}
lslgeneric::particle::particle(Eigen::Affine3d pose_){
  pose=pose_;
 probability=0;
  likelihood=0;
}


void lslgeneric::particle::Set(double roll,double pitch,double yaw,double x, double y, double z){
  // r_=roll;
  // p_=pitch;
  // t_=yaw;
  // x_=x;
  // y_=y;
  // z_=z;

   Eigen::Matrix3d orientation;
     orientation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
     *Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
     *Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
     Eigen::Translation3d position(x,y,z);
     //  std::cout<<x<<" "<<y<<" "<<z<<std::endl;
     //pose=position*orientation;
     pose=Eigen::Affine3d::Identity();
     pose.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
     pose.translation()<<x,y,z;
}

void lslgeneric::particle::GetXYZ(double &x, double &y, double &z){

   Eigen::Vector3d tr = pose.translation();
   x = tr[0];
   y = tr[1];
   z = tr[2];
  //x=x_;
  //y=y_;
  //z=z_;
}

void lslgeneric::particle::GetRPY(double &r,double &p,double &y){
  //r=r_;
  //p=p_;
  //y=t_;
   Eigen::Vector3d rot = pose.rotation().eulerAngles(0,1,2);
   r=rot[0];
   p=rot[1];
   y=rot[2];
}

Eigen::Affine3d lslgeneric::particle::GetAsAffine(){
  return pose;
}

void lslgeneric::particle::SetLikelihood(double l_){
  likelihood=l_;
}

void lslgeneric::particle::SetProbability(double p_){
  probability=p_;
}

double lslgeneric::particle::GetLikelihood(){
  return likelihood;
}

double lslgeneric::particle::GetProbability(){
  return probability;
}
