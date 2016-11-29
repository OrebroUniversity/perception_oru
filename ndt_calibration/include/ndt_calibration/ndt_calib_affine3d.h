#pragma once

#include <string>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <angles/angles.h>

Eigen::Affine3d getAsAffine3d(const Eigen::Vector3d &transl, const Eigen::Vector3d &euler) {
  Eigen::Affine3d T;
  {
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
    Eigen::Translation3d v(transl);
    T = v*m;
  }
  return T;
}

Eigen::Affine3d getAsAffine3dFromTranslationEulerAngles(const Eigen::VectorXd &x) {
  Eigen::Affine3d T;
  {
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(x[3], Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(x[4], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(x[5], Eigen::Vector3d::UnitZ());
    Eigen::Translation3d v(x[0], x[1], x[2]);
    T = v*m;
  }
  return T;
}

// 
inline void normalizeEulerAngles(Eigen::Vector3d &euler) {
  if (fabs(euler[0]) > M_PI/2) {
    euler[0] += M_PI;
    euler[1] += M_PI;
    euler[2] += M_PI;
    
    euler[0] = angles::normalize_angle(euler[0]);
    euler[1] = angles::normalize_angle(euler[1]);
    euler[2] = angles::normalize_angle(euler[2]);
  }
}

std::string affine3dToString(const Eigen::Affine3d &T) {
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::digits10);
  Eigen::Vector3d rot = T.rotation().eulerAngles(0,1,2);
  normalizeEulerAngles(rot);
  
  stream << T.translation().transpose() << " " << rot.transpose();
  return stream.str();
}

Eigen::VectorXd getTranslationEulerAnglesVectorFromAffine3d(const Eigen::Affine3d &T) {
  Eigen::VectorXd ret(6);
  ret[0] = T.translation()[0];
  ret[1] = T.translation()[1];
  ret[2] = T.translation()[2];

  Eigen::Vector3d rot = T.rotation().eulerAngles(0,1,2);
  normalizeEulerAngles(rot);
  ret[3] = rot[0];
  ret[4] = rot[1];
  ret[5] = rot[2];
  
  return ret;
}
