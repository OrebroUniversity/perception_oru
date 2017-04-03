#pragma once

#include <ndt_rviz/ndt_rviz.h>
#include <ndt_mcl/3d_ndt_mcl.h>

namespace ndt_visualisation {

visualization_msgs::Marker markerParticlesNDTMCL3D(const NDTMCL3D &mcl, int color, const std::string& ns) {
  visualization_msgs::Marker m;
  assignDefault(m);
  m.ns = ns;
  m.id = 0;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.scale.x = 0.005;
  m.color.g = m.color.a = 1.0;
  m.color.r = 0.6; m.color.b = 0.8;
  if (color >= 0) {
    assignColor(m, color);
  }
  for (unsigned int i = 0; i < mcl.pf.size(); i++) {
    const Eigen::Affine3d &T = mcl.pf.pcloud[i].T;
    m.points.push_back(toPointFromEigen(T.translation()));
    // Length of the line -> 0.3.
    Eigen::Vector3d p2 = T*Eigen::Vector3d(0.3, 0., 0.);
    m.points.push_back(toPointFromEigen(p2));
  }  
  return m;
}

} // namespace
