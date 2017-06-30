#ifndef NDTD2DREGTYPE_H
#define NDTD2DREGTYPE_H
#include "graphfactory.h"
#include "graph_map/reg_type.h"
#include "graph_map/map_type.h"
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include "Eigen/Dense"
#include <ndt_map/ndt_map.h>
#include "ndt_map/lazy_grid.h"
#include "ndt_map/ndt_map_hmt.h"
#include "math.h"
#include <ndt_map/pointcloud_utils.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_registration/ndt_matcher_d2d_sc.h>
#include "ndt/ndt_map_type.h"
#include "visualization/graph_plot.h"

#define ndt_d2d_reg_type_name "ndt_d2d_reg"
namespace libgraphMap{
using namespace lslgeneric;
class NDTD2DRegType:public registrationType{
public:
  ~NDTD2DRegType();
  bool Register(MapTypePtr maptype, Eigen::Affine3d &Tnow, pcl::PointCloud<pcl::PointXYZ> &cloud,Matrix6d covar=unit_covar);//This methods attempts to register the point cloud versus the map using Tmotion as a first guess
  bool RegisterMap2Map(MapTypePtr map_prev,MapTypePtr map_next, Eigen::Affine3d &Tdiff,double match_score);
protected:
  NDTD2DRegType(const Eigen::Affine3d &sensor_pose, RegParamPtr paramptr);
  double resolution_,resolutionLocalFactor_;
  NDTMatcherD2D_2D matcher2D_;
  NDTMatcherD2D matcher3D_;
private:
  friend class GraphFactory;
};


class NDTD2DRegParam:public registrationParameters{
public:
  ~NDTD2DRegParam();
  void GetParametersFromRos();
  double resolution_=0.4, resolutionLocalFactor_=1.0;
  //Matcher
  int  matcher2D_ITR_MAX = 30;
  bool matcher2D_step_control=true;
  int  matcher2D_n_neighbours=2;
NDTD2DRegParam();
protected:

private:
  friend class GraphFactory;

};
}
#endif // NDTD2DREGTYPE_H

