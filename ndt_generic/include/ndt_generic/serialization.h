#ifndef SERIALIZATION_H
#define SERIALIZATION_H


#include <boost/serialization/split_free.hpp>
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include <boost/serialization/array.hpp>
#include "serialization.h"

// Forward declaration of class boost::serialization::access
namespace boost {
namespace serialization {
class access;
// Serialization of Eigen::Vector2d
/* Already exists in semrob package semrob_generic as a typedef to pose2d
template<typename Archive>
void serialize(Archive& ar, Eigen::Vector3d& o, const unsigned int version) {
  ar & o[0] & o[1] & o[2];
}*/

template<typename Archive>
void serialize(Archive & ar, Eigen::Matrix<double, 3, 3> & m, const unsigned int /*version*/) {
  ar & boost::serialization::make_array(m.data(), 3 * 3);
}
template<typename Archive>
void serialize(Archive & ar, Eigen::Matrix<double, 6, 6> & m, const unsigned int /*version*/) {
  ar & boost::serialization::make_array(m.data(), 6 * 6);
}
#ifdef MATRIX_D31
#define MATRIX_D31
template<typename Archive>
void serialize(Archive & ar, Eigen::Matrix<double, 3, 1> & m, const unsigned int /*version*/) {
  ar & boost::serialization::make_array(m.data(), 3 * 1);
}
#endif

template<typename Archive>
void serialize(Archive& ar, std::vector<pcl::PointXYZ,Eigen::aligned_allocator<pcl::PointXYZ> > points, const unsigned version) {
  for(int i=0; i<points.size();i++)
    ar & points[i];
}

template<typename Archive>
void serialize(Archive& ar,  pcl::PointCloud<pcl::PointXYZ> points, const unsigned version) {
  for(int i=0; i<points.size();i++)
    ar & points[i];
}
template<typename Archive>
void serialize(Archive& ar, pcl::PointXYZ point, const unsigned version) {
  ar & point.x & point.y & point.z;
}

   template<typename Archive>
    void serialize(Archive& ar, Eigen::Affine3d &o, const unsigned version) {
      for (int i = 0; i < 16; i++) {
        ar & o.data()[i];
      }
    }


}
}


#endif // SERIALIZATION_H
