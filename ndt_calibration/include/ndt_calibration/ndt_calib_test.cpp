#pragma once

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <eigen_conversions/eigen_msg.h>

#include <ndt_map/pointcloud_utils.h>
#include <ndt_rviz/ndt_rviz.h>

#include <sstream>

#include <visualization_msgs/MarkerArray.h>

Eigen::Affine3d getAsAffine(const Eigen::Vector3d &transl, const Eigen::Vector3d &euler) {
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


std::string affine3dToString(const Eigen::Affine3d &T) {
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::digits10);
  Eigen::Vector3d rot = T.rotation().eulerAngles(0,1,2);

  stream << T.translation().transpose() << " " << rot.transpose();
  return stream.str();
}



class NDTCalibScan {
public:
  pcl::PointCloud<pcl::PointXYZ> cloud; // cloud in sensor coords
  Eigen::Affine3d pose;  // estimated pose in global frame (preferably from a GT system)...
  Eigen::Affine3d estSensorPose; // estimated sensor pose in global frame (typically from a SLAM / registration system)
  double stamp;
  
  NDTCalibScan() {
    
  }
 
  NDTCalibScan(const Eigen::Affine3d &_pose, const Eigen::Affine3d &_estSensorPose, double _stamp) : pose(_pose), estSensorPose(_estSensorPose), stamp(_stamp) {
    
  }

  NDTCalibScan(const pcl::PointCloud<pcl::PointXYZ> &_cloud, const Eigen::Affine3d &_pose, double _stamp) : cloud(_cloud), pose(_pose), stamp(_stamp) 
  {

  }
    
  NDTCalibScan(const pcl::PointCloud<pcl::PointXYZ> &_cloud, const Eigen::Affine3d &_pose, const Eigen::Affine3d &_estSensorPose, double _stamp)  : cloud(_cloud), pose(_pose), estSensorPose(_estSensorPose), stamp(_stamp)
  {
    
  }

  void appendPointCloudGlobal(const Eigen::Affine3d &Ts, pcl::PointCloud<pcl::PointXYZ> &p) const {
    
    pcl::PointCloud<pcl::PointXYZ> c =cloud;
    Eigen::Affine3d T = pose * Ts;
    lslgeneric::transformPointCloudInPlace(T, c);
    p += c;
  }

  // Simply the euclidean offset bettwen the estimated sensor pose and the given gt poses
  double scoreEstSensorPose(const Eigen::Affine3d &Ts) const {
    Eigen::Affine3d T = pose * Ts;
    return (T * estSensorPose.inverse()).translation().norm();
  }

    // Return Ts from the global estimate sensor pose and the vehicle pose (both in the same global reference frame).  
  Eigen::Affine3d getTs() const {
    return (pose.inverse() * estSensorPose);
  }
  
  Eigen::Affine3d getSensorPoseFromPose(const Eigen::Affine3d &Ts) const  {
    return pose*Ts;
  }
};

Eigen::VectorXd getTranslationEulerAnglesVector(const Eigen::Affine3d &T) {
  Eigen::VectorXd ret(6);
  ret[0] = T.translation()[0];
  ret[1] = T.translation()[1];
  ret[2] = T.translation()[2];
  Eigen::Vector3d rot = T.rotation().eulerAngles(0,1,2);
  ret[3] = rot[0];
  ret[4] = rot[1];
  ret[5] = rot[2];
  
  return ret;
}


class NDTCalibScanPair {

public:	
  NDTCalibScan first;
  NDTCalibScan second;

  NDTCalibScanPair() {

  }
  NDTCalibScanPair(const pcl::PointCloud<pcl::PointXYZ> &p1, const Eigen::Affine3d &pose1,
                   const pcl::PointCloud<pcl::PointXYZ> &p2, const Eigen::Affine3d &pose2) :
    first(NDTCalibScan(p1, pose1, -1.)), second(NDTCalibScan(p2, pose2, -1.))
  {    

  }

  Eigen::Affine3d getRelativePose() const {
    return this->second.pose * this->first.pose.inverse();
  }

  Eigen::Affine3d getRelativeEstSensorPose() const {
    return this->second.estSensorPose * this->first.estSensorPose.inverse();
  }
  
  // Compute the score for a relative sensor pose offset - that is the pose and the est sensor pose don't have to be in the same coordinate frame.
  double scoreEstSensorPoseRel(const Eigen::Affine3d &Ts) const {
    Eigen::Affine3d rel = getRelativePose();
    Eigen::Affine3d rel_est = getRelativeEstSensorPose();
    return (Ts * rel_est * Ts.inverse() * rel.inverse()).translation().norm();
  }

  // Compute the predicted relative estimated sensor pose.
  Eigen::Affine3d getPredictedRelativeEstSensorPose(const Eigen::Affine3d &Ts) const {
    Eigen::Affine3d rel = getRelativePose();
    return (Ts.inverse() * rel * Ts);
  }

  // Compute difference between the relative estimated and the predicted sensor pose
  Eigen::Affine3d getDifference(const Eigen::Affine3d &Ts) const {
    Eigen::Affine3d T = getPredictedRelativeEstSensorPose(Ts);
    Eigen::Affine3d rel_est = getRelativeEstSensorPose();
    
    return rel_est.inverse()*T;
  }

  // Get the difference in x,y,z and euler angles
  Eigen::VectorXd getDifferenceVector(const Eigen::Affine3d &Ts)  const {
    return getTranslationEulerAnglesVector(getDifference(Ts));
  }
  
  ///Get point cloud in global coords
  void appendPointCloudGlobal(const Eigen::Affine3d &Ts, pcl::PointCloud<pcl::PointXYZ> &p) const {
    
    this->first.appendPointCloudGlobal(Ts, p);
    this->second.appendPointCloudGlobal(Ts, p);
  }

  /**
   * Compute ICP score for sensor offset
   */
  double scoreICP(const Eigen::Affine3d &Ts) const {
    pcl::PointCloud<pcl::PointXYZ> c1=this->first.cloud;
    Eigen::Affine3d p1 = this->first.pose * Ts;
    lslgeneric::transformPointCloudInPlace(p1, c1);
    
    pcl::PointCloud<pcl::PointXYZ> c2=this->second.cloud;
    Eigen::Affine3d p2 = this->second.pose * Ts;
    lslgeneric::transformPointCloudInPlace(p2, c2);
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    typename pcl::KdTree<pcl::PointXYZ>::PointCloudPtr mp (new pcl::PointCloud<pcl::PointXYZ>);
    if(c1.size()==0){
      fprintf(stderr,"Check -> num points = %d\n",(int)c1.size());
    }
    //fprintf(stderr,"1: '%d' -- ",c1.size());
    (*mp) = c1;
    //fprintf(stderr,"2: '%d' == '%d'\n",c1.size(), mp->size());
    kdtree.setInputCloud (mp);
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    double e=0, error_th=0.1*0.1;
    
    for(unsigned int i=0;i<c2.size();i++){
      if ( kdtree.nearestKSearch (c2[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
        if(pointNKNSquaredDistance[0] > error_th){
          e+=error_th;
        }
        else{
          e+= pointNKNSquaredDistance[0];
        }
        //fprintf(f,"%f\n",pointNKNSquaredDistance[0]);
      }
    }//FOR
    
    return e;
  }
};


// Load the evaluation files that are generated by the fuser, <timestamp> x y x qx qy qz qw.
std::vector<Eigen::Affine3d> loadAffineFromEvalFile(const std::string &fileName) {
  std::vector<Eigen::Affine3d> ret;
  std::string line;
  std::ifstream myfile (fileName.c_str());
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      double time, x, y, z, qx, qy, qz, qw;
      std::istringstream ss(line);
      ss >> time >> x >> y >> z >> qx >> qy >> qz >> qw;
      ret.push_back(Eigen::Translation3d(x,y,z)*Eigen::Quaterniond(qw, qx, qy, qz));
    }
    myfile.close();
  }
  else {
    std::cout << "Unable to open file : " << fileName << std::endl;
  } 
  
  return ret;
}

// Load timestamps from the evaluation files that are generated by the fuser...
std::vector<double> loadTimeStampFromEvalFile(const std::string &fileName) {
  std::vector<double> ret;
  std::string line;
  std::ifstream myfile (fileName.c_str());
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      double time, x, y, z, qx, qy, qz, qw;
      std::istringstream ss(line);
      ss >> time >> x >> y >> z >> qx >> qy >> qz >> qw;
      ret.push_back(time);
   }
   myfile.close();
 }
  else std::cout << "Unable to open file : " << fileName << std::endl;; 
  
  return ret;
}


void loadNDTCalibScanPairs(const std::string &gt_file, const std::string &est_sensorpose_file, const std::string &base_name_pcd, std::vector<NDTCalibScanPair> &scans) {
  scans.resize(0);

  // Load the data...
  std::vector<double> stamps = loadTimeStampFromEvalFile(gt_file);
  std::vector<Eigen::Affine3d> Tgt = loadAffineFromEvalFile(gt_file);
  std::vector<Eigen::Affine3d> Test_sensorpose = loadAffineFromEvalFile(est_sensorpose_file);

  if (Tgt.size() != stamps.size() || Tgt.size() != Test_sensorpose.size()) {
    std::cerr << "Warning: the length of the provided files to not match(!)" << std::endl;
  }

  // Need to find a pair of scans which is useful in the optimization scheme. Pair of scans with limited translation but with some rotation...
  size_t j = 0;
  for (size_t i = 1; i < Tgt.size(); i++) {
    Eigen::Affine3d Tmotion = Tgt[j].inverse()*Tgt[i];

    Eigen::Affine3d Ttest = Tgt[j]*Tmotion;
    // while (Tmotion.translation().norm() > 3) {
    //   j++;
    //   Tmotion = Tgt[i] * Tgt[j].inverse();
    // }
    if (Tmotion.translation().norm() > 3) {
      j = i; // reset -> make this better, could be that many potential pairs are left out..., add a while loop instead.
      continue;
    }
    if (Tmotion.rotation().eulerAngles(0,1,2).norm() < 5.0*M_PI/180.0) {
      continue;
    }
    
    // Good pair found...
    NDTCalibScanPair pair;
    pair.first = NDTCalibScan(Tgt[j], Test_sensorpose[j], stamps[j]);
    pair.second = NDTCalibScan(Tgt[i], Test_sensorpose[i], stamps[i]);
    scans.push_back(pair);
    j = i; // reset
  }
  
}



// // Helper functions for ROS
// geometry_msgs::PoseStamped getGtPoseFromCalibScan(const NDTCalibScan &scan) {
//   geometry_msgs::PoseStamped p;
//   tf::poseEigenToMsg (scan.pose, p.pose);
//   p.header.stamp = ros::Time(scan.stamp);
//   p.header.frame_id = std::string("/world");
  
//   return p; 
// }

// geometry_msgs::Pose getEstSensorPoseFromCalibScan(const NDTCalibScan &scan) {
//   geometry_msgs::PoseStamped p;
//   tf::poseEigenToMsg (scan.estSensorPose, p.pose);
//   p.header.stamp = ros::Time(scan.stamp);
//   p.header.frame_id = std::string("/world");
  
//   return p;
// }




namespace ndt_visualisation {


// Visualization markers.
void appendMarkerArray(visualization_msgs::MarkerArray &array, const visualization_msgs::MarkerArray &add) {
  for (size_t i = 0; i < add.markers.size(); i++) {
    array.markers.push_back(add.markers[i]);
  }
}


visualization_msgs::Marker getMarkerArrowAffine3d(const Eigen::Affine3d &T, int id, int color, const std::string &ns) {
  visualization_msgs::Marker m;
  assignDefault(m);
  assignColor(m, color);
  m.ns = ns;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.y = 0.1; m.scale.z = 0.1;
  m.id = id;
  tf::poseEigenToMsg (T, m.pose);
  return m;
} 


visualization_msgs::Marker getMarkerCylinder(const Eigen::Affine3d &T,
                                                  int id, int color,
                                                  double length, double radius,
                                                  const std::string &ns) {
  visualization_msgs::Marker m;
  assignDefault(m);
  assignColor(m, color);
  m.ns = ns;
  m.type = visualization_msgs::Marker::CYLINDER;
  m.action = visualization_msgs::Marker::ADD;
  m.id = id;

  m.scale.x = radius; m.scale.y = radius; m.scale.z = length;
  tf::poseEigenToMsg(T, m.pose);
  return m;
}

/// Draw an x,y,z coordsystem given an affine3d.
visualization_msgs::MarkerArray getMarkerFrameAffine3d(const Eigen::Affine3d &T, const std::string &ns, double length, double radius) {

  visualization_msgs::MarkerArray m;
  // X
  {
    Eigen::Affine3d T_x =
      Eigen::Translation3d(length / 2.0, 0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
    T_x = T * T_x;
    m.markers.push_back(getMarkerCylinder(T_x, 0, 0, length, radius, ns));
  }
  // Y
  {
    Eigen::Affine3d T_y =
      Eigen::Translation3d(0, length / 2.0, 0) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());
    T_y = T * T_y;
    m.markers.push_back(getMarkerCylinder(T_y, 1, 1, length, radius, ns));
  }
  // Z
  {
    Eigen::Affine3d T_z = Eigen::Translation3d(0, 0, length / 2.0) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    T_z = T * T_z;
    m.markers.push_back(getMarkerCylinder(T_z, 2, 2, length, radius, ns));
  }
  return m;
}

visualization_msgs::Marker getMarkerPoseFromNDTCalibScan(const NDTCalibScan &scan, int id, int color) {
  return getMarkerArrowAffine3d(scan.pose, id, color, "ndt_calib_pose");
}

visualization_msgs::Marker getMarkerEstSensorPoseFromNDTCalibScan(const NDTCalibScan &scan, int id, int color) {
  return getMarkerArrowAffine3d(scan.estSensorPose, id, color, "ndt_calib_est_sensor_pose");
}

visualization_msgs::Marker getCorrMarkerPoseToEstSensorPose(const NDTCalibScan &scan, int id, int color) {
 visualization_msgs::Marker m;
 assignDefault(m);
 assignColor(m, color);
 m.type = visualization_msgs::Marker::LINE_STRIP;
 m.action = visualization_msgs::Marker::ADD;
 m.id = id;
 m.ns = "ndt_calib_corr_pose_est_sensor_pose";
 m.scale.x = 0.05;

 geometry_msgs::Point p;
 tf::pointEigenToMsg (scan.pose.translation(), p);
 m.points.push_back(p);
 tf::pointEigenToMsg (scan.estSensorPose.translation(), p);
 m.points.push_back(p);
 return m;
}

visualization_msgs::MarkerArray getMarkerArrayFromNDTCalibScanPairs(const std::vector<NDTCalibScanPair> &pairs) {
  visualization_msgs::MarkerArray m;

  for (size_t i = 0; i < pairs.size(); i++) {
    m.markers.push_back(getMarkerPoseFromNDTCalibScan(pairs[i].first, 2*i, 0));
    m.markers.push_back(getMarkerPoseFromNDTCalibScan(pairs[i].second, 2*i+1, 1));
    m.markers.push_back(getMarkerEstSensorPoseFromNDTCalibScan(pairs[i].first, 2*i, 2));
    m.markers.push_back(getMarkerEstSensorPoseFromNDTCalibScan(pairs[i].second, 2*i+1, 2));
    m.markers.push_back(getCorrMarkerPoseToEstSensorPose(pairs[i].first, i, 0));
  }  
  return m;
}



visualization_msgs::MarkerArray getMarkerArrayRelFromNDTCalibScanPair(const NDTCalibScanPair &pair, const Eigen::Affine3d &Ts) {
  visualization_msgs::MarkerArray m;

  // Draw the coordinate system of the relative frames.
  appendMarkerArray(m, getMarkerFrameAffine3d(pair.getRelativePose(), 
                                               std::string("ndt_calib_rel_pose"), 1.0, 0.1));
  appendMarkerArray(m, getMarkerFrameAffine3d(pair.getRelativeEstSensorPose(), 
                                              std::string("ndt_calib_rel_est_sensor_pose"), 1.0, 0.1));
  appendMarkerArray(m, getMarkerFrameAffine3d(pair.getPredictedRelativeEstSensorPose(Ts),
                                              std::string("ndt_calib_pred_rel_est_sensor_pose"), 1.0, 0.1));



  appendMarkerArray(m, getMarkerFrameAffine3d(pair.first.getSensorPoseFromPose(Ts), std::string("ndt_calib_sensor_pose_from_first_pose"), 1., 0.1));
  appendMarkerArray(m, getMarkerFrameAffine3d(pair.first.getSensorPoseFromPose(Ts), std::string("ndt_calib_sensor_pose_from_second_pose"), 1., 0.1));
  

  
  return m;
}


} // namespace ndt_visualisation




