#pragma once

#include <ndt_calibration/ndt_calib.h>
#include <ndt_rviz/ndt_rviz.h>
#include <visualization_msgs/MarkerArray.h>

namespace ndt_visualisation {


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



  appendMarkerArray(m, getMarkerFrameAffine3d(pair.first.pose, std::string("ndt_calib_pose_from_first_pose"), 2., 0.2));
  appendMarkerArray(m, getMarkerFrameAffine3d(pair.first.getSensorPoseFromPose(Ts), std::string("ndt_calib_sensor_pose_from_first_pose"), 1., 0.1));
  appendMarkerArray(m, getMarkerFrameAffine3d(pair.second.getSensorPoseFromPose(Ts), std::string("ndt_calib_sensor_pose_from_second_pose"), 1., 0.1));
  

  
  return m;
}


} // namespace ndt_visualisation


