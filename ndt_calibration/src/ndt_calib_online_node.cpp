#include <ros/ros.h>
#include <ndt_generic/utils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>

#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>

#include <boost/foreach.hpp>

#include <ndt_offline/PoseInterpolationNavMsgsOdo.h>
#include <ndt_calibration/ndt_calib.h>


//! Simple interface class implementation to be compatible with the NDTCalib API.
class PoseInterpolationTF : public PoseInterpolationInteface {

public:
  PoseInterpolationTF(const std::string &fixedframe,tf::Transformer &transformer) : fixedframe_(fixedframe), transformer_(transformer) {

  }

  bool getTransformationForTime(ros::Time t0, const std::string &frame_id,Eigen::Affine3d &T){
    std::string str;
    if (!transformer_.canTransform(fixedframe_, frame_id, t0, &str)) {
      return false;
    }
    tf::StampedTransform transform;
    transformer_.lookupTransform(fixedframe_,frame_id,t0, transform);
    tf::transformTFToEigen (transform, T);
    return true;
  }

private:
  tf::Transformer &transformer_;
  std::string fixedframe_;
};


class NDTCalibOnlineNode {

protected:
  // Our NodeHandle
  ros::NodeHandle nh_;

  ros::Subscriber points2_sub_;
  ros::Subscriber laser_sub_;

  // Components for publishing
  tf::TransformListener tf_listener_; // Keep the duration very long for this...
  tf::Transformer tr_transformer_;

  Eigen::Affine3d pose_, T, Ts;
  double sensor_pose_x,sensor_pose_y,sensor_pose_z,
  sensor_pose_r,sensor_pose_p,sensor_pose_t;
  double sensor_offset_t_;

  laser_geometry::LaserProjection projector_;

  ros::Publisher map_publisher_,laser_publisher_,point2_publisher_,odom_publisher_,adjusted_odom_publisher_,fuser_odom_publisher_;
  nav_msgs::Odometry fuser_odom,adjusted_odom_msg;
  Eigen::Affine3d last_odom, this_odom,last_gt_pose;

  std::string points_topic;
  std::string laser_topic;
  bool visualize;

  std::string pose_frame_;
  tf::Transform tf_sensor_pose_;
  double min_laser_range_;
  double varz_;
  double resolution_;
  double max_translation_;
  double min_rotation_;

  NDTCalibScanPair scan_pair_;
  NDTCalibScanPairs pairs_;
  PoseInterpolationTF pose_interp_;
  bool initialized;

  NDTCalibOptimize::ObjectiveType objective_type_;
  bool calib_;
  int score_type_;

  int min_nb_calibration_pairs_;
  int max_nb_calibration_pairs_;

public:
  // Constructor
  NDTCalibOnlineNode(ros::NodeHandle param_nh) : tf_listener_(ros::Duration(3600)), pose_interp_("/world", tf_listener_)
  {
    ///topic to wait for point clouds, if available
    param_nh.param<std::string>("points_topic",points_topic,"points");
    ///topic to wait for laser scan messages, if available
    param_nh.param<std::string>("laser_topic",laser_topic,"laser_scan");

    param_nh.param("visualize",visualize,true);

    ///initial pose and timing offset of the sensor w.r.t. the vehicle frame
    param_nh.param("sensor_pose_x",sensor_pose_x,0.);
    param_nh.param("sensor_pose_y",sensor_pose_y,0.);
    param_nh.param("sensor_pose_z",sensor_pose_z,0.);
    param_nh.param("sensor_pose_r",sensor_pose_r,0.);
    param_nh.param("sensor_pose_p",sensor_pose_p,0.);
    param_nh.param("sensor_pose_t",sensor_pose_t,0.);
    param_nh.param("sensor_offset_t",sensor_offset_t_,0.);

    // what to optimize
    bool cx, cy, cz, cex, cey, cez, ct;
    param_nh.param("cx", cx, true);
    param_nh.param("cy", cy, true);
    param_nh.param("cz", cz, false);
    param_nh.param("cex", cex, false);
    param_nh.param("cey", cey, false);
    param_nh.param("cez", cez, true);
    param_nh.param("ct", ct, true);
    this->objective_type_.setObjective(cx,cy,cz,cex,cey,cez,ct);

    param_nh.param<std::string>("pose_frame",pose_frame_,"/state_base_link");

    param_nh.param<double>("min_laser_range", min_laser_range_, 1.);
    param_nh.param<double>("resolution", resolution_, 0.6);
    param_nh.param<double>("laser_variance_z",varz_,resolution_/4);

    param_nh.param<double>("max_translation", max_translation_, 2);
    param_nh.param<double>("min_rotation",min_rotation_, 0.1);

    param_nh.param("calib", calib_, true);

    param_nh.param<int>("score_type", score_type_, 3);
    param_nh.param<int>("min_nb_calibration_pairs", min_nb_calibration_pairs_, 5);
    param_nh.param<int>("max_nb_calibration_pairs", max_nb_calibration_pairs_, 50);


    Ts =  Eigen::Translation<double,3>(sensor_pose_x,sensor_pose_y,sensor_pose_z)*
        Eigen::AngleAxis<double>(sensor_pose_r,Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxis<double>(sensor_pose_p,Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxis<double>(sensor_pose_t,Eigen::Vector3d::UnitZ()) ;

    tf::poseEigenToTF(Ts,tf_sensor_pose_);
  initialized = false;

   laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(laser_topic, 1, &NDTCalibOnlineNode::laserCallback, this);
   points2_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(points_topic, 1, &NDTCalibOnlineNode::points2Callback, this);
  }

  // The T is given in global vehicle coordinates (not any relative ones as commonly done in the registration / fuser approaches.
  void processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Affine3d T, ros::Time stamp) {

        if (!initialized) {
        scan_pair_.first = NDTCalibScan(cloud, T, cloud.header.stamp);
        initialized = true;
          return;
    }

    Eigen::Affine3d Tmotion = scan_pair_.first.getOriginalPose().inverse()*T;
    // We have gone to far without any "rotation", simply reset the system.
    if (Tmotion.translation().norm() > max_translation_) {
      scan_pair_.first = NDTCalibScan(cloud, T, cloud.header.stamp);
    }
    Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);
    normalizeEulerAngles(rot);
    if (rot.norm() < min_rotation_) {
      return;
    }

    // Good pair found...
    scan_pair_.second = NDTCalibScan(cloud, T, stamp.toSec());

    pairs_.push_back(scan_pair_);

    if (pairs_.size() > min_nb_calibration_pairs_) {
      if (pairs_.size() > max_nb_calibration_pairs_) {
        pairs_.erase(pairs_.begin(), pairs_.begin()+pairs_.size() - max_nb_calibration_pairs_);
      }
      this->calibrate();
    }

    // Reset the first one...
    scan_pair_.first = scan_pair_.second;

    ROS_INFO_STREAM("# pairs : " << pairs_.size());
  }

  // Callback
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_unfiltered, pcl_cloud;
    // TODO, check if the projector class could be used for individual interpolation of laser beams.
    projector_.projectLaser(*msg, cloud);
    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

    pcl::PointXYZ pt;
    //add some variance on z
    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
      pt = pcl_cloud_unfiltered.points[i];
      if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_laser_range_) {
        pt.z += varz_*((double)rand())/(double)INT_MAX;
        pcl_cloud.points.push_back(pt);
      }
    }

    Eigen::Affine3d T;
    ros::Time t = msg->header.stamp-ros::Duration(sensor_offset_t_);
    ROS_INFO_STREAM("L @ time:" << t);
    if (pose_interp_.getTransformationForTime(t, pose_frame_, T)) {
      this->processFrame(pcl_cloud,T, t);
    }

  }

  void points2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg){


    Eigen::Affine3d Tm;
    static bool last_odom_found=false;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*msg, cloud);

    Eigen::Affine3d T;
    ros::Time t = msg->header.stamp-ros::Duration(sensor_offset_t_);
    if (pose_interp_.getTransformationForTime(t, pose_frame_, T)) {
      this->processFrame(cloud,T, msg->header.stamp-ros::Duration(sensor_offset_t_));
    }
}

  void calibrate() {
    if (pairs_.empty()) {
      ROS_ERROR_STREAM("No calibration scan pairs");
      return;
    }
    ROS_INFO_STREAM("got : " << pairs_.size() << " scan pairs");
    ROS_INFO_STREAM("Computing NDTMap...");
    pairs_.computeNDTMap(resolution_);
    ROS_INFO_STREAM("done.");

    NDTCalibOptimize opt(pairs_, score_type_, objective_type_, pose_interp_, pose_frame_);
    ROS_INFO_STREAM("Optimize time : " << objective_type_.optimizeTime());

    double delta_sensor_time_offset = 0.;
    opt.interpPairPoses(delta_sensor_time_offset); // We already moved the interpolation points while adding the clouds to the pairs, the sensortime reported will be the provided on added with the output.

    ROS_INFO_STREAM("initial sensor pose est : " << affine3dToString(Ts) << " dt : " << sensor_offset_t_+delta_sensor_time_offset << " error : " << opt.getScore6d(Ts));
    if (calib_) {
      ROS_INFO_STREAM("starting calibration, using score type : " << score_type_);
      opt.calibrate(Ts, delta_sensor_time_offset);
      opt.interpPairPoses(delta_sensor_time_offset);
      std::cout << " new sensor pose est : " << affine3dToString(Ts) << " dt : " << sensor_offset_t_+delta_sensor_time_offset << " error : " << opt.getScore6d(Ts) << std::endl;
    }
  }


public:
  // map publishing function
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndt_calib_online_node");
  ros::NodeHandle param("~");
  NDTCalibOnlineNode n(param);
  ros::spin();

  return 0;
}

