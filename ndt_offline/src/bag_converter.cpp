#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/message_filter.h>
#include <boost/program_options.hpp>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <boost/circular_buffer.hpp>
#include <ndt_offline/convertvelodynebags.h>
#include <eigen_conversions/eigen_msg.h>
#include <angles/angles.h>
#include <velodyne_msgs/VelodyneScan.h>

using namespace std;
namespace po = boost::program_options;
bool create_odom=true,create_tf=true;
// This will underestimate the yaw.
double getYawFromXY(const geometry_msgs::Pose &prev,
                    const geometry_msgs::Pose &curr) {
  double dx = curr.position.x - prev.position.x;
  double dy = curr.position.y - prev.position.y;
  return atan2(dy,dx);
}

double getDistBetween(const geometry_msgs::Pose &p1,
                      const geometry_msgs::Pose &p2) {
  double dx = p1.position.x - p2.position.x;
  double dy = p1.position.y - p2.position.y;
  return sqrt(dx*dx+dy*dy);
}

double getYawUsingPoses(const boost::circular_buffer<geometry_msgs::Pose> &poses,
                        const geometry_msgs::Pose &pose,
                        double min_dist,
                        bool &valid_yaw) {
  double min_val =  std::numeric_limits<double>::max();
  int min_idx = -1;
  for (int i = 0; i < poses.size(); i++) {
    double val = getDistBetween(poses[i], pose);
    if (val > min_dist) {
      if (val < min_val) {
        min_val = val;
        min_idx = i;
      }
    }
  }

  //  std::cout << "[" << min_val << "]" << std::flush;

  if (min_idx < 0) {
    valid_yaw = false;
    return 0.;
  }

  valid_yaw = true;
  return getYawFromXY(poses[min_idx], pose);
}

geometry_msgs::Pose getPose(double x, double y, double yaw) {
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;

  tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::Transform getTransform(double x, double y, double yaw) {
  geometry_msgs::Transform t;
  t.translation.x = x;
  t.translation.y = y;
  t.translation.z = 0.;

  tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();
  t.rotation.w = q.w();
  return t;
}

// Return euler angles - roll, pitch, yaw...
void get_euler_angles(Eigen::Vector3d &euler, const geometry_msgs::PoseStamped &p) {

  // Set yaw from the current odometry... the current EKF pose utilize the GPS heading for the yaw, the roll and pitch are directly taken from the IMU.
  Eigen::Quaterniond q;
  tf::quaternionMsgToEigen(p.pose.orientation, q);
  // Need a rotation matrix
  Eigen::Matrix3d rmat = q.matrix();
  euler = rmat.eulerAngles(0, 1, 2);

  if (fabs(euler[0]) > M_PI/2) {
    euler[0] += M_PI;
    euler[1] += M_PI;
    euler[2] += M_PI;

    euler[0] = angles::normalize_angle(euler[0]);
    euler[1] = angles::normalize_angle(euler[1]);
    euler[2] = angles::normalize_angle(euler[2]);
  }
}
void GetSensorPose(const std::string &dataset,  Eigen::Vector3d & transl,  Eigen::Vector3d &euler,tf::Transform &tf_sensor){

  tf::Quaternion quat;

  bool found_sensor_pose=false;
  if(dataset.compare("oru-basement")==0){
    transl[0]=0.3;
    transl[1]=0;
    transl[2]=0;
    euler[0]=0;
    euler[1]=0;
    euler[2]=-1.62;
    found_sensor_pose=true;
  }
  else if(dataset.compare("default")==0){
    transl[0]=0;
    transl[1]=0;
    transl[2]=0;
    euler[0]=0;
    euler[1]=0;
    euler[2]=0;
    found_sensor_pose=true;
  }
  else if(dataset.compare("arla-2012")==0){
    transl[0]=1.18;
    transl[1]=-0.3;
    transl[2]=2.0;
    euler[0]=0;
    euler[1]=0;
    euler[2]=-1.62;
    found_sensor_pose=true;
  }
  quat.setRPY(euler[0], euler[1], euler[2]);
  tf::Vector3 trans(transl[0], transl[1], transl[2]);
  tf_sensor  = tf::Transform(quat,trans);

  if(found_sensor_pose)
    cout<<"Sensor settings for dataset : \""<<dataset<<"\" is used"<<endl;
  cout<<"Sensor offset (x,y,z)=("<<transl(0)<<","<<transl(1)<<","<<transl(2)<<")"<<endl<<"Sensor angles (r,p,y)=("<<euler(0)<<","<<euler(1)<<","<<euler(2)<<")"<<endl;
  cout<<"Are these the correct settings? y/n"<<endl;
  char c=getchar();
  getchar();
  if(!( c=='y'||c=='Y'))
    exit(0);
}

void update_roll_pitch(Eigen::Affine3d &t, Eigen::Vector3d euler, double yaw) {

  // Set yaw from the current odometry... the current EKF pose utilize the GPS heading for the yaw, the roll and pitch are directly taken from the IMU.
  t.linear() = (Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).toRotationMatrix();
}

// This is targeted to create imu data messages from odometry / encoder readings...
int main(int argc, char **argv){

  ros::Time::init();
  Eigen::Vector3d transl;
  Eigen::Vector3d euler;
  tf::Transform tf_sensor_pose;
  string inbag_name, outbag_name, dataset;
  string velodyne_config_file,velodyne_packets_topic,velodyne_frame_id,base_link_id,gt_base_link_id,tf_topic,tf_world_frame;
  double min_dist;
  double min_range=0.6,max_range=30;
  Eigen::Vector3d imu_offset_vec;;
  po::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("debug", "additional output")
      ("inbag", po::value<string>(&inbag_name)->required(), "bag file to be used")
      ("velodyne_config_file", po::value<std::string>(&velodyne_config_file)->default_value(std::string("../config/velo32.yaml")), "configuration file for the scanner")
      ("velodyne_packets_topic", po::value<std::string>(&velodyne_packets_topic)->default_value(std::string("/velodyne_packets")), "velodyne packets topic used")
      ("velodyne_frame_id", po::value<std::string>(&velodyne_frame_id)->default_value(std::string("/velodyne")), "frame_id of the velodyne")
      ("tf_base_link", po::value<std::string>(&base_link_id)->default_value(std::string("/odom_base_link")), "tf_base_link")
      ("tf_gt_link", po::value<std::string>(&gt_base_link_id)->default_value(std::string("/state_base_link")), "tf ground truth link")
      ("tf_world_frame", po::value<std::string>(&tf_world_frame)->default_value(std::string("/world")), "tf world frame")
      ("tf_topic", po::value<std::string>(&tf_topic)->default_value(std::string("/tf")), "tf topic to listen to")
      ("min_range", po::value<double>(&min_range)->default_value(0.7), "minimum range used from scanner")
      ("max_range", po::value<double>(&max_range)->default_value(30), "minimum range used from scanner")
      ("x", po::value<double>(&transl[0])->default_value(0.), "sensor pose - translation vector x")
      ("y", po::value<double>(&transl[1])->default_value(0.), "sensor pose - translation vector y")
      ("z", po::value<double>(&transl[2])->default_value(0.), "sensor pose - translation vector z")
      ("ex", po::value<double>(&euler[0])->default_value(0.), "sensor pose - euler angle vector x")
      ("ey", po::value<double>(&euler[1])->default_value(0.), "sensor pose - euler angle vector y")
      ("ez", po::value<double>(&euler[2])->default_value(0.), "sensor pose - euler angle vector z")
      ("data-set", po::value<string>(&dataset)->default_value(std::string("oru-basement")), "choose which dataset that is currently used, this option will assist with assigning the sensor pose")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help")) {
    cout << desc << "\n";
    return 1;
  }
  cout<<"Point cloud will be filtered according to:\n min="<<min_range<<" < range < max="<<max_range<<endl<<"Continue? y/n";
  char c=getchar();
  getchar();
  if(!( c=='y'||c=='Y')){
    cout << desc << "\n";
    exit(0);
  }

  po::notify(vm);

  rosbag::Bag outbag;
  //bag.open(inbag_name, rosbag::bagmode::Read);
  outbag_name = inbag_name + std::string("_edited.bag");
  GetSensorPose(dataset,transl,euler,tf_sensor_pose);

  tf::StampedTransform sensor_link;
  sensor_link.child_frame_id_ = velodyne_frame_id;
  sensor_link.frame_id_ = gt_base_link_id;//base_link_id;//tf_base_link; //"/odom_base_link";
  sensor_link.setData(tf_sensor_pose);
  //vmc_navserver::VMCEncodersStamped prev_enc_msg;
  ConvertVelodyneBagsToPcl reader(outbag_name,
                                                 velodyne_config_file,
                                                 inbag_name,
                                                 velodyne_packets_topic,  //"/velodyne_packets"
                                                 velodyne_frame_id,
                                                 tf_world_frame,
                                                 tf_topic,
                                                 ros::Duration(3600),
                                                 &sensor_link, max_range, min_range,
                                                 0);


  pcl::PointCloud<pcl::PointXYZ> cloud;
  tf::Transform tf_scan_source;
  tf::Transform tf_gt_base;
  Eigen::Affine3d Todom_base_prev,Tgt_base_prev;
  while(reader.ConvertToPclBag(tf_scan_source));


 // while(reader.readMultipleMeasurements(1,cloud,tf_scan_source,tf_gt_base,base_link_id)){



  //}
  reader.CloseOutputBag();

  // -----------------------------------------------------------------------------



  /*if (m.getTopic() == std::string(topic_prefix + "/vmc_navserver/odom")) {
      nav_msgs::Odometry::ConstPtr msg = m.instantiate<nav_msgs::Odometry>();
      if (msg != NULL) {
        //    get_euler_angles(odom_euler, msg->pose.pose);

      }
  }*/
  //outbag.write(m.getTopic(), m.getTime(), m);

  return 0;
}
