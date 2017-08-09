#include <ndt_fuser/ndt_fuser_hmt.h>
#include <ndt_offline/VelodyneBagReader.h>
#include <ndt_generic/eigen_utils.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/pointcloud_utils.h>
#include <tf_conversions/tf_eigen.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <algorithm>
#include <boost/program_options.hpp>
#include "graph_map_fuser.h"
#include "ndt/ndt_map_param.h"
#include "ndt/ndtd2d_reg_type.h"
#include "ndt/ndt_map_type.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ros/publisher.h"
#include "tf/transform_broadcaster.h"
#include "ndt_generic/eigen_utils.h"
//#include "ndt_mcl/3d_ndt_mcl.h"
#include "ndt_generic/io.h"
using namespace libgraphMap;
namespace po = boost::program_options;
using namespace std;
using namespace lslgeneric;

std::string dirname="";
std::string output_dir_name="";
std::string base_name="";
std::string dataset="";
//map parameters
int itrs=0;
int nb_neighbours=0;
int nb_scan_msgs=0;
bool use_odometry=true;
bool visualize=true;
bool filter_fov=false;
bool step_control=false;
bool registration2d=true;
bool alive=false;
bool disable_reg=false, do_soft_constraints=false;
lslgeneric::MotionModel2d::Params motion_params;
std::string base_link_id="", gt_base_link_id="", tf_world_frame="";
std::string velodyne_config_file="";
std::string velodyne_packets_topic="";
std::string velodyne_frame_id="";
std::string map_file_name="";
std::string tf_topic="";
tf::Transform tf_sensor_pose;
Eigen::Affine3d sensor_offset,fuser_pose;//Mapping from base frame to sensor frame
ros::NodeHandle *n_=NULL;
MapParamPtr mapParPtr=NULL;
GraphParamPtr graphParPtr=NULL;
double sensor_time_offset=0;
double resolution_local_factor=0;
double max_range=0, min_range=0;
double maxRotationNorm_=0;
double interchange_radius_=0;
double maxTranslationNorm_=0;
double rotationRegistrationDelta_=0;
double sensorRange_=30;
double translationRegistrationDelta_=0;
double resolution=0;
double hori_min=0, hori_max=0;
double min_dist=0, min_rot_in_deg=0;
ros::Publisher *gt_pub,*fuser_pub,*cloud_pub;
nav_msgs::Odometry gt_pose_msg,fuser_pose_msg;
pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cloud;



template<class T> std::string toString (const T& x)
{
  std::ostringstream o;

  if (!(o << x))
    throw std::runtime_error ("::toString()");

  return o.str ();
}

void filter_fov_fun(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &cloud_nofilter, double hori_min, double hori_max) {
  for(int i=0; i<cloud_nofilter.points.size(); ++i) {
    double ang = atan2(cloud_nofilter.points[i].y, cloud_nofilter.points[i].x);
    if(ang < hori_min || ang > hori_max) continue;
    cloud.points.push_back(cloud_nofilter.points[i]);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  std::cout << "nb clouds : " << cloud.points.size() << std::endl;
}


std::string transformToEvalString(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T) {
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::digits10);
  Eigen::Quaternion<double> tmp(T.rotation());
  stream << T.translation().transpose() << " " << tmp.x() << " " << tmp.y() << " " << tmp.z() << " " << tmp.w() << std::endl;
  return stream.str();
}
bool GetSensorPose(const std::string &dataset,  Eigen::Vector3d & transl,  Eigen::Vector3d &euler,tf::Transform &tf_sensor){

  tf::Quaternion quat;
  bool found_sensor_pose=false;
  if(dataset.compare("oru-basement")){
    transl[0]=0.3;
    transl[1]=0;
    transl[2]=0;
    euler[0]=0;
    euler[1]=0;
    euler[2]=-1.62;
    found_sensor_pose=true;
  }
  else if(dataset.compare("default")){
    transl[0]=0.3;
    transl[1]=0;
    transl[2]=0;
    euler[0]=0;
    euler[1]=0;
    euler[2]=-1.62;
    found_sensor_pose=true;
  }
  quat.setRPY(euler[0], euler[1], euler[2]);
  tf::Vector3 trans(transl[0], transl[1], transl[2]);
  tf_sensor  = tf::Transform(quat,trans);
  tf::poseTFToEigen(tf_sensor,sensor_offset);
  return found_sensor_pose;
}



bool LocateRosBagFilePaths(const std::string &folder_name,std::vector<std::string> &scanfiles){
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (folder_name.c_str())) != NULL) {
    while ((ent = readdir (dir)) != NULL) {
      if(ent->d_name[0] == '.') continue;
      char tmpcname[400];
      snprintf(tmpcname,399,"%s/%s",folder_name.c_str(),ent->d_name);
      std::string tmpfname = tmpcname;
      scanfiles.push_back(tmpfname);
    }
    closedir (dir);
  } else {
    std::cerr<<"Could not parse dir name\n";
    return false;
  }
  sort(scanfiles.begin(),scanfiles.end());
  {
    std::cout << "files to be loaded : " << std::endl;
    for (size_t i = 0; i < scanfiles.size(); i++) {
      std::cout << " " << scanfiles[i] << std::flush;
    }
    std::cout << std::endl;
  }
  return true;
}
bool ReadAllParameters(po::options_description &desc,int &argc, char ***argv){

  Eigen::Vector3d transl;
  Eigen::Vector3d euler;
  // First of all, make sure to advertise all program options
  desc.add_options()
      ("help", "produce help message")
      ("map-file-path", po::value<std::string>(&map_file_name)->default_value(std::string("graph_map.MAP")), "name of file to load containing graphMap")
      ("visualize", "visualize the output")
      ("base-name", po::value<string>(&base_name)->default_value(std::string("offline")), "prefix for all generated files")
      ("output-dir-name", po::value<string>(&output_dir_name)->default_value("/home/daniel/.ros/maps"), "where to save the pieces of the map (default it ./map)")
      ("data-set", po::value<string>(&dataset)->default_value(""), "where to save the pieces of the map (default it ./map)")
      ("filter-fov", "cutoff part of the field of view")
      ("dir-name", po::value<string>(&dirname), "where to look for ros bags")
      ("hori-max", po::value<double>(&hori_max)->default_value(2*M_PI), "the maximum field of view angle horizontal")
      ("hori-min", po::value<double>(&hori_min)->default_value(-hori_max), "the minimum field of view angle horizontal")
      ("Dd", po::value<double>(&motion_params.Dd)->default_value(1.), "forward uncertainty on distance traveled")
      ("Dt", po::value<double>(&motion_params.Dt)->default_value(1.), "forward uncertainty on rotation")
      ("Cd", po::value<double>(&motion_params.Cd)->default_value(1.), "side uncertainty on distance traveled")
      ("Ct", po::value<double>(&motion_params.Ct)->default_value(1.), "side uncertainty on rotation")
      ("Td", po::value<double>(&motion_params.Td)->default_value(1.), "rotation uncertainty on distance traveled")
      ("Tt", po::value<double>(&motion_params.Tt)->default_value(1.), "rotation uncertainty on rotation")
      ("min_dist", po::value<double>(&min_dist)->default_value(0.2), "minimum distance traveled before adding cloud")
      ("min_rot_in_deg", po::value<double>(&min_rot_in_deg)->default_value(5), "minimum rotation before adding cloud")
      ("tf_base_link", po::value<std::string>(&base_link_id)->default_value(std::string("/odom_base_link")), "tf_base_link")
      ("tf_gt_link", po::value<std::string>(&gt_base_link_id)->default_value(std::string("/state_base_link")), "tf ground truth link")
      ("velodyne_config_file", po::value<std::string>(&velodyne_config_file)->default_value(std::string("../config/velo32.yaml")), "configuration file for the scanner")
      ("tf_world_frame", po::value<std::string>(&tf_world_frame)->default_value(std::string("/world")), "tf world frame")
      ("velodyne_packets_topic", po::value<std::string>(&velodyne_packets_topic)->default_value(std::string("/velodyne_packets")), "velodyne packets topic used")
      ("velodyne_frame_id", po::value<std::string>(&velodyne_frame_id)->default_value(std::string("/velodyne")), "frame_id of the velodyne")
      ("min_range", po::value<double>(&min_range)->default_value(0.6), "minimum range used from scanner")
      ("max_range", po::value<double>(&max_range)->default_value(30), "minimum range used from scanner")
      ("save-map", "saves the graph map at the end of execution")
      ("nb_scan_msgs", po::value<int>(&nb_scan_msgs)->default_value(1), "number of scan messages that should be loaded at once from the bag")
      ("tf_topic", po::value<std::string>(&tf_topic)->default_value(std::string("/tf")), "tf topic to listen to")
      ("x", po::value<double>(&transl[0])->default_value(0.), "sensor pose - translation vector x")
      ("y", po::value<double>(&transl[1])->default_value(0.), "sensor pose - translation vector y")
      ("z", po::value<double>(&transl[2])->default_value(0.), "sensor pose - translation vector z")
      ("ex", po::value<double>(&euler[0])->default_value(0.), "sensor pose - euler angle vector x")
      ("ey", po::value<double>(&euler[1])->default_value(0.), "sensor pose - euler angle vector y")
      ("ez", po::value<double>(&euler[2])->default_value(0.), "sensor pose - euler angle vector z")
      ("sensor_time_offset", po::value<double>(&sensor_time_offset)->default_value(0.), "timeoffset of the scanner data")
      ("registration2d","registration2d")
      ("resolution", po::value<double>(&resolution)->default_value(0.4), "resolution of the map")
      ("resolution_local_factor", po::value<double>(&resolution_local_factor)->default_value(1.), "resolution factor of the local map used in the match and fusing step")
      ("use-submap", "Adopt the sub-mapping technique which represent the global map as a set of local submaps");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, *argv, desc), vm);
  po::notify(vm);
  cout<<"sensor pose"<<endl;
  if(!GetSensorPose(dataset,transl,euler,tf_sensor_pose))
    exit(0);

  visualize = vm.count("visualize");
  filter_fov = vm.count("filter-fov");
  step_control = (vm.count("no-step-control") == 0);
  alive = vm.count("alive");
  //Check if all iputs are assigned
  if (!vm.count("base-name") || !vm.count("dir-name")){
    cout << "Missing base or dir names.\n";
    cout << desc << "\n";
    return false;
  }
  if (vm.count("help")){
    cout << desc << "\n";
    return false;
  }
  cout<<"base-name:"<<base_name<<endl;
  cout<<"dir-name:"<<dirname<<endl;
  return true;


}
void initializeRosPublishers(){
  gt_pub=new ros::Publisher();
  fuser_pub=new ros::Publisher();
  cloud_pub=new ros::Publisher();
  *gt_pub    =n_->advertise<nav_msgs::Odometry>("/GT", 50);
  *fuser_pub =n_->advertise<nav_msgs::Odometry>("/fuser", 50);
  *cloud_pub = n_->advertise<pcl::PointCloud<pcl::PointXYZ>>("/points2", 1);
}
void printParameters(){
  cout<<"Output directory: "<<output_dir_name<<endl;
  if(filter_fov)
    cout << "Filtering FOV of sensor to min/max "<<hori_min<<" "<<hori_max<<endl;
  else
    cout<<"No FOV filter."<<endl;

}


/////////////////////////////////////////////////////////////////////////////////7
/////////////////////////////////////////////////////////////////////////////////7
/// *!!MAIN!!*
/////////////////////////////////////////////////////////////////////////////////7
/////////////////////////////////////////////////////////////////////////////////7
///

int main(int argc, char **argv){
  cout<<"start"<<endl;
  ros::init(argc, argv, "graph_fuser3d_offline");
  cout<<"po options"<<endl;
  po::options_description desc("Allowed options");
  cout<<"node handle"<<endl;
  n_=new ros::NodeHandle("~");
  cout<<"test"<<endl;


  bool succesfull=ReadAllParameters(desc,argc,&argv);
  if(!succesfull)
    exit(0);
  cout<<"load map from file"<<endl;
  GraphMapNavigatorPtr graph_map;
  //LoadGraphMap(map_file_name,graphMap);
  std::ifstream ifs(map_file_name);
  boost::archive::text_iarchive ia(ifs);
  ia >> graph_map;
  cout<<"map succesfully loaded"<<endl;
  cout<<graph_map->ToString()<<endl;
  ndt_generic::CreateEvalFiles eval_files(output_dir_name,base_name);
  printParameters();
  initializeRosPublishers();
  tf::TransformBroadcaster br;
  gt_pose_msg.header.frame_id="/world";
  fuser_pose_msg.header.frame_id="/world";

  base_name += motion_params.getDescString() + std::string("_res") + toString(resolution) + std::string("_SC") + toString(do_soft_constraints) + std::string("_mindist") + toString(min_dist) + std::string("_sensorcutoff") + toString(max_range) + std::string("_stepcontrol") + toString(step_control) + std::string("_neighbours") + toString(nb_neighbours) + std::string("_rlf") + toString(resolution_local_factor);

  ros::Time::init();
  srand(time(NULL));

  /// Set up the sensor link
  tf::StampedTransform sensor_link; ///Link from /odom_base_link -> velodyne
  sensor_link.child_frame_id_ = velodyne_frame_id;
  sensor_link.frame_id_ = base_link_id;//tf_base_link; //"/odom_base_link";
  sensor_link.setData(tf_sensor_pose);

  std::vector<std::string> ros_bag_paths;
  if(!LocateRosBagFilePaths(dirname,ros_bag_paths)){
    cout<<"couldnt locate ros bags"<<endl;
    exit(0);
  }

  int counter = 0;
  if(!eval_files.CreateOutputFiles()){
    cout<<"couldnt create output files"<<endl;
    exit(0);
  }

  cout<<"opening bag files"<<endl;
  for(int i=0; i<ros_bag_paths.size(); i++) {
    std::string bagfilename = ros_bag_paths[i];
    fprintf(stderr,"Opening %s\n",bagfilename.c_str());
    cout<<velodyne_config_file<<","<<bagfilename<<","<<velodyne_packets_topic<<","<<velodyne_frame_id<<","<<tf_world_frame<<","<<tf_topic<<endl;
    VelodyneBagReader<pcl::PointXYZ> vreader(velodyne_config_file,
                                             bagfilename,
                                             velodyne_packets_topic,  //"/velodyne_packets"
                                             velodyne_frame_id,
                                             tf_world_frame,
                                             tf_topic,
                                             ros::Duration(3600),
                                             &sensor_link, max_range, min_range,
                                             sensor_time_offset);

    pcl::PointCloud<pcl::PointXYZ> cloud, cloud_nofilter;
    tf::Transform tf_scan_source;
    tf::Transform tf_gt_base;
    Eigen::Affine3d Todom_base_prev,Tgt_base_prev;

    while(vreader.readMultipleMeasurements(nb_scan_msgs,cloud_nofilter,tf_scan_source,tf_gt_base,base_link_id)){
      if(!n_->ok())
        exit(0);

      if(cloud_nofilter.size()==0) continue;

      if(filter_fov) {
        filter_fov_fun(cloud,cloud_nofilter,hori_min,hori_max);
      } else {
        cloud = cloud_nofilter;
      }

      if (cloud.size() == 0) continue; // Check that we have something to work with depending on the FOV filter here...

      tf::Transform tf_odom_base;
      vreader.getPoseFor(tf_odom_base, base_link_id);
      vreader.getPoseFor(tf_gt_base, gt_base_link_id);
      Eigen::Affine3d Todom_base,Tgt_base;
      tf::transformTFToEigen(tf_gt_base,Tgt_base);
      tf::transformTFToEigen(tf_odom_base,Todom_base);

      if(counter == 0){
        counter ++;
        cloud.clear();
        cloud_nofilter.clear();
        continue;
      }
      if(counter == 1){
        fuser_pose=Tgt_base;
        Tgt_base_prev = Tgt_base;
        Todom_base_prev = Todom_base;
        // load graph fuser_=new GraphMapFuser(regParPtr,mapParPtr,graphParPtr,Tgt_base,sensor_offset);
        counter ++;
        cloud.clear();
        cloud_nofilter.clear();
        continue;
      }

      Eigen::Affine3d Tmotion = Todom_base_prev.inverse()*Todom_base;
      Eigen::Vector3d Tmotion_euler = Tmotion.rotation().eulerAngles(0,1,2);
      ndt_generic::normalizeEulerAngles(Tmotion_euler);

      /*    if(Tmotion.translation().norm()<min_dist && Tmotion_euler.norm()<(min_rot_in_deg*M_PI/180.0)) {
          cloud.clear();
          cloud_nofilter.clear();
          continue;
        }
        */
      counter++;
      fuser_pose=fuser_pose*Tmotion;

      if(visualize){
        br.sendTransform(tf::StampedTransform(tf_gt_base,ros::Time::now(), "/world", "/state_base_link"));
        if(counter%5==0){
          cloud.header.frame_id="/velodyne";
          pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
          cloud_pub->publish(cloud);
        }
        gt_pose_msg.header.stamp=ros::Time::now();
        tf::poseEigenToMsg(Tgt_base, gt_pose_msg.pose.pose);
        gt_pub->publish(gt_pose_msg);
        fuser_pose_msg.header.stamp=ros::Time::now();
        tf::poseEigenToMsg(fuser_pose, fuser_pose_msg.pose.pose);
        fuser_pub->publish(fuser_pose_msg);
      }

      /// MCL på något vänster
      //  fuser_->ProcessFrame(cloud,fuser_pose,Tmotion);
      Eigen::Affine3d T;
      graph_map->SwitchToClosestMapNode(fuser_pose,unit_covar,T,std::numeric_limits<double>::max());
      if(visualize && counter%5==0){
        GraphPlot::PlotPoseGraph(graph_map);
        NDTMapPtr curr_node = boost::dynamic_pointer_cast< NDTMapType >(graph_map->GetCurrentNode()->GetMap());
        GraphPlot::SendGlobalMapToRviz(curr_node->GetMap(),1,graph_map->GetCurrentNodePose());
      }
      double diff = (fuser_pose.inverse() * Tgt_base).translation().norm();
      cout<<"norm between estimated and actual pose="<<diff<<endl;
      Tgt_base_prev = Tgt_base;
      Todom_base_prev = Todom_base;
      cloud.clear();
      cloud_nofilter.clear();

      eval_files.Write( vreader.getTimeStampOfLastSensorMsg(),Tgt_base,Todom_base,fuser_pose,sensor_offset);

    }
  }
  eval_files.Close();


  if (alive) {
    while (1) {
      usleep(1000);
    }
  }
  usleep(1000*1000);
  std::cout << "Done." << std::endl;
}
