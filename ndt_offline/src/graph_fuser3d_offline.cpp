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
#include "ndt_generic/io.h"
#include "ndt_offline/pointcloudbagreader.h"
#include "ndt_offline/readbagfilegeneric.h"
using namespace libgraphMap;
namespace po = boost::program_options;
using namespace std;
using namespace lslgeneric;

std::string dirname="";
std::string output_dir_name="";
std::string base_name="";
std::string dataset="";
std::string bag_reader_type="";
//map parameters
int itrs=0;
int nb_neighbours=0;
int nb_scan_msgs=0;
bool use_odometry=true;
bool visualize=true;
bool use_multires=false;
bool beHMT=false;
bool filter_fov=false;
bool step_control=false;
bool check_consistency=true;
bool registration2d=true;
bool use_submap=true;
double min_keyframe_dist=0.5;
double min_keyframe_dist_rot_deg=15;
bool use_keyframe=true;
bool alive=false;
bool save_map=true;
bool gt_mapping=false;
bool disable_reg=false, do_soft_constraints=false;
bool pcl_reader=true;
lslgeneric::MotionModel2d::Params motion_params;
std::string base_link_id="", gt_base_link_id="", tf_world_frame="";
std::string velodyne_config_file="";
std::string velodyne_packets_topic="";
std::string velodyne_frame_id="";
std::string map_type_name="",registration_type_name="";
std::string tf_topic="";
tf::Transform tf_sensor_pose;
Eigen::Affine3d sensor_offset,fuser_pose;//Mapping from base frame to sensor frame
ros::NodeHandle *n_=NULL;
RegParamPtr regParPtr=NULL;
MapParamPtr mapParPtr=NULL;
GraphParamPtr graphParPtr=NULL;
double sensor_time_offset=0;
double map_size_xy=0;
double map_size_z=0;
double resolution_local_factor=0;
double max_range=0, min_range=0;
double maxRotationNorm_=0;
double compound_radius_=0;
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
//VelodyneBagReader<pcl::PointXYZ> *vreader;
//PointCloudBagReader<pcl::PointXYZ> *preader;
ReadBagFileGeneric *reader;
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
    transl[0]=0.3;
    transl[1]=0;
    transl[2]=0;
    euler[0]=0;
    euler[1]=0;
    euler[2]=-1.62;
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
      ("map_type_name", po::value<string>(&map_type_name)->default_value(std::string("default")), "type of map to use e.g. ndt_map or ndt_dl_map (default it default)")
      ("registration_type_name", po::value<string>(&registration_type_name)->default_value(std::string("default")), "type of map to use e.g. ndt_d2d_reg or ndt_dl_reg (default it default)")
      ("visualize", "visualize the output")
      ("use-odometry", "use initial guess from odometry")
      ("disable-mapping", "build maps from cloud data")
      ("no-step-control", "use step control in the optimization (default=false)")
      ("base-name", po::value<string>(&base_name), "prefix for all generated files")
      ("reader-type", po::value<string>(&bag_reader_type)->default_value("velodyne_reader"), "e.g. velodyne_reader or pcl_reader")
      ("data-set", po::value<string>(&dataset)->default_value(std::string("default")), "choose which dataset that is currently used, this option will assist with assigning the sensor pose")
      ("dir-name", po::value<string>(&dirname), "where to look for ros bags")
      ("output-dir-name", po::value<string>(&output_dir_name)->default_value("/home/daniel/.ros/maps"), "where to save the pieces of the map (default it ./map)")
      ("map-size-xy", po::value<double>(&map_size_xy)->default_value(83.), "size of submaps")
      ("map-size-z", po::value<double>(&map_size_z)->default_value(6.0), "size of submaps")
      ("itrs", po::value<int>(&itrs)->default_value(30), "number of iteration in the registration")
      ("fuse-incomplete", "fuse in registration estimate even if iterations ran out. may be useful in combination with low itr numbers")
      ("filter-fov", "cutoff part of the field of view")
      ("hori-max", po::value<double>(&hori_max)->default_value(2*M_PI), "the maximum field of view angle horizontal")
      ("hori-min", po::value<double>(&hori_min)->default_value(-hori_max), "the minimum field of view angle horizontal")
      ("do-soft-constraints", "if soft constraints from odometry should be used")
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
      ("velodyne-packets-topic", po::value<std::string>(&velodyne_packets_topic)->default_value(std::string("/velodyne_packets")), "velodyne packets topic used")
      ("velodyne_frame_id", po::value<std::string>(&velodyne_frame_id)->default_value(std::string("/velodyne")), "frame_id of the velodyne")
      ("alive", "keep the mapper/visualization running even though it is completed (e.g. to take screen shots etc.")
      ("nb_neighbours", po::value<int>(&nb_neighbours)->default_value(2), "number of neighbours used in the registration")
      ("min_range", po::value<double>(&min_range)->default_value(1.0), "minimum range used from scanner")
      ("max_range", po::value<double>(&max_range)->default_value(30), "minimum range used from scanner")
      ("save-map", "saves the graph map at the end of execution")
      ("nb_scan_msgs", po::value<int>(&nb_scan_msgs)->default_value(1), "number of scan messages that should be loaded at once from the bag")
      ("disable-keyframe-update", "use every scan to update map rather than update map upon distance traveled")
      ("keyframe-min-distance", po::value<double>(&min_keyframe_dist)->default_value(0.5), "minimum range used from scanner")
      ("keyframe-min-rot-deg", po::value<double>(&min_keyframe_dist_rot_deg)->default_value(15), "minimum range used from scanner")
      ("gt-mapping", "disable registration and use ground truth as input to mapping")
      ("tf_topic", po::value<std::string>(&tf_topic)->default_value(std::string("/tf")), "tf topic to listen to")
      ("x", po::value<double>(&transl[0])->default_value(0.), "sensor pose - translation vector x")
      ("y", po::value<double>(&transl[1])->default_value(0.), "sensor pose - translation vector y")
      ("z", po::value<double>(&transl[2])->default_value(0.), "sensor pose - translation vector z")
      ("ex", po::value<double>(&euler[0])->default_value(0.), "sensor pose - euler angle vector x")
      ("ey", po::value<double>(&euler[1])->default_value(0.), "sensor pose - euler angle vector y")
      ("ez", po::value<double>(&euler[2])->default_value(0.), "sensor pose - euler angle vector z")
      ("sensor_time_offset", po::value<double>(&sensor_time_offset)->default_value(0.), "timeoffset of the scanner data")
      ("registration2d","registration2d")
      ("check-consistency", "if consistency should be checked after registration")
      ("do-soft-constraints", "do_soft_constraints_")
      ("disable-registration", "Disable Registration")
      ("maxRotationNorm",po::value<double>(&maxRotationNorm_)->default_value(0.78539816339),"maxRotationNorm")
      ("maxTranslationNorm",po::value<double>(&maxTranslationNorm_)->default_value(0.4),"maxTranslationNorm")
      ("rotationRegistrationDelta",po::value<double>(&rotationRegistrationDelta_)->default_value(M_PI/6),"rotationRegistrationDelta")
      ("translationRegistrationDelta",po::value<double>(&translationRegistrationDelta_)->default_value(1.5),"sensorRange")
      ("resolution", po::value<double>(&resolution)->default_value(0.4), "resolution of the map")
      ("resolution_local_factor", po::value<double>(&resolution_local_factor)->default_value(1.), "resolution factor of the local map used in the match and fusing step")
      ("use-submap", "Adopt the sub-mapping technique which represent the global map as a set of local submaps")
      ("compound-radius", po::value<double>(&compound_radius_)->default_value(10.0), "Requires sub-mapping enabled, When creating new sub-lamps, information from previous map is transfered to the new map. The following radius is used to select the map objects to transfer")
      ("interchange-radius", po::value<double>(&interchange_radius_)->default_value(10.0), "This radius is used to trigger creation or selection of which submap to use");


  //Boolean parameres are read through notifiers
  po::variables_map vm;
  po::store(po::parse_command_line(argc, *argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << "\n";
    return false;
  }

  if(GetSensorPose(dataset,transl,euler,tf_sensor_pose)) {
    cout<<"sensor pose from dataset utilized [" << dataset << "]" << endl;
  }

  mapParPtr= GraphFactory::CreateMapParam(map_type_name); //map_type_name
  regParPtr=GraphFactory::CreateRegParam(registration_type_name);
  graphParPtr=GraphFactory::CreateGraphParam();
  if(mapParPtr==NULL || regParPtr==NULL || graphParPtr==NULL)
    return false;

  use_odometry = vm.count("use-odometry");
  visualize = vm.count("visualize");
  filter_fov = vm.count("filter-fov");
  step_control = (vm.count("no-step-control") == 0);
  gt_mapping= vm.count("gt-mapping");
  use_keyframe=!vm.count("disable-keyframe-update");
  if(gt_mapping)
    base_link_id=gt_base_link_id;
  check_consistency=vm.count("check-consistency");
  alive = vm.count("alive");
  save_map = vm.count("save-map");
  registration2d=vm.count("registration2d");
  do_soft_constraints=vm.count("do-soft-constraints");
  regParPtr->do_soft_constraints_ = vm.count("do-soft-constraints");
  regParPtr->enableRegistration_ = !vm.count("disable-registration") && !gt_mapping;
  regParPtr->registration2d_=registration2d;
  regParPtr->maxRotationNorm_=maxRotationNorm_;
  regParPtr->maxTranslationNorm_=maxTranslationNorm_;
  regParPtr->rotationRegistrationDelta_=rotationRegistrationDelta_;
  regParPtr->translationRegistrationDelta_=translationRegistrationDelta_;
  regParPtr->sensorRange_=max_range;
  regParPtr->mapSizeZ_= map_size_z;
  regParPtr->checkConsistency_=check_consistency;
  mapParPtr->sizez_=map_size_z;
  mapParPtr->max_range_=max_range;
  mapParPtr->min_range_=min_range;
  graphParPtr->compound_radius_=compound_radius_;
  graphParPtr->interchange_radius_=interchange_radius_;
  cout<<"use keyframe="<<use_keyframe;
  graphParPtr->use_keyframe_=use_keyframe;
  graphParPtr->min_keyframe_dist_=min_keyframe_dist;
  graphParPtr->min_keyframe_rot_deg_=min_keyframe_dist_rot_deg;
  mapParPtr->enable_mapping_=!vm.count("disable-mapping");
  mapParPtr->sizey_=map_size_xy;
  mapParPtr->sizex_=map_size_xy;
  use_submap=vm.count("use-submap");
  graphParPtr->use_submap_=use_submap;

  if(  NDTD2DRegParamPtr ndt_reg_ptr=boost::dynamic_pointer_cast<NDTD2DRegParam>(regParPtr)){
    ndt_reg_ptr->resolution_=resolution;
    ndt_reg_ptr->resolutionLocalFactor_=resolution_local_factor;
  }
  if(  NDTMapParamPtr ndt_map_ptr=boost::dynamic_pointer_cast<NDTMapParam>(mapParPtr)){
    ndt_map_ptr->resolution_=resolution;
  }


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
std::string boolToString(bool input){
  return input?std::string("true"):std::string("false");
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

  cout<<"read params"<<endl;
  bool succesfull=ReadAllParameters(desc,argc,&argv);
  if(!succesfull)
    exit(0);

  cout<<"node handle"<<endl;
  n_=new ros::NodeHandle("~");
  cout<<"test"<<endl;

  GraphMapFuser *fuser_;

  ndt_generic::CreateEvalFiles eval_files(output_dir_name,base_name,false);
  printParameters();
  initializeRosPublishers();
  tf::TransformBroadcaster br;
  gt_pose_msg.header.frame_id="/world";
  fuser_pose_msg.header.frame_id="/world";

  std::string tf_interp_link = base_link_id;
  if(gt_mapping)
    tf_interp_link = gt_base_link_id;

  stringstream ss;
  string name= gt_mapping? "_gt_":"_fuser_";
  ss<<name<<dataset<<std::string("_Sub=")<<use_submap<<"_sizexy="<<map_size_xy<<"_Z="<<map_size_z<<std::string("_intrchR=")<<interchange_radius_<<std::string("_compR=")<<compound_radius_<<std::string("_res=") <<resolution<<std::string("maxSensd=") << max_range<<"keyF="<<use_keyframe<<"_d="<<min_keyframe_dist<<"_deg="<<min_keyframe_dist_rot_deg;
  base_name+=ss.str();
  //base_name += dataset+std::string("_Sub")+boolToString(use_submap)+std::string("intch_r")+interchange_radius_+std::string("comp_r_")+compound_radius_+std::string("_res") + toString(resolution)+ std::string("_sensCut") + toString(max_range);

  ros::Time::init();
  srand(time(NULL));

  //ndtslammer.disableRegistration = disable_reg;

  /// Set up the sensor link
  tf::StampedTransform sensor_link; ///Link from /odom_base_link -> velodyne
  sensor_link.child_frame_id_ = velodyne_frame_id;
  sensor_link.frame_id_ = tf_interp_link;//tf_base_link; //"/odom_base_link";
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
    char c=getchar();
    reader=new ReadBagFileGeneric(bag_reader_type,
                                  tf_interp_link,
                                  velodyne_config_file,
                                  bagfilename,
                                  velodyne_packets_topic,
                                  velodyne_frame_id,
                                  tf_world_frame,
                                  tf_topic,
                                  ros::Duration(3600),
                                  &sensor_link, max_range, min_range,
                                  sensor_time_offset);
  }
  pcl::PointCloud<pcl::PointXYZ> cloud, cloud_nofilter;
  tf::Transform tf_scan_source;
  tf::Transform tf_gt_base;
  Eigen::Affine3d Todom_base_prev,Tgt_base_prev;
  ros::Time itr=ros::Time::now();
  ros::Time itr_end=itr;
  bool found_scan=true;

  while(found_scan){

    //end_of_bag_file=  vreader->readMultipleMeasurements(nb_scan_msgs,cloud_nofilter,tf_scan_source,tf_gt_base,tf_interp_link);
    //found_scan= preader->readNextMeasurement(cloud_nofilter);
    found_scan= reader->ReadNextMeasurement(cloud_nofilter);
    if(!n_->ok())
      exit(0);


    cout<<"iteration time="<<ros::Time::now()-itr<<endl;
    itr=ros::Time::now();
    cout<<"time to read bag file="<<itr_end-itr<<endl;

    if(cloud_nofilter.size()==0) continue;

    if(filter_fov) {
      filter_fov_fun(cloud,cloud_nofilter,hori_min,hori_max);
    } else {
      cloud = cloud_nofilter;
    }
    cout<<"cloud size:"<<cloud.size()<<endl;
    if (cloud.size() == 0) continue; // Check that we have something to work with depending on the FOV filter here...

    tf::Transform tf_odom_base;
    reader->getPoseFor(tf_odom_base, base_link_id);
    reader->getPoseFor(tf_gt_base, gt_base_link_id);
    //  vreader->getPoseFor(tf_odom_base, base_link_id);
    // vreader->getPoseFor(tf_gt_base, gt_base_link_id);

    Eigen::Affine3d Todom_base,Tgt_base;
    tf::transformTFToEigen(tf_gt_base,Tgt_base);
    tf::transformTFToEigen(tf_odom_base,Todom_base);

    if(counter == 0){
      if( found_scan!=true){
        cout<<"Cannot find any scans at all"<<endl;
        exit(0);
      }
      counter ++;
      cloud.clear();
      cloud_nofilter.clear();
      continue;
    }
    if(counter == 1){
      fuser_pose=Tgt_base;
      Tgt_base_prev = Tgt_base;
      Todom_base_prev = Todom_base;
      fuser_=new GraphMapFuser(regParPtr,mapParPtr,graphParPtr,Tgt_base,sensor_offset);
      cout<<"----------------------PARAMETERS FOR MAPPING--------------------------"<<endl;
      cout<<fuser_->ToString()<<endl;
      cout<<"----------------------PARAMETERS FOR MAPPING--------------------------"<<endl;
      fuser_->Visualize(visualize);
      counter ++;
      cloud.clear();
      cloud_nofilter.clear();
      continue;
    }
    if( found_scan!=true)
      break;

    Eigen::Affine3d Tmotion = Todom_base_prev.inverse()*Todom_base;
    Eigen::Vector3d Tmotion_euler = Tmotion.rotation().eulerAngles(0,1,2);
    ndt_generic::normalizeEulerAngles(Tmotion_euler);
    if(!use_odometry) {
      Tmotion.setIdentity();
    }
    counter++;

    fuser_pose=fuser_pose*Tmotion;

    if(visualize){
      br.sendTransform(tf::StampedTransform(tf_gt_base,ros::Time::now(), "/world", "/state_base_link"));


      cloud.header.frame_id="/velodyne";
      pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
      cloud_pub->publish(cloud);

      gt_pose_msg.header.stamp=ros::Time::now();
      tf::poseEigenToMsg(Tgt_base, gt_pose_msg.pose.pose);
      gt_pub->publish(gt_pose_msg);
      if(!gt_mapping){
        fuser_pose_msg.header.stamp=ros::Time::now();
        tf::poseEigenToMsg(fuser_pose, fuser_pose_msg.pose.pose);
        fuser_pub->publish(fuser_pose_msg);
      }
    }

    fuser_->ProcessFrame(cloud,fuser_pose,Eigen::Affine3d::Identity());
    double diff = (fuser_pose.inverse() * Tgt_base).translation().norm();
    if(visualize ){
      fuser_->plotMap();
      cout<<"norm between estimated and actual pose="<<diff<<endl;
    }
    sleep(1);
    Tgt_base_prev = Tgt_base;
    Todom_base_prev = Todom_base;
    cloud.clear();
    cloud_nofilter.clear();

    eval_files.Write( reader->getTimeStampOfLastSensorMsg(),Tgt_base,Todom_base,fuser_pose,sensor_offset);
    itr_end=ros::Time::now();
  }

  eval_files.Close();

  if(save_map && fuser_!=NULL && fuser_->FramesProcessed()>0){
    char path[1000];
    snprintf(path,999,"%s/%s.map",output_dir_name.c_str(),base_name.c_str());
    fuser_->SaveGraphMap(path);
    exit(0);
  }

  if (alive) {
    while (1) {
      usleep(1000);
    }
  }
  usleep(1000*1000);
  std::cout << "Done." << std::endl;
}
