#include <ndt_fuser/ndt_fuser_hmt.h>
#include <ndt_offline/VelodyneBagReader.h>
#include <ndt_generic/eigen_utils.h>
#include <ndt_generic/pcl_utils.h>
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
#include "ndt_mcl/3d_ndt_mcl.h"
#include "ndt_generic/io.h"
#include "graph_localisation/localisation_factory.h"
#include "graph_localisation/localisation_type.h"
#include "mcl_ndt/mcl_ndt.h"
#include "graph_map/graph_map_navigator.h"
#include "ndt_offline/readbagfilegeneric.h"
using namespace libgraphMap;
namespace po = boost::program_options;
using namespace std;
using namespace lslgeneric;
using namespace GraphMapLocalisation;
std::string map_dir_name="";
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
bool filter_ring_nb=false;
bool step_control=false;
bool registration2d=true;
bool alive=false;
bool disable_reg=false, do_soft_constraints=false;
bool save_eval_results=false;
lslgeneric::MotionModel2d::Params motion_params;
std::string base_link_id="", gt_base_link_id="", tf_world_frame="", tf_fuser_frame="fuser";
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
unsigned int n_particles=0;
double SIR_varP_threshold=0;
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
double z_filter_min_height=0;
double score_cell_weight=0;
ros::Publisher *gt_pub,*fuser_pub,*cloud_pub,*odom_pub;
nav_msgs::Odometry gt_pose_msg,fuser_pose_msg,odom_pose_msg;
pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cloud;
LocalisationTypePtr localisation_type_ptr;
LocalisationParamPtr localisation_param_ptr;
GraphMapNavigatorPtr graph_map;
ReadBagFileGeneric<pcl::PointXYZ> *reader;
/// Set up the sensor link
tf::StampedTransform sensor_link; ///Link from /odom_base_link -> velodyne
std::string bagfilename;
std::string reader_type="velodyne_reader";
bool use_pointtype_xyzir;
int min_nb_points_for_gaussian;
bool keep_min_nb_points;
bool min_nb_points_set_uniform;

template<class T> std::string toString (const T& x)
{
  std::ostringstream o;

  if (!(o << x))
    throw std::runtime_error ("::toString()");

  return o.str ();
}

template <typename PointT> void filter_ring_nb_fun(pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &cloud_nofilter, const std::set<int>& rings) {
  std::cerr << "Can only filter_ring_nb if they are of type velodyne_pointcloud::PointXYZIR" << std::endl;
  cloud = cloud_nofilter;
}

template <> void filter_ring_nb_fun<velodyne_pointcloud::PointXYZIR>(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud,
                                                                 pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud_nofilter,
                                                                 const std::set<int>& rings) {
  for(int i=0; i<cloud_nofilter.points.size(); ++i) {
    if (rings.find((int)cloud_nofilter[i].ring) != rings.end()) {
      cloud.points.push_back(cloud_nofilter.points[i]);
    }
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
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
    cout<<"foud oru parameters"<<endl;
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



bool LocateMapFilePath(const std::string &folder_name,std::vector<std::string> &scanfiles){
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (folder_name.c_str())) != NULL) {
    while ((ent = readdir (dir)) != NULL) {
      if(ent->d_name[0] == '.') continue;
      char tmpcname[400];
      snprintf(tmpcname,399,"%s/%s",folder_name.c_str(),ent->d_name);
      std::string tmpfname = tmpcname;
      if(tmpfname.substr(tmpfname.find_last_of(".") + 1) == "MAP"|| tmpfname.substr(tmpfname.find_last_of(".") + 1) == "map") {
        scanfiles.push_back(tmpfname);
      }
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
void ReadAllParameters(po::options_description &desc,int &argc, char ***argv){

  Eigen::Vector3d transl;
  Eigen::Vector3d euler;
  std::string localisation_type="";
  // First of all, make sure to advertise all program options
  desc.add_options()
      ("help", "produce help message")
      ("map-file-path", po::value<std::string>(&map_file_name)->default_value(std::string("")), "file path to .MAP file containing graphMapNavigator")
      ("map-dir-path", po::value<string>(&map_dir_name), "Folder to locate .MAP files in")
      ("reader-type", po::value<std::string>(&reader_type)->default_value(std::string("velodyne_reader")), "Type of reader to use when open rosbag e.g. velodyne_reader (config file needed) or pcl_reader when opening pcl2 messages")
      ("bag-file-path", po::value<string>(&bagfilename)->default_value(""), "File path to rosbag to play with maps")
      ("visualize", "visualize the rosbag and fuser estimate/gt")
      ("save-results", "save trajectory for gt, estimation, sensor and odometry")
      ("base-name", po::value<string>(&base_name)->default_value(std::string("mcl")), "prefix for all generated files")
      ("output-dir-name", po::value<string>(&output_dir_name)->default_value(""), "where to save the pieces of the map (default it ./map)")
      ("data-set", po::value<string>(&dataset)->default_value(""), "where to save the pieces of the map (default it ./map)")
      ("localisation-algorithm-name", po::value<string>(&localisation_type)->default_value("mcl_ndt"), "name of localisation algorihm e.g. mcl_ndt")
      ("filter-fov", "cutoff part of the field of view")
      ("hori-max", po::value<double>(&hori_max)->default_value(2*M_PI), "the maximum field of view angle horizontal")
      ("hori-min", po::value<double>(&hori_min)->default_value(-hori_max), "the minimum field of view angle horizontal")
      ("filter-ring-nb", "if the number of rings should be reduced")
      ("z-filter-height", po::value<double>(&z_filter_min_height)->default_value(-10000.0), "The minimum height of which ndtcells are used for localisation")
      ("score-cell-weight", po::value<double>(&score_cell_weight)->default_value(0.1), "The constant score added to the likelihood by hitting a cell with a gaussian.")
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
      ("n-particles", po::value<unsigned int>(&n_particles)->default_value(270), "Total number of particles to use")
      ("SIR_varP_threshold", po::value<double>(&SIR_varP_threshold)->default_value(0.6), "resampling threshold")
      ("resolution_local_factor", po::value<double>(&resolution_local_factor)->default_value(1.), "resolution factor of the local map used in the match and fusing step")
      ("use_pointtype_xyzir", "If the points to be processed should contain ring and intensity information (velodyne_pointcloud::PointXYZIR)")
      ("min_nb_points_for_gaussian", po::value<int>(&min_nb_points_for_gaussian)->default_value(6), "minimum number of points per cell to compute a gaussian")
      ("keep_min_nb_points", "If the number of points stored in a NDTCell should be cleared if the number is less than min_nb_points_for_gaussian")
      ("min_nb_points_set_uniform", "If the number of points of one cell is less than min_nb_points_for_gaussian, set the distribution to a uniform one (cov = Identity)")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, *argv, desc), vm);
  po::notify(vm);

  keep_min_nb_points = vm.count("clear_min_nb_points");
  min_nb_points_set_uniform = vm.count("min_nb_points_set_uniform");
  NDTCell::setParameters(0.1, 8*M_PI/18., 1000, min_nb_points_for_gaussian, !keep_min_nb_points, min_nb_points_set_uniform);

  save_eval_results=vm.count("save-results");
  visualize = vm.count("visualize");
  filter_fov = vm.count("filter-fov");
  filter_ring_nb = vm.count("filter-ring-nb");
  use_pointtype_xyzir = vm.count("use_pointtype_xyzir");
  //Check if all iputs are assigned
  if (!vm.count("map-dir-path") && !vm.count("map-file-path")){
    cout << "No .map file specified. Missing map-dir-path and map-file-path.\n";
    cout << desc << "\n";
    exit(0);
  }
  if (vm.count("help")){
    cout << desc << "\n";
    exit(0);
  }

  localisation_param_ptr=LocalisationFactory::CreateLocalisationParam(localisation_type);
  if(MCLNDTParamPtr parPtr=boost::dynamic_pointer_cast<MCLNDTParam>(localisation_param_ptr )){
    parPtr->resolution=resolution;
    parPtr->n_particles_=n_particles;
    parPtr->z_filter_min=z_filter_min_height;
    parPtr->score_cell_weight=score_cell_weight;
    parPtr->SIR_varP_threshold=SIR_varP_threshold;

    if (dataset == "hx") {
      parPtr->motion_model[0] = 0.01;
      parPtr->motion_model[1] = 0.002;
      parPtr->motion_model[2] = 0.001;
      parPtr->motion_model[3] = 0.001;
      parPtr->motion_model[4] = 0.001;
      parPtr->motion_model[5] = 0.005;

      parPtr->motion_model[6] = 0.002;
      parPtr->motion_model[7] = 0.005;
      parPtr->motion_model[8] = 0.001;
      parPtr->motion_model[9] = 0.001;
      parPtr->motion_model[10] = 0.001;
      parPtr->motion_model[11] = 0.005;

      parPtr->motion_model[12] = 0.005;
      parPtr->motion_model[13] = 0.001;
      parPtr->motion_model[14] = 0.01;
      parPtr->motion_model[15] = 0.0001;
      parPtr->motion_model[16] = 0.0001;
      parPtr->motion_model[17] = 0.005;

      parPtr->motion_model[18] = 0.002;
      parPtr->motion_model[19] = 0.001;
      parPtr->motion_model[20] = 0.001;
      parPtr->motion_model[21] = 0.01;
      parPtr->motion_model[22] = 0.001;
      parPtr->motion_model[23] = 0.001;

      //      parPtr->motion_model[24] = 0.002;
      //      parPtr->motion_model[25] = 0.0001;
      //      parPtr->motion_model[26] = 0.001;
      //      parPtr->motion_model[27] = 0.001;
      //      parPtr->motion_model[28] = 0.01;
      //      parPtr->motion_model[29] = 0.001;
      parPtr->motion_model[25] = 0.005;
      parPtr->motion_model[26] = 0.002;
      parPtr->motion_model[24] = 0.0001;
      parPtr->motion_model[27] = 0.001;
      parPtr->motion_model[28] = 0.04;
      parPtr->motion_model[29] = 0.001;

      parPtr->motion_model[30] = 0.005;
      parPtr->motion_model[31] = 0.002;
      parPtr->motion_model[32] = 0.0001;
      parPtr->motion_model[33] = 0.001;
      parPtr->motion_model[34] = 0.001;
      parPtr->motion_model[35] = 0.01;

      parPtr->motion_model_offset[0] = 0.02;

      parPtr->motion_model_offset[1] = 0.00002;
      parPtr->motion_model_offset[2] = 0.002;
      parPtr->motion_model_offset[3] = 0.000002;
      parPtr->motion_model_offset[4] = 0.02;//0.000002;
      parPtr->motion_model_offset[5] = 0.000002;
    }
  }

  cout<<"sensor pose"<<endl;
  if(!GetSensorPose(dataset,transl,euler,tf_sensor_pose)) {
    cout << "no valid dataset specified, will use the provided sensor pose params" << endl;
  }
  cout << "transl : " << transl << " euler : " << euler;
  sensor_link.child_frame_id_ = velodyne_frame_id;
  sensor_link.frame_id_ = base_link_id;//tf_base_link; //"/odom_base_link";
  sensor_link.setData(tf_sensor_pose);

  return;
}
void initializeRosPublishers(){
  gt_pub=new ros::Publisher();
  odom_pub=new ros::Publisher();
  fuser_pub=new ros::Publisher();
  cloud_pub=new ros::Publisher();
  *gt_pub    =n_->advertise<nav_msgs::Odometry>("/GT", 50);
  *fuser_pub =n_->advertise<nav_msgs::Odometry>("/fuser", 50);
  *odom_pub =n_->advertise<nav_msgs::Odometry>("/odom", 50);
  *cloud_pub = n_->advertise<pcl::PointCloud<pcl::PointXYZ>>("/points2", 1);
  cout<<"initialized publishers"<<endl;
}
void printParameters(){
  cout<<"Output directory: "<<output_dir_name<<endl;
  if(filter_fov)
    cout << "Filtering FOV of sensor to min/max "<<hori_min<<" "<<hori_max<<endl;
  else
    cout<<"No FOV filter."<<endl;

  if(reader_type.compare("velodyne_reader"));
  cout<<"Velodyne config path:"<<velodyne_config_file<<endl;

  cout<<"Bagfile: "<<bagfilename<<endl;
  cout<<"Lidar topic: "<<velodyne_packets_topic<<", lidar frame id: "<<velodyne_frame_id<<endl;
  cout<<"World frame: "<<tf_world_frame<<", tf topic"<<tf_topic<<endl;
}

template<typename PointT>
void processData() {

srand(time(NULL));
tf::TransformBroadcaster br;
gt_pose_msg.header.frame_id=tf_world_frame;
fuser_pose_msg.header.frame_id=tf_world_frame;
odom_pose_msg.header.frame_id=tf_world_frame;


//NDTMapPtr = NULL;
MapNodePtr curr_node = NULL;

std::vector<std::string> map_file_path;
if(map_file_name.length()>0){
  cout<<"Open single map: " << map_file_name <<endl;
  map_file_path.push_back(map_file_name);
}
else if(map_dir_name.length()>0){
  cout<<"Map directory: "<<map_dir_name<<endl;
  if(LocateMapFilePath(map_dir_name,map_file_path)){
    cout<<"Maps found: "<<endl;
    for (std::vector<std::string>::iterator it = map_file_path.begin() ; it != map_file_path.end(); ++it)
      cout<<*it<<endl;
  }
  else{
    cout<<"No maps found"<<endl;
    exit(0);
  }

}

for (std::vector<string>::iterator it = map_file_path.begin() ; it != map_file_path.end(); ++it){

  string map_file= *it;
  cout<<"Opening map number :"<<(it-map_file_path.begin()+1)<<" out of "<<((map_file_path.end()-map_file_path.begin()))<<endl;
  std::ifstream ifs(map_file);
  boost::archive::text_iarchive ia(ifs);
  ia >> graph_map;


  localisation_param_ptr->graph_map_=graph_map;

  localisation_type_ptr=LocalisationFactory::CreateLocalisationType(localisation_param_ptr);

  if(graph_map==NULL ||localisation_type_ptr==NULL){
    cout<<"problem opening map"<<endl;
    exit(0);
  }

  cout<<"-------------------------- Map and Localisation parameter ----------------------------"<<endl;
  cout<<localisation_type_ptr->ToString()<<endl;
  cout<<"--------------------------------------------------------"<<endl;

  std::string output_file_name = map_file+"_npart="+toString(n_particles)+"_res="+toString(resolution)+"_mpsu="+toString(min_nb_points_set_uniform)+"_mnpfg="+toString(min_nb_points_for_gaussian);
  ndt_generic::CreateEvalFiles eval_files(output_dir_name,output_file_name,save_eval_results);
  int counter = 0;
  ReadBagFileGeneric<PointT> reader(reader_type,
                                base_link_id,
                                velodyne_config_file,
                                bagfilename,
                                velodyne_packets_topic,
                                velodyne_frame_id,
                                tf_world_frame,
                                tf_topic,
                                ros::Duration(3600),
                                &sensor_link, max_range, min_range,
                                sensor_time_offset);
  printParameters();

  pcl::PointCloud<PointT> cloud, cloud_nofilter;
  tf::Transform tf_scan_source;
  tf::Transform tf_gt_base;
  Eigen::Affine3d Todom_base,odom_pose,Todom_base_prev, Todom_init; //Todom_base =current odometry pose, odom_pose=current aligned with gt, Todom_base_prev=previous pose, Todom_init= first odometry pose in dataset.
  Eigen::Affine3d Tgt_base,Tgt_base_prev,Tgt_init;//Tgt_base=current GT pose,Tgt_base_prev=previous GT pose, Tgt_init=first gt pose in dataset;
  ros::Time t0,t1,t2,t3,t4,t5;
  t5=ros::Time::now();
  while(reader.ReadNextMeasurement(cloud_nofilter)){
    t0=ros::Time::now();
    if(!n_->ok())
      exit(0);

    if(cloud_nofilter.size()==0) continue;

    if(filter_fov) {
      ndt_generic::filter_fov_fun(cloud,cloud_nofilter,hori_min,hori_max);
    } else {
      cloud = cloud_nofilter;
    }

    if (filter_ring_nb) {
      std::set<int> rings;
      rings.insert(7);
      cloud_nofilter = cloud;
      cloud.clear();
      filter_ring_nb_fun(cloud, cloud_nofilter, rings);
    }

    if (cloud.size() == 0) continue; // Check that we have something to work with depending on the FOV filter here...

    //  reader.getPoseFor(Todom_base,base_link_id);
    //   reader.getPoseFor(Tgt_base,gt_base_link_id);
    tf::Transform tf_odom_base;
    reader.getPoseFor(tf_odom_base,base_link_id);
    reader.getPoseFor(tf_gt_base,gt_base_link_id);

    Eigen::Affine3d Todom_base,Tgt_base;
    tf::transformTFToEigen(tf_gt_base,Tgt_base);
    tf::transformTFToEigen(tf_odom_base,Todom_base);

    if(counter == 0){
      counter ++;
      cloud.clear();
      cloud_nofilter.clear();
      continue;
    }
    if((counter == 1)){
      Todom_init=Todom_base;
      Tgt_init=Tgt_base;
      Tgt_base_prev = Tgt_base;
      Todom_base_prev = Todom_base;
      graph_map->SwitchToClosestMapNode(Tgt_base);
      Eigen::Affine3d init_pose=graph_map->GetCurrentNodePose().inverse()*Tgt_base;
      Vector6d variances;
      variances<<0.1,0.1,0.000001,0.0000001,0.0000001,0.001;
      localisation_type_ptr->InitializeLocalization(init_pose,variances);
      counter ++;
      cloud.clear();
      cloud_nofilter.clear();
      continue;
    }

    Eigen::Affine3d Tmotion = Todom_base_prev.inverse()*Todom_base;

    t1=ros::Time::now();
    lslgeneric::transformPointCloudInPlace(sensor_offset, cloud);
    localisation_type_ptr->UpdateAndPredict(cloud,Tmotion);
    t2=ros::Time::now();


    fuser_pose=localisation_type_ptr->GetPose();
    odom_pose=Tgt_init*Todom_init.inverse()*Todom_base;//Correct for initial odometry

    if (visualize)
    {
      tf::Transform tf_fuser;
      tf::transformEigenToTF(fuser_pose, tf_fuser);
      br.sendTransform(tf::StampedTransform(tf_fuser,ros::Time::now(), tf_world_frame,  tf_fuser_frame));
      if (tf_world_frame != "/world") {
        tf::Transform tf_none;
        tf_none.setIdentity();
        br.sendTransform(tf::StampedTransform(tf_none, ros::Time::now(), "/world", tf_world_frame));
      }
    }

    if(visualize/* &&counter%30==0*/){ // This is relatively cheap to plot, note also that this is given in the vehicle frame...
      br.sendTransform(tf::StampedTransform(sensor_link,ros::Time::now(), tf_fuser_frame, velodyne_frame_id));
      cloud.header.frame_id=tf_fuser_frame;//velodyne_frame_id;// "/velodyne";
      pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
      cloud_pub->publish(cloud);
    }

    if(visualize){
      gt_pose_msg.header.stamp=ros::Time::now();
      fuser_pose_msg.header.stamp=gt_pose_msg.header.stamp;
      odom_pose_msg.header.stamp=gt_pose_msg.header.stamp;
      tf::poseEigenToMsg(Tgt_base, gt_pose_msg.pose.pose);
      tf::poseEigenToMsg(fuser_pose, fuser_pose_msg.pose.pose);
      tf::poseEigenToMsg(odom_pose, odom_pose_msg.pose.pose);
      odom_pub->publish(odom_pose_msg);
      gt_pub->publish(gt_pose_msg);
      fuser_pub->publish(fuser_pose_msg);
    }

    if(visualize){
      bool plot_map = false;
      if (counter %30 == 2) { // Plot the map occationally...
        plot_map = true;
      }
      GraphPlot::PlotPoseGraph(graph_map);

      if (curr_node == graph_map->GetCurrentNode()) {
        // Do nothing
      }
      else {
        curr_node = graph_map->GetCurrentNode();
        plot_map = true;
      }
      if (plot_map) {
        GraphPlot::PlotMap(curr_node->GetMap(),1,graph_map->GetCurrentNodePose(),/*plotmarker::sphere*/plotmarker::point);
      }
    }

    t3=ros::Time::now();
    double diff = (fuser_pose.translation() - Tgt_base.translation()).norm();
    //cout<<"norm between estimated and actual pose="<<diff<<endl;
    Tgt_base_prev = Tgt_base;
    Todom_base_prev = Todom_base;
    cloud.clear();
    cloud_nofilter.clear();
    eval_files.Write( reader.getTimeStampOfLastSensorMsg(),Tgt_base,odom_pose,fuser_pose,sensor_offset);

    t4=ros::Time::now();
    cout<<"iteration: "<<t5-t4<<", update: "<<t2-t1<<", plot: "<<t3-t2<<endl;
    t5=ros::Time::now();
    counter++;
  }
  eval_files.Close();
}
}


/////////////////////////////////////////////////////////////////////////////////7
/////////////////////////////////////////////////////////////////////////////////7
/// *!!MAIN!!*
/////////////////////////////////////////////////////////////////////////////////7
/////////////////////////////////////////////////////////////////////////////////7
///

int main(int argc, char **argv){



  ros::init(argc, argv, "graph_fuser3d_offline");
  po::options_description desc("Allowed options");
  n_=new ros::NodeHandle("~");
  ros::Time::init();
  initializeRosPublishers();
  ReadAllParameters(desc,argc,&argv);

  if (use_pointtype_xyzir) {
    processData<velodyne_pointcloud::PointXYZIR>();
  }
  else {
    processData<pcl::PointXYZ>();
  }

}
