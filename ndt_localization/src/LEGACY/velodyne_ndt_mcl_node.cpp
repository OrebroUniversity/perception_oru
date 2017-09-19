//perception_oru
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ndt_mcl/particle_filter.hpp"
#include "ndt_map/ndt_map.h"
#include "ndt_map/ndt_conversions.h"
//pcl
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
//ros
#include "geometry_msgs/PoseArray.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/rate.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

//std
#include <Eigen/Dense>
#include <string>
#include <map>
#include <fstream>
#include <iostream>
#include <chrono>

class particle_filter_wrap {
  ros::NodeHandle nh;
  //Map parameters
  std::string mapFile;
  lslgeneric::NDTMap* ndtMap;
  lslgeneric::LazyGrid* mapGrid;
  //MCL
  bool be2D;
  bool forceSIR;
  bool showParticles;
  bool showPose;
  double resolution;
  // std::map<std::string,lslgeneric::init_type> inits;
  std::string initType;
  lslgeneric::particle_filter* MCL;
  int particleCount;
  ros::Publisher mclPosePub;
  //laser input
  std::string inputTopicName;   //std::string laserTopicName;
  ros::Publisher particlePub;
  ros::Subscriber initPoseSub;
  ros::Subscriber PCSub;
  
  double sensorOffsetX;
  double sensorOffsetY;
  double sensorOffsetZ;
  double sensorOffsetR;
  double sensorOffsetP;
  double sensorOffsetT;

  std::string rootTF;
  std::string odomTF;
  std::string baseTF;
  std::string mclTF;

  Eigen::Affine3d tOld;
  bool firstLoad;
  geometry_msgs::PoseArray parMsg;
  double minx;
  double miny;

  double initX;
  double initY;
  double initT;
  double initVar;
  double var_x;
  double var_y;
  double var_th;
  double r_var_x;
  double r_var_y;
  double r_var_th;
  int tres;
  int counter;
  int counterLimit;
  bool flip;
  //  double particlePeriod;//DEBUG
  std::ofstream res;
  std::string resF_name;
  bool initialized;
  double time_0;
  geometry_msgs::Pose initial_pose;
  velodyne_rawdata::RawData dataParser;
  double min_range;
  double max_range;

  double v_size_x;
  double v_size_y;
  double v_size_z;
  double range;
  double fraction;
  double cutoff;

  bool beLaser;
  bool beVelodyne;
  bool bePC;
  
  void Pose2DToTF(Eigen::Vector3d mean, ros::Time ts, Eigen::Affine3d Todometry){
    static tf::TransformBroadcaster br, br_mapOdom;
    tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(0, 0, mean[2]);
    transform.setOrigin( tf::Vector3(mean[0], mean[1], 0.0) );
    transform.setRotation( q );
    //br.sendTransform(tf::StampedTransform(transform, ts, "world", "mcl_pose"));
    // br.sendTransform(tf::StampedTransform(transform, ts, rootTF, "/mcl_pose"));
    ///Compute TF between map and odometry frame
    Eigen::Affine3d Tmcl = getAsAffine(mean[0], mean[1], mean[2]);
    Eigen::Affine3d Tmap_odo = Tmcl * Todometry.inverse();
    tf::Transform tf_map_odo;
    tf_map_odo.setOrigin( tf::Vector3(Tmap_odo.translation() (0), Tmap_odo.translation() (1), 0.0) );
    tf::Quaternion q_map_odo;
    q_map_odo.setRPY( 0, 0, Tmap_odo.rotation().eulerAngles(0, 1, 2) (2) );
    tf_map_odo.setRotation( q_map_odo );
    // /// broadcast TF
    br_mapOdom.sendTransform(tf::StampedTransform(tf_map_odo, ts + ros::Duration(0.3), rootTF, odomTF));
  }


  nav_msgs::Odometry Pose2DToMsg(Eigen::Vector3d mean, Eigen::Matrix3d cov, ros::Time ts, Eigen::Affine3d Todometry){
    nav_msgs::Odometry O;
    static int seq = 0;

    //	O.header.stamp = ts;
    //O.header.seq = seq;
    O.header.frame_id = rootTF;
    O.child_frame_id = mclTF;

    O.pose.pose.position.x = mean[0];
    O.pose.pose.position.y = mean[1];
    tf::Quaternion q;
    q.setRPY(0, 0, mean[2]);
    O.pose.pose.orientation.x = q.getX();
    O.pose.pose.orientation.y = q.getY();
    O.pose.pose.orientation.z = q.getZ();
    O.pose.pose.orientation.w = q.getW();

    O.pose.covariance[0] = cov(0, 0);
    O.pose.covariance[1] = cov(0, 1);
    O.pose.covariance[2] = 0;
    O.pose.covariance[3] = 0;
    O.pose.covariance[4] = 0;
    O.pose.covariance[5] = 0;

    O.pose.covariance[6] = cov(1, 0);
    O.pose.covariance[7] = cov(1, 1);
    O.pose.covariance[8] = 0;
    O.pose.covariance[9] = 0;
    O.pose.covariance[10] = 0;
    O.pose.covariance[11] = 0;

    O.pose.covariance[12] = 0;
    O.pose.covariance[13] = 0;
    O.pose.covariance[14] = 0;
    O.pose.covariance[15] = 0;
    O.pose.covariance[16] = 0;
    O.pose.covariance[17] = 0;

    O.pose.covariance[18] = 0;
    O.pose.covariance[19] = 0;
    O.pose.covariance[20] = 0;
    O.pose.covariance[21] = 0;
    O.pose.covariance[22] = 0;
    O.pose.covariance[23] = 0;

    O.pose.covariance[24] = 0;
    O.pose.covariance[25] = 0;
    O.pose.covariance[26] = 0;
    O.pose.covariance[27] = 0;
    O.pose.covariance[28] = 0;
    O.pose.covariance[29] = 0;

    O.pose.covariance[30] = 0;
    O.pose.covariance[31] = 0;
    O.pose.covariance[32] = 0;
    O.pose.covariance[33] = 0;
    O.pose.covariance[34] = 0;
    O.pose.covariance[35] = cov(2, 2);

    seq++;
    //    ROS_INFO("publishing tf");
    static tf::TransformBroadcaster br, br_mapOdom;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(mean[0], mean[1], 0.0) );
    transform.setRotation( q );
    //br.sendTransform(tf::StampedTransform(transform, ts, "world", "mcl_pose"));
    br.sendTransform(tf::StampedTransform(transform, ts, rootTF, mclTF));
    ///Compute TF between map and odometry frame
    Eigen::Affine3d Tmcl = getAsAffine(mean[0], mean[1], mean[2]);
    Eigen::Affine3d Tmap_odo = Tmcl * Todometry.inverse();
    tf::Transform tf_map_odo;
    tf_map_odo.setOrigin( tf::Vector3(Tmap_odo.translation() (0), Tmap_odo.translation() (1), 0.0) );
    tf::Quaternion q_map_odo;
    q_map_odo.setRPY( 0, 0, Tmap_odo.rotation().eulerAngles(0, 1, 2) (2) );
    tf_map_odo.setRotation( q_map_odo );
    // /// broadcast TF
    br_mapOdom.sendTransform(tf::StampedTransform(tf_map_odo, ts, rootTF, odomTF));
    return O;
  }

  geometry_msgs::PoseArray ParticlesToMsg(std::vector<lslgeneric::particle> particles){
    //ROS_INFO("publishing particles");
    geometry_msgs::PoseArray ret;
    for(int i = 0; i < particles.size(); i++){
      geometry_msgs::Pose temp;
      double x, y, z, r, p, t;
      particles[i].GetXYZ(x, y, z);
      particles[i].GetRPY(r, p, t);
      temp.position.x = x;
      temp.position.y = y;
      temp.position.z = z;
      temp.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, t);
      ret.poses.push_back(temp);
    }
    ret.header.frame_id = rootTF;
    return ret;
  }

  int LoadMap(){
    //FILE * jffin;
    //jffin = fopen(mapFile.c_str(),"r+b");
    mapGrid = new lslgeneric::LazyGrid(resolution);
    ndtMap = new lslgeneric::NDTMap(mapGrid);
    if(ndtMap->loadFromJFF(mapFile.c_str()) < 0)
      return -1;
    return 0;
  }

  lslgeneric::NDTMap* VelodyneToNDT(const velodyne_msgs::VelodyneScan::ConstPtr& scan){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t next = 0; next < scan->packets.size(); ++next){
      velodyne_rawdata::VPointCloud pnts;
      dataParser.unpack(scan->packets[next], pnts);
      for(size_t i = 0; i < pnts.size(); i++){
	pcl::PointXYZ p;
	p.x = pnts.points[i].x;
	p.y = pnts.points[i].y;
	p.z = pnts.points[i].z;                                                                                                                                        
	cloud->push_back(p);
      }
      pnts.clear();
    }                                                                                                                                                                  
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.rotate(Eigen::AngleAxisf(sensorOffsetR, Eigen::Vector3f::UnitX())
		       * Eigen::AngleAxisf(sensorOffsetP, Eigen::Vector3f::UnitY())
		       * Eigen::AngleAxisf(sensorOffsetT, Eigen::Vector3f::UnitZ()));                                                                                                                  ////////////////////
    transform_2.translation() << sensorOffsetX, sensorOffsetY, sensorOffsetZ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);
    lslgeneric::NDTMap *localMap = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution));
    localMap->loadPointCloud(*transformed_cloud, range);
    localMap->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
    cloud->clear();
    transformed_cloud->clear();
    return localMap;
  }

  lslgeneric::NDTMap* PCToNDT(const sensor_msgs::PointCloud2::ConstPtr& scan){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*scan,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
                                                                                                                                                                  
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.rotate(Eigen::AngleAxisf(sensorOffsetR, Eigen::Vector3f::UnitX())
		       * Eigen::AngleAxisf(sensorOffsetP, Eigen::Vector3f::UnitY())
		       * Eigen::AngleAxisf(sensorOffsetT, Eigen::Vector3f::UnitZ()));                                                                                                                  ////////////////////
    transform_2.translation() << sensorOffsetX, sensorOffsetY, sensorOffsetZ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);
    lslgeneric::NDTMap *localMap = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution));
    localMap->loadPointCloud(*transformed_cloud, range);
    localMap->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
    cloud->clear();
    transformed_cloud->clear();
    return localMap;
  }
  
  lslgeneric::NDTMap* LaserToNDT(const sensor_msgs::LaserScan::ConstPtr& scan){
      double cxs,cys,czs;
    //minx=std::numeric_limits<double>::max();
    //miny=std::numeric_limits<double>::max();
    mapGrid->getCellSize(cxs,cys,czs);

    //laserProjector.projectLaser(*msg,tempCloud);
    //pcl::fromROSMsg(tempCloud,tempCloudPCL);

	///Calculate the laser pose with respect to the base
	float dy =sensorOffsetY;
	float dx = sensorOffsetX;
	float alpha = atan2(dy,dx);
	float L = sqrt(dx*dx+dy*dy);

	///Laser pose in base frame
	float lpx = L* cos(alpha);
	float lpy = L * sin(alpha);
	float lpa = sensorOffsetT;

	///Laser scan to PointCloud expressed in the base frame
    int N =(scan->angle_max - scan->angle_min)/scan->angle_increment;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for (int j=0;j<N;j++){
      double r  = scan->ranges[j];
      if(r>=scan->range_min && r<scan->range_max){// && r>close_cut){
        double a  = scan->angle_min + j*scan->angle_increment;
        pcl::PointXYZ pt;
        pt.x = r*cos(a+lpa)+lpx;
        pt.y = r*sin(a+lpa)+lpy;
        pt.z = 0.02 * (double)rand()/(double)RAND_MAX;
        cloud->push_back(pt);
      }
	}

	


    // if(firstLoad&&(informed||ndt)){
    //   for(int i=0;i<cloud->points.size();i++){
    //     if(cloud->points[i].x<minx)
    //       minx=cloud->points[i].x;
    //     if(cloud->points[i].y<miny)
    //       miny=cloud->points[i].y;
    //   }
    // }
	    lslgeneric::NDTMap *localMap = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution));
    localMap->guessSize(0, 0, 0,max_range*2.0,max_range*2.0,cxs*3);
    localMap->loadPointCloud(*cloud);
    localMap->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
    //    ROS_INFO_STREAM("cloud size " << cloud->size());
    //    ROS_INFO_STREAM("map size size " << localMap->numberOfActiveCells());
    return localMap;

  }





    /*

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 pc2;
    pcl::PCLPointCloud2 pcl_pc2;
    laser_geometry::LaserProjection projector_;
    projector_.projectLaser(*scan, pc2);
    pcl_conversions::toPCL(pc2,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.rotate(Eigen::AngleAxisf(sensorOffsetR, Eigen::Vector3f::UnitX())
		       * Eigen::AngleAxisf(sensorOffsetP, Eigen::Vector3f::UnitY())
		       * Eigen::AngleAxisf(sensorOffsetT, Eigen::Vector3f::UnitZ()));                                                                                                                  ////////////////////
    transform_2.translation() << sensorOffsetX, sensorOffsetY, sensorOffsetZ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);
    lslgeneric::NDTMap *localMap = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution));
    localMap->loadPointCloud(*transformed_cloud, range);
    localMap->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
     /////////////////////////////////
    ROS_INFO_STREAM(cloud->size());
    ROS_INFO_STREAM(transformed_cloud->size());
    pcl::PointXYZ p=*(cloud->begin());
    ROS_INFO_STREAM("cloud point"<<p.x<<" "<<p.y<<" "<<p.z);
    p=*(transformed_cloud->begin());
    ROS_INFO_STREAM("transformed_cloud point"<<p.x<<" "<<p.y<<" "<<p.z);
     std::vector<lslgeneric::NDTCell*> vec_ndt;
    vec_ndt=localMap->getAllCells();
    ROS_INFO_STREAM("No NDT"<<vec_ndt.size());
    ROS_INFO_STREAM("No NDT"<<);
    ROS_INFO_STREAM("range"<<range);
    /////////////////////////////////
    cloud->clear();
    transformed_cloud->clear();
    return localMap;
  }
    */
  void VeloCallback(const velodyne_msgs::VelodyneScan::ConstPtr& msg){
    //ROS_INFO_STREAM("VELO recived");
    sensor_msgs::PointCloud2 tempCloud;
    pcl::PointCloud<pcl::PointXYZ> tempCloudPCL;
    static tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    double x, y, yaw;
    if(!initialized){
      initialized = true;
      MCL->InitializeNormal(initial_pose.position.x, initial_pose.position.y, tf::getYaw(initial_pose.orientation), initVar);
      //MCL->InitializeNormal(0, 0,0, initVar);
      ROS_INFO("Initialized");

    }
    tf_listener.waitForTransform(odomTF, baseTF, msg->header.stamp, ros::Duration(0.1));
    try{
      tf_listener.lookupTransform(odomTF, baseTF,msg->header.stamp, transform);
      yaw = tf::getYaw(transform.getRotation());
      x = transform.getOrigin().x();
      y = transform.getOrigin().y();
    }
    catch(tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      return;
    }
    Eigen::Affine3d T = getAsAffine(x, y, yaw);
    if(firstLoad){
      tOld = T;
      firstLoad = false;
    }
    Eigen::Affine3d tMotion = tOld.inverse() * T;

    tOld = T;
    lslgeneric::NDTMap *localNDT = VelodyneToNDT(msg);
    MCL->UpdateAndPredictEff(tMotion, localNDT,fraction, cutoff);
    //ROS_INFO_STREAM(r_var_x<<" "<<r_var_y<<" "<<r_var_th);
    //MCL->UpdateAndPredictEffRe(tMotion, localNDT,fraction, cutoff, var_x, var_y, var_th, r_var_x, r_var_y, r_var_th, tres);
    //MCL->UpdateAndPredict(tMotion, localNDT);
    delete localNDT;
    if(showParticles)
      particlePub.publish(ParticlesToMsg(MCL->particleCloud));
    Eigen::Matrix3d cov;
    Eigen::Vector3d mean;
    nav_msgs::Odometry mcl_pose;
    Eigen::Vector3d mean_pose=MCL->GetMeanPose2D();
    Pose2DToTF(mean_pose,msg->header.stamp,T);

    //ROS_INFO("publishing pose");
    MCL->GetPoseMeanAndVariance2D(mean, cov);
    mcl_pose = Pose2DToMsg(mean, cov, msg->header.stamp, T);
    //ROS_INFO_STREAM(mean);
    if(showPose){
    mcl_pose.header.stamp = msg->header.stamp;
    mclPosePub.publish(mcl_pose);
    }
  }


  void PCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    //ROS_INFO_STREAM("VELO recived");
    sensor_msgs::PointCloud2 tempCloud;
    pcl::PointCloud<pcl::PointXYZ> tempCloudPCL;
    static tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    double x, y, yaw;
    if(!initialized){
      initialized = true;
      MCL->InitializeNormal(initial_pose.position.x, initial_pose.position.y, tf::getYaw(initial_pose.orientation), initVar);
      //MCL->InitializeNormal(0, 0,0, initVar);
      ROS_INFO("Initialized");

    }
    tf_listener.waitForTransform(odomTF, baseTF, msg->header.stamp, ros::Duration(0.1));
    try{
      tf_listener.lookupTransform(odomTF, baseTF,/*ros::Time(0)*/msg->header.stamp, transform);
      yaw = tf::getYaw(transform.getRotation());
      x = transform.getOrigin().x();
      y = transform.getOrigin().y();
    }
    catch(tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      return;
    }
    Eigen::Affine3d T = getAsAffine(x, y, yaw);
    if(firstLoad){
      tOld = T;
      firstLoad = false;
    }
    Eigen::Affine3d tMotion = tOld.inverse() * T;

    tOld = T;
    lslgeneric::NDTMap *localNDT = PCToNDT(msg);
    MCL->UpdateAndPredictEff(tMotion, localNDT,fraction, cutoff);
    //ROS_INFO_STREAM(r_var_x<<" "<<r_var_y<<" "<<r_var_th);
    //MCL->UpdateAndPredictEffRe(tMotion, localNDT,fraction, cutoff, var_x, var_y, var_th, r_var_x, r_var_y, r_var_th, tres);
    //MCL->UpdateAndPredict(tMotion, localNDT);
    delete localNDT;
    if(showParticles)
      particlePub.publish(ParticlesToMsg(MCL->particleCloud));
    Eigen::Matrix3d cov;
    Eigen::Vector3d mean;
    nav_msgs::Odometry mcl_pose;
    Eigen::Vector3d mean_pose=MCL->GetMeanPose2D();
    Pose2DToTF(mean_pose,msg->header.stamp,T);
    MCL->GetPoseMeanAndVariance2D(mean, cov);
    mcl_pose = Pose2DToMsg(mean, cov, msg->header.stamp, T);
    //ROS_INFO_STREAM(mean);
    if(showPose){
    mcl_pose.header.stamp = msg->header.stamp;
    mclPosePub.publish(mcl_pose);
    }
    
    //ROS_INFO("publishing pose");
    // MCL->GetPoseMeanAndVariance2D(mean, cov);
    // mcl_pose = Pose2DToMsg(mean, cov, msg->header.stamp, T);
    //ROS_INFO_STREAM(mean);
    //if(showPose){
    //mcl_pose.header.stamp = msg->header.stamp;
    //mclPosePub.publish(mcl_pose);
    //}
  }


  void processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, 
                    Eigen::Affine3d tMotion, ros::Time ts){
    Eigen::Affine3d T;
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.rotate(Eigen::AngleAxisf(sensorOffsetR, Eigen::Vector3f::UnitX())
		       * Eigen::AngleAxisf(sensorOffsetP, Eigen::Vector3f::UnitY())
		       * Eigen::AngleAxisf(sensorOffsetT, Eigen::Vector3f::UnitZ()));                                                                                                                  ////////////////////
    transform_2.translation() << sensorOffsetX, sensorOffsetY, sensorOffsetZ;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(cloud, *transformed_cloud, transform_2);
    lslgeneric::NDTMap *localMap = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution));
    localMap->loadPointCloud(*transformed_cloud, range);
    localMap->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
    cloud.clear();
    transformed_cloud->clear();

    MCL->UpdateAndPredictEff(tMotion, localMap,fraction, cutoff);
    delete localMap;
        if(showParticles)
      particlePub.publish(ParticlesToMsg(MCL->particleCloud));
    Eigen::Matrix3d cov;
    Eigen::Vector3d mean;
    nav_msgs::Odometry mcl_pose;
    Eigen::Vector3d mean_pose=MCL->GetMeanPose2D();
    Pose2DToTF(mean_pose,ts,T);
    MCL->GetPoseMeanAndVariance2D(mean, cov);
    mcl_pose = Pose2DToMsg(mean, cov, ts, T);
    //ROS_INFO_STREAM(mean);
    if(showPose){
    mcl_pose.header.stamp = ts;
    mclPosePub.publish(mcl_pose);
    }


  }
    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
      laser_geometry::LaserProjection projector_;
      sensor_msgs::PointCloud2 cloud;
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud_unfiltered, pcl_cloud;
      //      message_m.lock();
      projector_.projectLaser(*msg, cloud);
      //message_m.unlock();
      pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);
      
      pcl::PointXYZ pt;
      //add some variance on z
      for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
	pt = pcl_cloud_unfiltered.points[i];
	if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_range) {
	  pt.z += 0.1*((double)rand())/(double)INT_MAX;
	  pcl_cloud.points.push_back(pt);
	}
      }
      ROS_INFO("Got laser points");
      
      static tf::TransformListener tf_listener;
      tf::StampedTransform transform;
      double x, y, yaw;
      if(!initialized){
	initialized = true;
	MCL->InitializeNormal(initial_pose.position.x, initial_pose.position.y, tf::getYaw(initial_pose.orientation), initVar);
	//MCL->InitializeNormal(0, 0,0, initVar);
	ROS_INFO("Initialized");
    }
      tf_listener.waitForTransform(odomTF, baseTF, msg->header.stamp, ros::Duration(0.1));
      try{
	tf_listener.lookupTransform(odomTF, baseTF,/*ros::Time(0)*/msg->header.stamp, transform);
	yaw = tf::getYaw(transform.getRotation());
	x = transform.getOrigin().x();
	y = transform.getOrigin().y();
      }
      catch(tf::TransformException ex){
	ROS_ERROR("%s", ex.what());
	return;
      }
      Eigen::Affine3d T = getAsAffine(x, y, yaw);
      if(firstLoad){
	tOld = T;
	firstLoad = false;
      }
      Eigen::Affine3d tMotion = tOld.inverse() * T;
      
      tOld = T;
      
      this->processFrame(pcl_cloud,tMotion,msg->header.stamp);

      ///////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////
  //   //ROS_INFO_STREAM("VELO recived");
  //   sensor_msgs::PointCloud2 tempCloud;
  //   pcl::PointCloud<pcl::PointXYZ> tempCloudPCL;
  //   static tf::TransformListener tf_listener;
  //   tf::StampedTransform transform;
  //   double x, y, yaw;
  //   if(!initialized){
  //     initialized = true;
  //     MCL->InitializeNormal(initial_pose.position.x, initial_pose.position.y, tf::getYaw(initial_pose.orientation), initVar);
  //     //MCL->InitializeNormal(0, 0,0, initVar);
  //     ROS_INFO("Initialized");

  //   }
  //   tf_listener.waitForTransform(odomTF, baseTF, msg->header.stamp, ros::Duration(0.1));
  //   try{
  //     tf_listener.lookupTransform(odomTF, baseTF,/*ros::Time(0)*/msg->header.stamp, transform);
  //     yaw = tf::getYaw(transform.getRotation());
  //     x = transform.getOrigin().x();
  //     y = transform.getOrigin().y();
  //   }
  //   catch(tf::TransformException ex){
  //     ROS_ERROR("%s", ex.what());
  //     return;
  //   }
  //   Eigen::Affine3d T = getAsAffine(x, y, yaw);
  //   if(firstLoad){
  //     tOld = T;
  //     firstLoad = false;
  //   }
  //   Eigen::Affine3d tMotion = tOld.inverse() * T;

  //   tOld = T;
  //   lslgeneric::NDTMap *localNDT = LaserToNDT(msg);
   
  //   MCL->UpdateAndPredictEff(tMotion, localNDT,fraction, cutoff);
  //   //ROS_INFO_STREAM(r_var_x<<" "<<r_var_y<<" "<<r_var_th);
  //   //MCL->UpdateAndPredictEffRe(tMotion, localNDT,fraction, cutoff, var_x, var_y, var_th, r_var_x, r_var_y, r_var_th, tres);
  //   //MCL->UpdateAndPredict(tMotion, localNDT);
  //    ///////////////////////////////////////////////////
   
   
  // ROS_INFO_STREAM("local map size " << localNDT->numberOfActiveCells());
  //   ROS_INFO_STREAM("gloabl map size " << ndtMap->numberOfActiveCells());
  // std::vector<lslgeneric::NDTCell*> cells_local= localNDT->getAllCells();
  //   std::vector<lslgeneric::NDTCell*> cells_global= ndtMap->getAllCells();
  // ROS_INFO_STREAM(" local mean"<<cells_local[0]->getMean());
  //   ROS_INFO_STREAM(" global mean"<<cells_global[0]->getMean());
  //     //////////////////////////////////
  //   delete localNDT;
  //   if(showParticles)
  //     particlePub.publish(ParticlesToMsg(MCL->particleCloud));
  //   Eigen::Matrix3d cov;
  //   Eigen::Vector3d mean;
  //   nav_msgs::Odometry mcl_pose;
  //   Eigen::Vector3d mean_pose=MCL->GetMeanPose2D();
  //   Pose2DToTF(mean_pose,msg->header.stamp,T);

  //   //ROS_INFO("publishing pose");
  //    MCL->GetPoseMeanAndVariance2D(mean, cov);
  //    mcl_pose = Pose2DToMsg(mean, cov, msg->header.stamp, T);
  //    //    ROS_INFO_STREAM(mean);
  //   if(showPose){
  //   mcl_pose.header.stamp = msg->header.stamp;
  //   mclPosePub.publish(mcl_pose);
  //   }
      ///////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////
    }


  void initialposeCallback(geometry_msgs::PoseWithCovarianceStamped input_init){
    initialized = false;
    initial_pose = input_init.pose.pose;
    firstLoad = true;
  }
  
  Eigen::Affine3d getAsAffine(float x, float y, float yaw ){
    Eigen::Matrix3d m;

    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Translation3d v(x, y, 0);
    Eigen::Affine3d T = v * m;
    return T;
  }

public:
  particle_filter_wrap(ros::NodeHandle param){
    param.param<std::string>("map_file", mapFile, "file.jff");
    param.param<double>("resolution", resolution, 0.2);
    param.param<bool>("be2D", be2D, true);
    param.param<bool>("force_SIR", forceSIR, true);
    param.param<bool>("show_particles", showParticles, false);
    param.param<bool>("show_pose", showPose, true);
    param.param<int>("particle_count", particleCount, 500);
    
    param.param<std::string>("input_topic_name", inputTopicName, "/velodyne_packets");
    param.param<bool>("Velodyne", beVelodyne, false);
    param.param<bool>("PoinCloud", bePC, false);
    param.param<bool>("Laser", beLaser, false);
    
    param.param<std::string>("root_tf", rootTF, "/map");
    param.param<std::string>("odom_tf", odomTF, "/odom");
    param.param<std::string>("base_tf", baseTF, "/base_link");
    param.param<std::string>("mcl_tf", mclTF, "/mcl");
    
    param.param<double>("sensor_pose_x", sensorOffsetX, 0.0);
    param.param<double>("sensor_pose_y", sensorOffsetY, 0.0);
    param.param<double>("sensor_pose_z", sensorOffsetZ, 0.0);
    param.param<double>("sensor_pose_r", sensorOffsetR, 0.0);
    param.param<double>("sensor_pose_p", sensorOffsetP, 0.0);
    param.param<double>("sensor_pose_t", sensorOffsetT, 0.0);
    
    param.param<double>("initial_x", initX, 0.0);
    param.param<double>("initial_y", initY, 0.0);
    param.param<double>("initial_Th", initT, 0.0);
    param.param<double>("init_var", initVar, 0.5);
    
    param.param<bool>("flip", flip, false);
    param.param<double>("v_size_x", v_size_x, 200.0);
    param.param<double>("v_size_y", v_size_y, 200.0);
    param.param<double>("v_size_z", v_size_z, 6.0);

    param.param<double>("var_x", var_x, 0.07);
    param.param<double>("var_y", var_y, 0.07);
    param.param<double>("var_th", var_th, 0.035);

    param.param<double>("r_var_x", r_var_x, 1.0);
    param.param<double>("r_var_y", r_var_y, 1.0);
    param.param<double>("r_var_th", r_var_th, 0.001);

    param.param<int>("keep", tres, 400);
    param.param<double>("range", range, 40.0);
    param.param<double>("fraction", fraction, 1.0);
    param.param<double>("cutoff", cutoff, -10);
    
    param.param("min_range", min_range, 2.0);
    param.param("max_range", max_range, 100.0);
    int retu = dataParser.setup(param);
    
    dataParser.setParameters(min_range, max_range, 0, 2 * 3.1415);

    /////////
    firstLoad = true;
    initialized = false;
    res.open(resF_name);
    std::cout << "velodyne_ndt_mcl_node will load map\n";
    if(LoadMap() < 0){
      std::cout << "velodyne_ndt_mcl_node load map failed; shutting down\n";
      ros::shutdown();
    }
    std::cout << "velodyne_ndt_mcl_node load map OK\n";
    initial_pose.position.x = initX;
    initial_pose.position.x = initY;
    initial_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, initY);

    MCL = new lslgeneric::particle_filter(ndtMap, particleCount, be2D, forceSIR);
    
    if(beVelodyne)
      PCSub = nh.subscribe(inputTopicName, 1, &particle_filter_wrap::VeloCallback, this);
    if(bePC)
      PCSub = nh.subscribe(inputTopicName, 1, &particle_filter_wrap::PCCallback, this);
    if(beLaser)
      PCSub = nh.subscribe(inputTopicName, 1, &particle_filter_wrap::LaserCallback, this);
    
    initPoseSub = nh.subscribe("/initialpose", 1000, &particle_filter_wrap::initialposeCallback, this);
    
    if(showParticles)
      particlePub = nh.advertise<geometry_msgs::PoseArray>("particles", 1);
    if(showPose)
      mclPosePub=nh.advertise<nav_msgs::Odometry>("ndt_mcl_pose", 1);
    
    std::cout << "velodyne_ndt_mcl_node will now enter spin()\n";
    ros::spin();
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "ndt_mcl");
  ros::NodeHandle parameters("~");
  particle_filter_wrap pf(parameters);
}


