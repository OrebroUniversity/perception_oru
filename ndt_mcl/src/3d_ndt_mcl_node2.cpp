/**
* 3D NDT-MCL Node. 
* This application runs the ndt-mcl localization based on a map and laser scanner and odometry. 
* 
* The initialization is based now on ground truth pose information (should be replaced with manual input). 
* 
* The visualization depends on mrpt-gui 
* 
* More details about the algorithm:
* Jari Saarinen, Henrik Andreasson, Todor Stoyanov and Achim J. Lilienthal, Normal Distributions Transform Monte-Carlo Localization (NDT-MCL)
* IEEE/RSJ International Conference on Intelligent Robots and Systems November 3-8, 2013, Tokyo Big Sight, Japan
* 
* @author Jari Saarinen (jari.p.saarinen@gmail.com), mods by Todor Stoyanov
* @NOTE: experimental 3D version in progress 
* @TODO Global initialization possibility 
* Known issues: in launch file (or hard coded parameters) you have to set the same resolution for NDT map as is saved -- otherwise it wont work
*/

#include <ndt_visualisation/ndt_viz.h>

#include <ros/ros.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "sensor_msgs/PointCloud2.h"

//#include "ndt_mcl/impl/ndt_mcl.hpp"
#include "ndt_mcl/3d_ndt_mcl.h"
#include <ndt_map/ndt_map.h>

#include <ndt_rviz/ndt_rviz.h>
//#include <ndt_feature/ndt_feature_rviz.h>

inline void normalizeEulerAngles(Eigen::Vector3d &euler) {
  if (fabs(euler[0]) > M_PI/2) {
    euler[0] += M_PI;
    euler[1] += M_PI;
    euler[2] += M_PI;
    
    euler[0] = angles::normalize_angle(euler[0]);
    euler[1] = angles::normalize_angle(euler[1]);
    euler[2] = angles::normalize_angle(euler[2]);
  }
}

std::string affine3dToString(const Eigen::Affine3d &T) {
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::digits10);
  Eigen::Vector3d rot = T.rotation().eulerAngles(0,1,2);
  normalizeEulerAngles(rot);
  
  stream << T.translation().transpose() << " " << rot.transpose();
  return stream.str();
}


#define SYNC_FRAMES 20

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> PointsOdomSync;
//TODO typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> PointsPoseSync;

class NDTMCL3DNode {

    private:
	ros::NodeHandle nh_;
	NDTMCL3D *ndtmcl;
        boost::mutex mcl_m,message_m;

        message_filters::Subscriber<sensor_msgs::PointCloud2> *points2_sub_;
        message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber scan2_sub_;
        ros::Subscriber gt_sub;

	///Laser sensor offset
	Eigen::Affine3d sensorPoseT; //<<Sensor offset with respect to odometry frame
        ros::Duration sensorTimeOffset_;
	Eigen::Affine3d sensorPoseT2; //<<Sensor offset with respect to odometry frame
        ros::Duration sensorTimeOffset2_;
	Eigen::Affine3d Told,Todo,Todo_old,Tcum; //<<old and current odometry transformations
	Eigen::Affine3d initPoseT; //<<Sensor offset with respect to odometry frame

        pcl::PointCloud<pcl::PointXYZ> scan2_cloud_;
    ros::Time scan2_t0_;
            

    bool use_dual_scan;
	bool hasSensorPose, hasInitialPose;
	bool isFirstLoad;
	bool forceSIR, do_visualize;
	bool saveMap;						///< indicates if we want to save the map in a regular intervals
	std::string mapName; ///<name and the path to the map
	std::string output_map_name;
	double resolution;
	double subsample_level;
	int pcounter;
        NDTViz ndt_viz;

	ros::Publisher mcl_pub; ///< The output of MCL is published with this!
    ros::Publisher marker_pub_;
    ros::Publisher pointcloud_pub_;
    message_filters::Synchronizer< PointsOdomSync > *sync_po_; 

    std::string tf_base_link, tf_sensor_link, tf_gt_link, points_topic, odometry_topic, odometry_frame, scan_topic, scan2_topic;

  ros::Timer heartbeat_slow_visualization_;
  ros::Timer heartbeat_fast_visualization_;
bool do_pub_ndt_markers_;
  bool do_pub_particles_markers_;

  int numParticles;
  std::ofstream gt_file_;
  std::ofstream gt2d_file_;
    
  std::ofstream est_file_;
  std::ofstream est2d_file_;
  std::string gt_topic;

  bool use_initial_pose_from_gt;

    boost::shared_ptr<velodyne_rawdata::RawData> data_; 
    boost::shared_ptr<velodyne_rawdata::RawData> data2_;
   tf::TransformListener tf_listener;
    
    int skip_nb_rings_;
    int skip_start_;

    int cloud_min_size_;
    
    public:
    NDTMCL3DNode(ros::NodeHandle param_nh) : data_(new velodyne_rawdata::RawData()), data2_(new velodyne_rawdata::RawData()) {
	    
	    //////////////////////////////////////////////////////////
            // Setup the data parser for the raw packets
	    //////////////////////////////////////////////////////////
        {
            data_->setup(param_nh); // To get the calibration file
            double max_range, min_range, view_direction, view_width;
            param_nh.param<double>("max_range", max_range, 130.);
            param_nh.param<double>("min_range", min_range, 2.);
            param_nh.param<double>("view_direction", view_direction, 0.);
            param_nh.param<double>("view_width", view_width, 6.3);
            data_->setParameters(min_range, max_range, view_direction, view_width);

            param_nh.param<int>("skip_nb_rings", skip_nb_rings_, 0);
            param_nh.param<int>("skip_start", skip_start_, 0);
            param_nh.param<int>("cloud_min_size", cloud_min_size_, 2000); // minimum amount of points (to avoid some problems with lost packages...).
        }
            // Need to have a separate configuration file for the second scanner...
            std::string calibration2;
            param_nh.param<std::string>("calibration2", calibration2, std::string(""));
            {
                if (calibration2 != std::string("")) {
                    ros::NodeHandle nh_tmp("scan2");
                    nh_tmp.setParam("calibration", calibration2);
                    data2_->setup(nh_tmp);
                    double max_range, min_range, view_direction, view_width;
                    param_nh.param<double>("max_range2", max_range, 130.);
                    param_nh.param<double>("min_range2", min_range, 2.);
                    param_nh.param<double>("view_direction2", view_direction, 0.);
                    param_nh.param<double>("view_width2", view_width, 6.3);
                    data2_->setParameters(min_range, max_range, view_direction, view_width);        }
            } 
            
	    //////////////////////////////////////////////////////////
	    /// Prepare Pose offsets
	    //////////////////////////////////////////////////////////
	    bool use_sensor_pose, use_initial_pose;
	    double pose_init_x,pose_init_y,pose_init_z,
		   pose_init_r,pose_init_p,pose_init_t;
	    double sensor_pose_x,sensor_pose_y,sensor_pose_z,
                   sensor_pose_r,sensor_pose_p,sensor_pose_t;

	    param_nh.param<bool>("set_sensor_pose", use_sensor_pose, true);
	    param_nh.param<bool>("set_initial_pose", use_initial_pose, false);
            param_nh.param<bool>("set_initial_pose_from_gt", use_initial_pose_from_gt, false);

	    if(use_initial_pose) {
		///initial pose of the vehicle with respect to the map
		param_nh.param("pose_init_x",pose_init_x,0.);
		param_nh.param("pose_init_y",pose_init_y,0.);
		param_nh.param("pose_init_z",pose_init_z,0.);
		param_nh.param("pose_init_r",pose_init_r,0.);
		param_nh.param("pose_init_p",pose_init_p,0.);
		param_nh.param("pose_init_t",pose_init_t,0.);
		initPoseT =  Eigen::Translation<double,3>(pose_init_x,pose_init_y,pose_init_z)*
		    Eigen::AngleAxis<double>(pose_init_r,Eigen::Vector3d::UnitX()) *
		    Eigen::AngleAxis<double>(pose_init_p,Eigen::Vector3d::UnitY()) *
		    Eigen::AngleAxis<double>(pose_init_t,Eigen::Vector3d::UnitZ()) ;

		hasInitialPose=true;
	    } else {
		hasInitialPose=false;
	    }

	    if(use_sensor_pose) {
		///pose of the sensor with respect to the vehicle odometry frame
		param_nh.param("sensor_pose_x",sensor_pose_x,0.);
		param_nh.param("sensor_pose_y",sensor_pose_y,0.);
		param_nh.param("sensor_pose_z",sensor_pose_z,0.);
		param_nh.param("sensor_pose_r",sensor_pose_r,0.);
		param_nh.param("sensor_pose_p",sensor_pose_p,0.);
		param_nh.param("sensor_pose_t",sensor_pose_t,0.);
		hasSensorPose = true;
		sensorPoseT =  Eigen::Translation<double,3>(sensor_pose_x,sensor_pose_y,sensor_pose_z)*
		    Eigen::AngleAxis<double>(sensor_pose_r,Eigen::Vector3d::UnitX()) *
		    Eigen::AngleAxis<double>(sensor_pose_p,Eigen::Vector3d::UnitY()) *
		    Eigen::AngleAxis<double>(sensor_pose_t,Eigen::Vector3d::UnitZ()) ;
	    
                param_nh.param("sensor_pose2_x",sensor_pose_x,0.);
		param_nh.param("sensor_pose2_y",sensor_pose_y,0.);
		param_nh.param("sensor_pose2_z",sensor_pose_z,0.);
		param_nh.param("sensor_pose2_r",sensor_pose_r,0.);
		param_nh.param("sensor_pose2_p",sensor_pose_p,0.);
		param_nh.param("sensor_pose2_t",sensor_pose_t,0.);
		
                sensorPoseT2 =  Eigen::Translation<double,3>(sensor_pose_x,sensor_pose_y,sensor_pose_z)*
		    Eigen::AngleAxis<double>(sensor_pose_r,Eigen::Vector3d::UnitX()) *
		    Eigen::AngleAxis<double>(sensor_pose_p,Eigen::Vector3d::UnitY()) *
		    Eigen::AngleAxis<double>(sensor_pose_t,Eigen::Vector3d::UnitZ()) ;
                
            } else {
		hasSensorPose = false;
	    }

            double sensor_time_offset;
            param_nh.param("sensor_time_offset", sensor_time_offset, 0.);
            sensorTimeOffset_ = ros::Duration(sensor_time_offset);
            param_nh.param("sensor_time_offset2", sensor_time_offset, 0.);
            sensorTimeOffset2_ = ros::Duration(sensor_time_offset);
            
	    //////////////////////////////////////////////////////////
	    /// Prepare the map
	    //////////////////////////////////////////////////////////
	    param_nh.param<std::string>("map_file_name", mapName, std::string("basement.ndmap"));
	    param_nh.param<bool>("save_output_map", saveMap, true);
	    param_nh.param<std::string>("output_map_file_name", output_map_name, std::string("ndt_mapper_output.ndmap"));
	    param_nh.param<double>("map_resolution", resolution , 0.2);
	    param_nh.param<double>("subsample_level", subsample_level , 1);

            fprintf(stderr,"USING RESOLUTION %lf\n",resolution);
	    
	    lslgeneric::NDTMap ndmap(new lslgeneric::LazyGrid(resolution));
	    ndmap.loadFromJFF(mapName.c_str());
            ROS_INFO_STREAM("Loaded map: " << mapName << " containing " << ndmap.getAllCells().size() << " cells");
            

	    //////////////////////////////////////////////////////////
	    /// Prepare MCL object 
	    //////////////////////////////////////////////////////////

	    ndtmcl = new NDTMCL3D(resolution,ndmap,-1000);
	    param_nh.param<bool>("forceSIR", forceSIR, false);
	    if(forceSIR) ndtmcl->forceSIR=true;

	    fprintf(stderr,"*** FORCE SIR = %d****",forceSIR);
	    param_nh.param<double>("SIR_varP_threshold", ndtmcl->SIR_varP_threshold, 0.006);
            param_nh.param<int>("SIR_max_iters_wo_resampling", ndtmcl->SIR_max_iters_wo_resampling, 25);
            param_nh.param<int>("numParticles", numParticles, 100);

            param_nh.getParam("motion_model", ndtmcl->motion_model);
            param_nh.getParam("motion_model_offset", ndtmcl->motion_model_offset);
	    param_nh.param<double>("resolution_sensor", ndtmcl->resolution_sensor, resolution);
            

            mcl_pub = nh_.advertise<nav_msgs::Odometry>("ndt_mcl",10);
	    
	    //////////////////////////////////////////////////////////
	    /// Prepare the callbacks and message filters
	    //////////////////////////////////////////////////////////
	    //the name of the TF link associated to the base frame / odometry frame    
	    param_nh.param<std::string>("tf_base_link", tf_base_link, std::string("/base_link"));
	    //the name of the tf link associated to the 3d laser scanner
	    param_nh.param<std::string>("tf_laser_link", tf_sensor_link, std::string("/velodyne_link"));
            param_nh.param<std::string>("tf_gt_link", tf_gt_link, std::string(""));


            ///topic to wait for packet data 
            param_nh.param<std::string>("scan_topic", scan_topic, "");
            param_nh.param<std::string>("scan2_topic", scan2_topic, "");
            use_dual_scan = false;
            if (scan2_topic != std::string("")) {
                use_dual_scan = true;
                if (calibration2 == std::string("")) {
                    ROS_ERROR("no calibration2 parameter given for the scan2 scanner!!!");
                }
            }

	    ///topic to wait for point clouds
	    param_nh.param<std::string>("points_topic",points_topic,"points");
	    ///topic to wait for odometry messages
	    param_nh.param<std::string>("odometry_topic",odometry_topic,"odometry");
            param_nh.param<std::string>("odometry_frame", odometry_frame, "/world");

            
            // If a scan_topic is provided, force to use it
            if (scan_topic != std::string("")) {
                ROS_INFO_STREAM("A scan topic is specified [will use that along with tf] : " << scan_topic);
                scan_sub_ = nh_.subscribe<velodyne_msgs::VelodyneScan>(scan_topic,10,&NDTMCL3DNode::scan_callback, this);
                if (scan2_topic != std::string("")) {
                    scan2_sub_ = nh_.subscribe<velodyne_msgs::VelodyneScan>(scan2_topic,10,&NDTMCL3DNode::scan2_callback, this);
                    ROS_INFO_STREAM("A scan2 topic is specified [will use that along with tf] : " << scan_topic);
                }
            }
            else {
                points2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,points_topic,1);
                odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
                sync_po_ = new message_filters::Synchronizer< PointsOdomSync >(PointsOdomSync(SYNC_FRAMES), *points2_sub_, *odom_sub_);
                sync_po_->registerCallback(boost::bind(&NDTMCL3DNode::callback, this, _1, _2));
            }
	    isFirstLoad=true;
	    pcounter =0;

            //////////////////////////////////////////////////////////
            /// Visualization
            //////////////////////////////////////////////////////////
            param_nh.param<bool>("do_visualize", do_visualize, true);
	    param_nh.param<bool>("do_pub_ndt_markers", do_pub_ndt_markers_, true);
            param_nh.param<bool>("do_pub_particles_markers", do_pub_particles_markers_, true);

            if (do_visualize) {
                ndt_viz.win3D->start_main_loop_own_thread();
            }
            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 3);
            pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("interppoints", 15);

            heartbeat_slow_visualization_   = nh_.createTimer(ros::Duration(1.0),&NDTMCL3DNode::publish_visualization_slow,this);
            heartbeat_fast_visualization_   = nh_.createTimer(ros::Duration(0.1),&NDTMCL3DNode::publish_visualization_fast,this);
    
            //////////////////////////////////////////////////////////
            /// Evaluation
            //////////////////////////////////////////////////////////
            std::string gt_filename, gt2d_filename, est_filename, est2d_filename;
            param_nh.param<std::string>("gt_topic",gt_topic,"");
            param_nh.param<std::string>("output_gt_file",  gt_filename, "loc_gt_pose.txt");
            param_nh.param<std::string>("output_gt2d_file",  gt2d_filename, "loc_gt2d_pose.txt");
            param_nh.param<std::string>("output_est_file", est_filename, "loc_est_pose.txt"); 
            param_nh.param<std::string>("output_est2d_file", est2d_filename, "loc_est2d_pose.txt");
            if (gt_filename != std::string("")) {
              gt_file_.open(gt_filename.c_str());
              if (tf_gt_link == std::string("")) {
                  ROS_WARN("tf_gt_link not set - will not log any gt data");
              }
            }
            if (gt2d_filename != std::string("")) {
              gt2d_file_.open(gt2d_filename.c_str());
              if (tf_gt_link == std::string("")) {
                  ROS_WARN("tf_gt_link not set - will not log any gt data");
              }
            }

            if (est_filename != std::string("")) {
              est_file_.open(est_filename.c_str());
            }
            if (est2d_filename != std::string("")) {
              est2d_file_.open(est2d_filename.c_str());
            }
            
            if (!gt_file_.is_open() || !gt2d_file_.is_open() || !est_file_.is_open() || !est2d_file_.is_open())
            {
                ROS_ERROR_STREAM("Failed to open : " << est_file_ << " | " << est2d_file_ << " | " << gt_file_ << " | " << gt2d_file_); 
            }
                        
            if (gt_topic != std::string("")) 
            {
              ROS_INFO_STREAM("Subscribing to : " << gt_topic);
              gt_sub = nh_.subscribe<nav_msgs::Odometry>(gt_topic,10,&NDTMCL3DNode::gt_callback, this);	
            }
        

	}
	~NDTMCL3DNode() {
          if (gt_file_.is_open())
            gt_file_.close();
          if (gt2d_file_.is_open())
            gt2d_file_.close();
          if (est_file_.is_open())
            est_file_.close();
          if (est2d_file_.is_open())
            est2d_file_.close();
          
	    delete points2_sub_;
	    delete odom_sub_;
	    delete sync_po_;
	    delete ndtmcl;
	}

    void publish_visualization_fast(const ros::TimerEvent &event) {
                if (do_pub_particles_markers_) 
                {
                  mcl_m.lock();
                  marker_pub_.publish(ndt_visualisation::markerParticlesNDTMCL3D(*ndtmcl, -1, "particles"));
                  mcl_m.unlock();
                }
    }

  void publish_visualization_slow(const ros::TimerEvent &event) {
                if (do_pub_ndt_markers_)
                {
                  // visualization_msgs::Marker markers_ndt;
                  // ndt_visualisation::markerNDTCells2(*(graph->getLastFeatureFuser()->map),
                  //                                    graph->getT(), 1, "nd_global_map_last", markers_ndt);
                  // marker_pub_.publish(markers_ndt);
                  mcl_m.lock();
                  marker_pub_.publish(ndt_visualisation::markerNDTCells(ndtmcl->map, 1, "nd_map"));
                  mcl_m.unlock();
                }
  }
    
    void visualize() {
        //update visualization
        ///DRAW STUFF///
        if(do_visualize) {
            pcounter++;
            if(pcounter%500==0){
                ndt_viz.clear();
                ndt_viz.plotNDTSAccordingToOccupancy(-1,&ndtmcl->map);
                
            }
            
            //if(pcounter%10==0){
            
            ndt_viz.clearParticles();
            for(int i=0;i<ndtmcl->pf.size();i++){
                double x,y,z;
                ndtmcl->pf.pcloud[i].getXYZ(x,y,z);
                ndt_viz.addParticle(x, y,z+0.5, 1.0, 1.0, 1.0);
            }
            
            Eigen::Affine3d mean = ndtmcl->pf.getMean();
            ndt_viz.addTrajectoryPoint(mean.translation()[0],mean.translation()[1],mean.translation()[2]+0.5,1.0,0,0);	
            Eigen::Vector3d tr = Tcum.translation();
            ndt_viz.addTrajectoryPoint(tr[0],tr[1],tr[2]+0.5,0.0,1.0,0);	
            
            //ndt_viz.addPointCloud(cloudT,1,0,0);
            ndt_viz.displayParticles();
            ndt_viz.displayTrajectory();
            ndt_viz.win3D->setCameraPointingToPoint(mean.translation()[0],mean.translation()[1],3.0);
            ndt_viz.repaint();
            //}
        }
    }

    void initialize() {
        //if not, check if initial robot pose has been set
        if(!hasInitialPose) {
            //can't do anything, wait for pose message...
            ROS_INFO("waiting for initial pose");
            return;
        }
        //initialize filter
        Eigen::Vector3d tr = initPoseT.translation();
        Eigen::Vector3d rot = initPoseT.rotation().eulerAngles(0,1,2);
	
        Todo_old=Todo;
        Tcum = initPoseT;

        ndtmcl->initializeFilter(tr[0], tr[1],tr[2],rot[0],rot[1],rot[2],0.5, 0.5, 0.1, 2.0*M_PI/180.0, 2.0*M_PI/180.0 ,2.0*M_PI/180.0, this->numParticles);
        //        ndtmcl->initializeFilter(tr[0], tr[1],tr[2],rot[0],rot[1],rot[2],0.005, 0.005, 0.001, 0.02*M_PI/180.0, 0.02*M_PI/180.0 ,0.02*M_PI/180.0, this->numParticles);
        ndt_viz.plotNDTSAccordingToOccupancy(-1,&ndtmcl->map);
        isFirstLoad = false;
    }
    

    // Simply to get an easier interface to tf.
    bool getTransformationForTime(const ros::Time &t0, const std::string &frame_id, tf::Transform &T) {
        bool success = false;
        try {
            success = tf_listener.waitForTransform(odometry_frame, frame_id, t0, ros::Duration(2.0));
        }
        catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(100, "%s", ex.what());
            return false;
        }
        if (!success) {
            return false;
        }
        tf::StampedTransform Ts;
        tf_listener.lookupTransform (odometry_frame, frame_id, t0, Ts);
        T = Ts;
        return true;
    }

    // Simply to get an easier interface to tf.
    bool getRelativeTransformationForTime(const ros::Time &t0,const ros::Time &t1, const std::string &frame_id,tf::Transform &T)  {
        bool success = false;
        try { 
            success = tf_listener.waitForTransform(frame_id, t0, frame_id, t1, odometry_frame, ros::Duration(2.0));
        }
        catch (tf::TransformException &ex) {
            ROS_WARN_THROTTLE(100, "%s", ex.what());
            return false;
        }
        if (!success) {
            return false;
        }
        tf::StampedTransform Ts;
        tf_listener.lookupTransform (frame_id, t0, frame_id, t1, odometry_frame, Ts);
        T = Ts;
        return true;
    }

    bool getTransformationAffine3d(const ros::Time &t0, const std::string &frame_id, Eigen::Affine3d &T) {
        tf::Transform tf_T;
        if (!getTransformationForTime(t0, frame_id, tf_T)) {
            ROS_ERROR_STREAM("Failed to get from /tf : " << frame_id);
            return false;
        }
        tf::transformTFToEigen(tf_T, T);
        return true;
    }

        
    // Access the scans directly. This to better handle the time offsets / perform interpolation. Anyway it is useful to have better access to the points.
    void scan_callback(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg) {
        mcl_m.lock();

      // Get the time stamp from the header, the pose at this time will be seen as the origin.
        ros::Time t0; // Corresponding time stamp which relates the scan to the odometry tf frame.
        t0 = scanMsg->header.stamp + sensorTimeOffset_;

        // If we are using the dual scan setup then we need to have the timestamp of the previous one in order to get syncronized pointclouds
        if (use_dual_scan) {
            if (scan2_cloud_.empty()) {
                mcl_m.unlock();
                return;
            }
            t0 = scan2_t0_ - sensorTimeOffset2_ + sensorTimeOffset_;  
        }

        // Get the current odometry position (need to check the incremental odometry meassure since last call).
        {
            tf::Transform T;
            if (!getTransformationForTime(t0, tf_base_link, T)) {
                ROS_INFO_STREAM("Waiting for : " << tf_base_link);
                mcl_m.unlock();
                return;
            }
            tf::transformTFToEigen(T, Todo);
        }
 
        //check if we have done iterations 
        if(isFirstLoad) {
            initialize();
            mcl_m.unlock();
            return;
        }

        Eigen::Affine3d Tm;
        Tm = Todo_old.inverse()*Todo;

        if(Tm.translation().norm()<0.01 && fabs(Tm.rotation().eulerAngles(0,1,2)[2])<(0.5*M_PI/180.0)) {
            ROS_INFO_STREAM("Not enough distance / rot traversed : " << Tm.translation().norm() << " / " << fabs(Tm.rotation().eulerAngles(0,1,2)[2]));
            mcl_m.unlock();
            return;
        }

        // We're interessted in getting the pointcloud in the vehicle frame (that is to transform it using the sensor pose offset).
        pcl::PointCloud<pcl::PointXYZ> cloud;
        
        velodyne_rawdata::VPointCloud pnts,conv_points;
        tf::Transform T_sensorpose;
        tf::transformEigenToTF(sensorPoseT, T_sensorpose);

        for (size_t next = 0; next < scanMsg->packets.size(); ++next) {
            data_->unpack(scanMsg->packets[next], pnts);
            // Get the transformation of this scan packet (the vehicle pose)
            
            ros::Time t1 = scanMsg->packets[next].stamp + sensorTimeOffset_;
            // Get the relative pose...
            tf::Transform T;
            if (!getRelativeTransformationForTime(t0, t1, tf_base_link, T)) {
                continue;
            }
            
            tf::Transform Tcloud =  T * T_sensorpose;
            pcl_ros::transformPointCloud(pnts,conv_points,Tcloud);
            for (size_t i = 0; i < conv_points.size(); i++) {
                if (skip_nb_rings_ == 0) {
                    cloud.push_back(pcl::PointXYZ(conv_points.points[i].x,
                                                  conv_points.points[i].y,
                                                  conv_points.points[i].z));
                }
                else if (conv_points.points[i].ring % (skip_nb_rings_+1) == skip_start_) {
                    cloud.push_back(pcl::PointXYZ(conv_points.points[i].x,
                                                  conv_points.points[i].y,
                                                  conv_points.points[i].z));
                }
                else {
                    
                }
            }
            pnts.clear();
            conv_points.clear();
        }

        // Check size of the cloud
        ROS_INFO_STREAM("cloud.size() : " << cloud.size());
        if (cloud.size() < cloud_min_size_) {
            mcl_m.unlock();
            return;
        }

        
        if (use_dual_scan) {
            cloud += scan2_cloud_;
            scan2_cloud_.clear();
        }

        {
            sensor_msgs::PointCloud2 pcloud;
            pcl::toROSMsg(cloud,pcloud);
            pcloud.header.stamp = t0;
            pcloud.header.frame_id = "interppoints";
            pointcloud_pub_.publish(pcloud);
        }
        // 
        Tcum = Tcum*Tm;
        Todo_old=Todo;

        //update filter -> + add parameter to subsample ndt map in filter step
        ndtmcl->updateAndPredictEff(Tm, cloud, subsample_level);
        //ndtmcl->predict(Tm);

        if (do_visualize) {
            visualize();
        }
        //publish pose
        sendROSOdoMessage(ndtmcl->pf.getMean(),t0);
        mcl_m.unlock();
    }

    // Simply store the point cloud, all the updates are from the scan_callback()
    void scan2_callback(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg) {
        mcl_m.lock();

      // Get the time stamp from the header, the pose at this time will be seen as the origin.
        ros::Time t0;
        t0 = scanMsg->header.stamp + sensorTimeOffset2_;
        scan2_t0_ = t0;
        // Get the current odometry position (need to check the incremental odometry meassure since last call).
        {
            tf::Transform T;
            if (!getTransformationForTime(t0, tf_base_link, T)) {
                ROS_INFO_STREAM("Waiting for : " << tf_base_link);
                mcl_m.unlock();
                return;
            }
            tf::transformTFToEigen(T, Todo);
        }
 
        //check if we have done iterations 
        if(isFirstLoad) {
            initialize();
            mcl_m.unlock();
            return;
        }

        Eigen::Affine3d Tm;
        Tm = Todo_old.inverse()*Todo;

        if(Tm.translation().norm()<0.01 && fabs(Tm.rotation().eulerAngles(0,1,2)[2])<(0.5*M_PI/180.0)) {
            mcl_m.unlock();
            return;
        }

        // We're interessted in getting the pointcloud in the vehicle frame (that is to transform it using the sensor pose offset).
        pcl::PointCloud<pcl::PointXYZ> cloud;
        
        velodyne_rawdata::VPointCloud pnts,conv_points;
        tf::Transform T_sensorpose;
        tf::transformEigenToTF(sensorPoseT2, T_sensorpose);

        for (size_t next = 0; next < scanMsg->packets.size(); ++next) {
            data2_->unpack(scanMsg->packets[next], pnts);
            // Get the transformation of this scan packet (the vehicle pose)
            
            ros::Time t1 = scanMsg->packets[next].stamp + sensorTimeOffset_;
            // Get the relative pose...
            tf::Transform T;
            if (!getRelativeTransformationForTime(t0, t1, tf_base_link, T)) {
                continue;
            }
            
            tf::Transform Tcloud =  T * T_sensorpose;
            pcl_ros::transformPointCloud(pnts,conv_points,Tcloud);
            for (size_t i = 0; i < conv_points.size(); i++) {
                cloud.push_back(pcl::PointXYZ(conv_points.points[i].x,
                                              conv_points.points[i].y,
                                              conv_points.points[i].z));
            }
            pnts.clear();
            conv_points.clear();
        }

        // Check size of the cloud
        ROS_INFO_STREAM("cloud.size() : " << cloud.size());
        if (cloud.size() < 3000) {
            mcl_m.unlock();
            return;
        }

        scan2_cloud_ = cloud;

        mcl_m.unlock();
    }

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//new callback: looks up transforms on TF
	void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in,
		const nav_msgs::Odometry::ConstPtr& odo_in)
	{
	    mcl_m.lock();
	    //compute new odo and point cloud
	    Eigen::Quaterniond qd;
	    Eigen::Affine3d Tm;
	    pcl::PointCloud<pcl::PointXYZ> cloud, cloudT, cloudTO;
	    
	    qd.x() = odo_in->pose.pose.orientation.x;
	    qd.y() = odo_in->pose.pose.orientation.y;
	    qd.z() = odo_in->pose.pose.orientation.z;
	    qd.w() = odo_in->pose.pose.orientation.w;

	    Todo = Eigen::Translation3d (odo_in->pose.pose.position.x,
		    odo_in->pose.pose.position.y,odo_in->pose.pose.position.z) * qd;
	    
	    pcl::fromROSMsg (*cloud_in, cloud);
	    
	    fprintf(stderr,"cloud has %d points\n",(int)cloud.points.size());		
	    //check if sensor pose is known, if not, look it up on TF
	    if(!hasSensorPose) {

	    }
	    //check if we have done iterations 
	    if(isFirstLoad) {
                initialize();
                mcl_m.unlock();
                return;
	    }
	    
	    Tm = Todo_old.inverse()*Todo;
	    if(Tm.translation().norm()<0.01 && fabs(Tm.rotation().eulerAngles(0,1,2)[2])<(0.5*M_PI/180.0)) {
		mcl_m.unlock();
		return;
	    }
	    
	    Tcum = Tcum*Tm;
	    Todo_old=Todo;

	    //cut off user defined slice form the sensor data
	    //transform point cloud from sensor pose into base pose
    
	    ///Set the cloud to sensor frame with respect to base
	    lslgeneric::transformPointCloudInPlace(sensorPoseT, cloud);
	    //cloudT = lslgeneric::transformPointCloud(Tcum, cloud);

	    //update filter -> + add parameter to subsample ndt map in filter step
            ndtmcl->updateAndPredictEff(Tm, cloud, subsample_level);
            

	    if(do_visualize) {
                visualize();
            }

	    //publish pose
	    sendROSOdoMessage(ndtmcl->pf.getMean(),odo_in->header.stamp);
	    mcl_m.unlock();
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	bool sendROSOdoMessage(Eigen::Affine3d mean,ros::Time ts){
	    nav_msgs::Odometry O;
	    static int seq = 0;
	    O.header.stamp = ts;
	    O.header.seq = seq;
	    O.header.frame_id = odometry_frame;
	    O.child_frame_id = "/mcl_pose";

	    O.pose.pose.position.x = mean.translation()[0];
	    O.pose.pose.position.y = mean.translation()[1];
	    O.pose.pose.position.z = mean.translation()[2];
	    Eigen::Quaterniond q (mean.rotation());
	    tf::Quaternion qtf;
	    tf::quaternionEigenToTF (q, qtf);
	    O.pose.pose.orientation.x = q.x();
	    O.pose.pose.orientation.y = q.y();
	    O.pose.pose.orientation.z = q.z();
	    O.pose.pose.orientation.w = q.w();

	    seq++;
	    mcl_pub.publish(O);

	    static tf::TransformBroadcaster br;
	    tf::Transform transform;
	    transform.setOrigin( tf::Vector3(mean.translation()[0],mean.translation()[1], mean.translation()[2]) );

	    transform.setRotation( qtf );
	    br.sendTransform(tf::StampedTransform(transform, ts, "world", "mcl_pose"));

            if (est_file_.is_open()) {
              est_file_ << ts << " " << lslgeneric::transformToEvalString(mean);
            }
            if (est2d_file_.is_open()) {
              est2d_file_ << ts << " " << lslgeneric::transformToEval2dString(mean);
            }
            if (gt_file_.is_open() && tf_gt_link != std::string("")) {
                Eigen::Affine3d T_gt;
                if (getTransformationAffine3d(ts, tf_gt_link, T_gt)) {
                    gt_file_ << ts << " " << lslgeneric::transformToEvalString(T_gt);
                }
                if (gt2d_file_.is_open()) {
                    gt2d_file_ << ts << " " << lslgeneric::transformToEval2dString(T_gt);
                }
            }

	    return true;
	}

	// Callback
	void gt_callback(const nav_msgs::Odometry::ConstPtr& msg_in)
	{
	    Eigen::Quaterniond qd;
	    Eigen::Affine3d gt_pose;

	    qd.x() = msg_in->pose.pose.orientation.x;
	    qd.y() = msg_in->pose.pose.orientation.y;
	    qd.z() = msg_in->pose.pose.orientation.z;
	    qd.w() = msg_in->pose.pose.orientation.w;
	    
	    gt_pose = Eigen::Translation3d (msg_in->pose.pose.position.x,
		    msg_in->pose.pose.position.y,msg_in->pose.pose.position.z) * qd;
	     

            // Query the pose from the /tf instead (see abouve in sendOdoMessage).
            // if (gt_file_.is_open()) {
            //   gt_file_ << msg_in->header.stamp << " " << lslgeneric::transformToEvalString(gt_pose);
            // }

	    // m.lock();
            if(use_initial_pose_from_gt && !hasInitialPose) {
              hasInitialPose = true;
              ROS_INFO("Set initial pose from GT track");
              initPoseT = gt_pose;
            }
	    // m.unlock();
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

};


int main(int argc, char **argv){
	ros::init(argc, argv, "NDT-MCL");
	ros::NodeHandle paramHandle ("~");
	NDTMCL3DNode mclnode(paramHandle);   	
	ros::spin();
	return 0;
}
