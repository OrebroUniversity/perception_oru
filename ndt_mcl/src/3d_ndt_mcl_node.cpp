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

//#define USE_VISUALIZATION_DEBUG ///< Enable / Disable visualization
#include <ndt_visualisation/ndt_viz.h>
#include <mrpt/utils/CTicTac.h>

#include <ros/ros.h>
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
#include "ndt_mcl/impl/3d_ndt_mcl.hpp"
#include <ndt_map/ndt_map.h>

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

	///Laser sensor offset
	Eigen::Affine3d sensorPoseT; //<<Sensor offset with respect to odometry frame
	Eigen::Affine3d Told,Todo,Todo_old,Tcum; //<<old and current odometry transformations
	Eigen::Affine3d initPoseT; //<<Sensor offset with respect to odometry frame

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
        message_filters::Synchronizer< PointsOdomSync > *sync_po_;

	std::string tf_base_link, tf_sensor_link, points_topic, odometry_topic;
    public:
	NDTMCL3DNode(ros::NodeHandle param_nh) {
	    

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
	    } else {
		hasSensorPose = false;
	    }

	    //////////////////////////////////////////////////////////
	    /// Prepare the map
	    //////////////////////////////////////////////////////////
	    param_nh.param<std::string>("map_file_name", mapName, std::string("basement.ndmap"));
	    param_nh.param<bool>("save_output_map", saveMap, true);
	    param_nh.param<bool>("do_visualize", do_visualize, true);
	    param_nh.param<std::string>("output_map_file_name", output_map_name, std::string("ndt_mapper_output.ndmap"));
	    param_nh.param<double>("map_resolution", resolution , 0.2);
	    param_nh.param<double>("subsample_level", subsample_level , 1);

	    fprintf(stderr,"USING RESOLUTION %lf\n",resolution);
	    
	    lslgeneric::NDTMap ndmap(new lslgeneric::LazyGrid(resolution));
	    ndmap.loadFromJFF(mapName.c_str());

	    //////////////////////////////////////////////////////////
	    /// Prepare MCL object 
	    //////////////////////////////////////////////////////////

	    ndtmcl = new NDTMCL3D(resolution,ndmap,-5);
	    param_nh.param<bool>("forceSIR", forceSIR, false);
	    if(forceSIR) ndtmcl->forceSIR=true;

	    fprintf(stderr,"*** FORCE SIR = %d****",forceSIR);
	    mcl_pub = nh_.advertise<nav_msgs::Odometry>("ndt_mcl",10);
	    
	    //////////////////////////////////////////////////////////
	    /// Prepare the callbacks and message filters
	    //////////////////////////////////////////////////////////
	    //the name of the TF link associated to the base frame / odometry frame    
	    param_nh.param<std::string>("tf_base_link", tf_base_link, std::string("/base_link"));
	    //the name of the tf link associated to the 3d laser scanner
	    param_nh.param<std::string>("tf_laser_link", tf_sensor_link, std::string("/velodyne_link"));
	    ///topic to wait for point clouds
	    param_nh.param<std::string>("points_topic",points_topic,"points");
	    ///topic to wait for odometry messages
	    param_nh.param<std::string>("odometry_topic",odometry_topic,"odometry");

	    points2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,points_topic,1);
	    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
	    sync_po_ = new message_filters::Synchronizer< PointsOdomSync >(PointsOdomSync(SYNC_FRAMES), *points2_sub_, *odom_sub_);
	    sync_po_->registerCallback(boost::bind(&NDTMCL3DNode::callback, this, _1, _2));

	    isFirstLoad=true;
	    pcounter =0;
	}
	~NDTMCL3DNode() {
	    delete points2_sub_;
	    delete odom_sub_;
	    delete sync_po_;
	    delete ndtmcl;
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
	    mrpt::utils::CTicTac	tictac;
	    
	    qd.x() = odo_in->pose.pose.orientation.x;
	    qd.y() = odo_in->pose.pose.orientation.y;
	    qd.z() = odo_in->pose.pose.orientation.z;
	    qd.w() = odo_in->pose.pose.orientation.w;

	    Todo = Eigen::Translation3d (odo_in->pose.pose.position.x,
		    odo_in->pose.pose.position.y,odo_in->pose.pose.position.z) * qd;
	    
	    pcl::fromROSMsg (*cloud_in, cloud);
	    
	    fprintf(stderr,"cloud has %d points\n",cloud.points.size());		
	    //check if sensor pose is known, if not, look it up on TF
	    if(!hasSensorPose) {

	    }
	    //check if we have done iterations 
	    if(isFirstLoad) {
		//if not, check if initial robot pose has been set
		if(!hasInitialPose) {
		    //can't do anything, wait for pose message...
		    mcl_m.unlock();
		    return;
		}
		//initialize filter
		Eigen::Vector3d tr = initPoseT.translation();
		Eigen::Vector3d rot = initPoseT.rotation().eulerAngles(0,1,2);
		
		Todo_old=Todo;
		Tcum = initPoseT;

		ndtmcl->initializeFilter(tr[0], tr[1],tr[2],rot[0],rot[1],rot[2],0.5, 0.5, 0.1, 2.0*M_PI/180.0, 2.0*M_PI/180.0 ,2.0*M_PI/180.0, 100);
		//ndt_viz.plotNDTMap(&ndtmcl->map,0,1.0,1.0,true, false); 
                ndt_viz.plotNDTSAccordingToOccupancy(-1,&ndtmcl->map);
		isFirstLoad = false;
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

	    tictac.Tic();	
	    //update filter -> + add parameter to subsample ndt map in filter step
	    ndtmcl->updateAndPredictEff(Tm, cloud, subsample_level);

	    double TimeElapsed = tictac.Tac();
	    fprintf(stderr,"Time elapsed %lfs\n",TimeElapsed);		

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
	    //publish pose
	    sendROSOdoMessage(ndtmcl->pf.getMean(),odo_in->header.stamp);
	    mcl_m.unlock();

	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	//FIXME: this should be in 3D
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool sendROSOdoMessage(Eigen::Affine3d mean,ros::Time ts){
	    nav_msgs::Odometry O;
	    static int seq = 0;
	    O.header.stamp = ts;
	    O.header.seq = seq;
	    O.header.frame_id = "/world";
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

	    return true;
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
};

//Getting rid of RVIZ visualization for now...

//TODO: this goes?
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Initial pose stuff (quick and dirty as always)
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#if 0
double ipos_x=0,ipos_y=0,ipos_yaw=0;
double ivar_x=0,ivar_y=0,ivar_yaw=0;

ros::Subscriber initial_pose_sub_;

void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
	tf::Pose ipose;
  tf::poseMsgToTF(msg->pose.pose, ipose);
  ipos_x = ipose.getOrigin().x();
  ipos_y = ipose.getOrigin().y();
  double pitch, roll;
  ipose.getBasis().getEulerYPR(ipos_yaw,pitch,roll);
  
  ivar_x = msg->pose.covariance[0];
  ivar_x = msg->pose.covariance[6];
  ivar_x = msg->pose.covariance[35];
  
  hasNewInitialPose=true;
}
#endif
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//new callback
#if 0
void callback(pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    static int pcounter = 0;
    mrpt::utils::CTicTac	tictac;
    static tf::TransformListener tf_listener;

    if(has_sensor_offset_set == false) return;
    double gx,gy,gyaw,x,y,yaw;

    ///Get state information
    tf::StampedTransform transform;

    ///Odometry 
    try{
	tf_listener.lookupTransform("world", tf_odo_topic, cloud->header.stamp, transform);
	yaw = tf::getYaw(transform.getRotation());  
	x = transform.getOrigin().x();
	y = transform.getOrigin().y();
    }
    catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	return;
    }
    
    Eigen::Affine3d Ttot,Ts,Tbase,Tgt;

    tf::TransformTFToEigen (sensor_pose, Ttot); ///< This is the velodyne pose (according to odometry)
    tf::TransformTFToEigen (T, Ts); ///<sensor offset
    tf::TransformTFToEigen (basepose, Tgt); ///<Ground truth

    ///Set the cloud to sensor frame with respect to base
    tf::TransformTFToEigen (baseodo, Tbase); ///<Odometry in vehicle frame

    if(counter == 0){

	cloud_old = cloud;
	Eigen::Vector3d tr = Tgt.translation();
	Eigen::Vector3d rot = Tgt.rotation().eulerAngles(0,1,2);
	Todo=Tgt;
	Tsold = Ttot;
	Tprev = Tgt;

	Told = Tbase;

	ndtmcl.initializeFilter(tr[0], tr[1],tr[2],rot[0],rot[1],rot[2],0.5, 0.5, 0.1, 2.0*M_PI/180.0, 2.0*M_PI/180.0 ,2.0*M_PI/180.0, 100);
	counter ++;

	ndt_viz.plotNDTMap(&ndtmcl.map,0,1.0,1.0,true, false);

	return;
    }
    counter++;

    Eigen::Affine3d Tmotion = Told.inverse()*Tbase;

    if(Tmotion.translation().norm()<0.01 && fabs(Tmotion.rotation().eulerAngles(0,1,2)[2])<(0.5*M_PI/180.0)) return;

    Tprev = Todo;
    cloud_old = cloud;
    Told = Tbase;

    ///Set the cloud to sensor frame with respect to base
    lslgeneric::transformPointCloudInPlace(Ts, cloud);

    tictac.Tic();	
    ndtmcl.updateAndPredictEff(Tmotion, cloud);

    double TimeElapsed = tictac.Tac();
    fprintf(stderr,"Time elapsed %lfs\n",TimeElapsed);		

    ///DRAW STUFF///
    pcounter++;
    if(pcounter%500==0){
	ndt_viz.clear();
	ndt_viz.plotNDTMap(&ndtmcl.map,0,1.0,1.0,true, false);

    }

    if(pcounter%10==0){

	ndt_viz.clearParticles();
	for(int i=0;i<ndtmcl.pf.size();i++){
	    double x,y,z;
	    ndtmcl.pf.pcloud[i].getXYZ(x,y,z);
	    ndt_viz.addParticle(x, y,z+0.5, 1.0, 1.0, 1.0);
	}

	Eigen::Affine3d mean = ndtmcl.pf.getMean();
	ndt_viz.addTrajectoryPoint(mean.translation()[0],mean.translation()[1],mean.translation()[2]+0.5,1.0,0,0);	
	Eigen::Vector3d tr = Tgt.translation();
	ndt_viz.addTrajectoryPoint(tr[0],tr[1],tr[2]+0.5,0.0,1.0,0);	

	ndt_viz.displayParticles();
	ndt_viz.displayTrajectory();
	ndt_viz.win3D->setCameraPointingToPoint(mean.translation()[0],mean.translation()[1],3.0);
	ndt_viz.repaint();
    }

}

mrpt::utils::CTicTac	TT;

std::string tf_odo_topic =   "odom_base_link";
std::string tf_state_topic = "base_link";
std::string tf_laser_link =  "base_laser_link";

/**
 * Callback for laser scan messages 
 */
void callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	static int counter = 0;
	counter++;
	
	static tf::TransformListener tf_listener;
	double looptime = TT.Tac();
	TT.Tic();
	fprintf(stderr,"Lt( %.1lfms %.1lfHz seq:%d) -",looptime*1000,1.0/looptime,scan->header.seq);
	
	if(has_sensor_offset_set == false) return;
	double gx,gy,gyaw,x,y,yaw;
	
	///Get state information
	tf::StampedTransform transform;
	tf_listener.waitForTransform("world", tf_state_topic, scan->header.stamp,ros::Duration(1.0));
	///Ground truth --- Not generally available so should be changed to the manual initialization
	try{
		tf_listener.lookupTransform("world", tf_state_topic, scan->header.stamp, transform);
		gyaw = tf::getYaw(transform.getRotation());  
	  gx = transform.getOrigin().x();
	  gy = transform.getOrigin().y();
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	
	///Odometry 
	try{
		tf_listener.lookupTransform("world", tf_odo_topic, scan->header.stamp, transform);
		yaw = tf::getYaw(transform.getRotation());  
	  x = transform.getOrigin().x();
	  y = transform.getOrigin().y();
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	
		
	mrpt::utils::CTicTac	tictac;
	tictac.Tic();
	
	///Number of scans
	int N =(scan->angle_max - scan->angle_min)/scan->angle_increment;
	/////
	/// Pose conversions
	////
	
	Eigen::Affine3d T = getAsAffine(x,y,yaw);
	Eigen::Affine3d Tgt = getAsAffine(gx,gy,gyaw);
	
	if(userInitialPose && hasNewInitialPose){
		gx = ipos_x;
		gy = ipos_y;
		gyaw = ipos_yaw;
	}
	
	if(isFirstLoad || hasNewInitialPose){
		fprintf(stderr,"Initializing to (%lf, %lf, %lf)\n",gx,gy,gyaw);
		/// Initialize the particle filter
		ndtmcl->initializeFilter(gx, gy,gyaw,0.2, 0.2, 2.0*M_PI/180.0, 150);
		Told = T;
		Todo = Tgt;
		hasNewInitialPose = false;
	}
	
	///Calculate the differential motion from the last frame
	Eigen::Affine3d Tmotion = Told.inverse() * T;
	Todo = Todo*Tmotion; ///< just integrates odometry for the visualization
	
	if(isFirstLoad==false){
		if( (Tmotion.translation().norm()<0.005 && fabs(Tmotion.rotation().eulerAngles(0,1,2)[2])<(0.2*M_PI/180.0))){
			Eigen::Vector3d dm = ndtmcl->getMean();
			Eigen::Matrix3d cov = ndtmcl->pf.getDistributionVariances();
			sendROSOdoMessage(dm,cov, scan->header.stamp);
			double Time = tictac.Tac();
			fprintf(stderr,"Time elapsed %.1lfms\n",Time*1000);
			return;
		}
	}
	
	Told = T;
	
	///Calculate the laser pose with respect to the base
	float dy =offy;
	float dx = offx;
	float alpha = atan2(dy,dx);
	float L = sqrt(dx*dx+dy*dy);
	
	///Laser pose in base frame
	float lpx = L * cos(alpha);
	float lpy = L * sin(alpha);
	float lpa = offa;
	
	///Laser scan to PointCloud expressed in the base frame
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);	
	for (int j=0;j<N;j++){
		double r  = scan->ranges[j];
		if(r>=scan->range_min && r<scan->range_max && r>0.3 && r<20.0){
			double a  = scan->angle_min + j*scan->angle_increment;
			pcl::PointXYZ pt;
			pt.x = r*cos(a+lpa)+lpx;
			pt.y = r*sin(a+lpa)+lpy;
			pt.z = 0.1+0.02 * (double)rand()/(double)RAND_MAX;
			cloud->push_back(pt);
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/// Now we have the sensor origin and pointcloud -- Lets do MCL
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	ndtmcl->updateAndPredict(Tmotion, *cloud); ///<predicts, updates and resamples if necessary (ndt_mcl.hpp)
	
	Eigen::Vector3d dm = ndtmcl->getMean(); ///Maximum aposteriori pose
	Eigen::Matrix3d cov = ndtmcl->pf.getDistributionVariances(); ///Pose covariance
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	double Time = tictac.Tac();
	fprintf(stderr,"Time elapsed %.1lfms (%lf %lf %lf) \n",Time*1000,dm[0],dm[1],dm[2]);
	isFirstLoad = false;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	sendROSOdoMessage(dm,cov, scan->header.stamp); ///Spit out the pose estimate
	
	///This is all for visualization
#ifdef USE_VISUALIZATION_DEBUG
	Eigen::Vector3d origin(dm[0] + L * cos(dm[2]+alpha),dm[1] + L * sin(dm[2]+alpha),0.1);
	Eigen::Affine3d ppos = getAsAffine(dm[0],dm[1],dm[2]);
	
	//lslgeneric::transformPointCloudInPlace(Tgt, *cloud);
	lslgeneric::transformPointCloudInPlace(ppos, *cloud);
	mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();	
	win3D.setCameraPointingToPoint(gx,gy,1);
	if(counter%2000==0) gl_points->clear();
	scene->clear();
	scene->insert(plane);
	
	addMap2Scene(ndtmcl->map, origin, scene);
	addPoseCovariance(dm[0],dm[1],cov,scene);
	addScanToScene(scene, origin, cloud);
	addParticlesToWorld(ndtmcl->pf,Tgt.translation(),dm, Todo.translation());
	scene->insert(gl_points);
	scene->insert(gl_particles);
	win3D.unlockAccess3DScene();
	win3D.repaint();

	if (win3D.keyHit())
	{
		mrpt::gui::mrptKeyModifier kmods;
		int key = win3D.getPushedKey(&kmods);
	}
#endif
}

#endif

int main(int argc, char **argv){
	ros::init(argc, argv, "NDT-MCL");
	ros::NodeHandle paramHandle ("~");
	NDTMCL3DNode mclnode(paramHandle);   	
	ros::spin();
	return 0;
}
