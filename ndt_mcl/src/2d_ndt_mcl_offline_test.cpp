
/**
* 2D NDT-MCL for offline processing (reads directly the bag files). 
* This application runs the ndt-mcl localization based on a map and laser scanner and odometry. 
* 
* This is based now on our sample file (see data folder). The sample file also has ground truth pose information, 
* which we use now for the initialization (replace with manual input if you will). 
* 
* The visualization depends on mrpt-gui 
* 
* More details about the algorithm:
* Jari Saarinen, Henrik Andreasson, Todor Stoyanov and Achim J. Lilienthal, Normal Distributions Transform Monte-Carlo Localization (NDT-MCL)
* IEEE/RSJ International Conference on Intelligent Robots and Systems November 3-8, 2013, Tokyo Big Sight, Japan
* 
* Known issues: in launch file (or hard coded parameters) you have to set the same resolution for NDT map as is saved -- otherwise it wont work
* @author Jari Saarinen (jari.p.saarinen@gmail.com)
* 
*/

#define USE_VISUALIZATION_DEBUG ///< Enable / Disable visualization
 
#include <mrpt/utils/CTicTac.h>

#ifdef USE_VISUALIZATION_DEBUG
	#include <mrpt/gui.h>	
	#include <mrpt/base.h>
	#include <mrpt/opengl.h>
	#include <GL/gl.h>
	#include "CMyEllipsoid.h"
#endif

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ndt_map.h>
#include <tf/transform_broadcaster.h>
#include "tfMessageReader.h"
#include "ndt_mcl.hpp"

#ifdef USE_VISUALIZATION_DEBUG
#include "mcl_visualization.hpp" ///< here is a punch of visualization code based on the MRPT's GUI components
#endif

/**
 * Convert x,y,yaw to Eigen::Affine3d 
 */ 
Eigen::Affine3d getAsAffine(float x, float y, float yaw ){
	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Translation3d v(x,y,0);
	Eigen::Affine3d T = v*m;
	
	return T;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Update measurement
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
NDTMCL<pcl::PointXYZ> *ndtmcl;

///Laser sensor offset
float offx = 0;
float offy = 0;
float offa = 0;

static bool has_sensor_offset_set = false;
static bool isFirstLoad=true;
Eigen::Affine3d Told,Todo;
FILE *flog = fopen("ndtmcl_result.txt","wt");

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
mrpt::utils::CTicTac	TT;

std::string tf_odo_topic =   "odom_base_link";
std::string tf_state_topic = "base_link";
std::string tf_laser_link =  "base_laser_link";

/**
 * This is where the localization is done... not really callback in the offline version
 */ 
void callback(const sensor_msgs::LaserScan &scan, tf::Transform odo_pose, tf::Transform state_pose)
{
	static int counter = 0;
	counter++;
	
	static tf::TransformListener tf_listener;
	
	double looptime = TT.Tac();
	TT.Tic();
	fprintf(stderr,"Lt( %.1lfms %.1lfHz seq:%d) -",looptime*1000,1.0/looptime,scan.header.seq);
	
	if(has_sensor_offset_set == false) return;
	
	double gx,gy,gyaw,x,y,yaw;
	
	///Get the ground truth
	gyaw = tf::getYaw(state_pose.getRotation());  
	gx = state_pose.getOrigin().x();
	gy = state_pose.getOrigin().y();
	
	///Get the odometry
	yaw = tf::getYaw(odo_pose.getRotation());  
	x = odo_pose.getOrigin().x();
	y = odo_pose.getOrigin().y();
		
	mrpt::utils::CTicTac	tictac;
	tictac.Tic();

	int N =(scan.angle_max - scan.angle_min)/scan.angle_increment; ///< number of scan lines
	
	/////
	/// Pose conversions
	////
	
	Eigen::Affine3d T = getAsAffine(x,y,yaw);
	Eigen::Affine3d Tgt = getAsAffine(gx,gy,gyaw);
	
	/**
	 * We are now using the information from the ground truth to initialize the filter
	 */ 
	if(isFirstLoad){
		fprintf(stderr,"Initializing to (%lf, %lf, %lf)\n",gx,gy,gyaw);
		///Initialize the filter with 1m^2 variance in position and 20deg in heading
		ndtmcl->initializeFilter(gx, gy,gyaw,1.0, 1.0, 20.0*M_PI/180.0, 450);
		Told = T;
		Todo = Tgt;
	}
	
	///Compute the differential motion between two time steps 
	Eigen::Affine3d Tmotion = Told.inverse() * T;
	Todo = Todo*Tmotion;
	Told = T;
	
	///Get the laser pose with respect to the base
	float dy =offy;
	float dx = offx;
	float alpha = atan2(dy,dx);
	float L = sqrt(dx*dx+dy*dy);
	
	///Laser pose in base frame
	float lpx = L * cos(alpha);
	float lpy = L * sin(alpha);
	float lpa = offa;
	
	
	
	///Laser scan to PointCloud transformed with respect to the base
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);	
	for (int j=0;j<N;j++){
		double r  = scan.ranges[j];
		if(r>=scan.range_min && r<scan.range_max && r>0.3 && r<20.0){
			double a  = scan.angle_min + j*scan.angle_increment;
			pcl::PointXYZ pt;
			pt.x = r*cos(a+lpa)+lpx;
			pt.y = r*sin(a+lpa)+lpy;
			pt.z = 0.1+0.02 * (double)rand()/(double)RAND_MAX;
			cloud->push_back(pt);
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/// Now we have the differential motion and pointcloud -- Lets do MCL
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	ndtmcl->updateAndPredict(Tmotion, *cloud); ///< does the prediction, update and resampling steps (see ndt_mcl.hpp)
	
	Eigen::Vector3d dm = ndtmcl->getMean(); ///<Maximum aposteriori estimate of the pose
	Eigen::Matrix3d cov = ndtmcl->pf.getDistributionVariances(); ///distribution variances
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	double Time = tictac.Tac();
	fprintf(stderr,"Time elapsed %.1lfms (%lf %lf %lf) \n",Time*1000,dm[0],dm[1],dm[2]);
	isFirstLoad = false;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//If visualization desired
#ifdef USE_VISUALIZATION_DEBUG
	///The sensor pose in the global frame for visualization
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

int main(int argc, char **argv){
	ros::init(argc, argv, "sauna_mcl");
	double resolution=0.2;
#ifdef USE_VISUALIZATION_DEBUG	
	initializeScene();
#endif
	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::NodeHandle paramHandle ("~");
	TT.Tic();
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	/// Parameters for the mapper
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	bool loadMap = false; ///< flag to indicate that we want to load a map
	std::string mapName("basement.ndmap"); ///<name and the path to the map
	
	bool makeMapVeryStatic = false; ///< indicates if we should make the map over confident (stationary that is)
	
	bool saveMap = true;						///< indicates if we want to save the map in a regular intervals
	std::string output_map_name = std::string("ndt_mapper_output.ndmap");
	
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	/// Set the values from a config file
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	
	std::string input_laser_topic;
	paramHandle.param<std::string>("input_laser_topic", input_laser_topic, std::string("/base_scan"));
	
	paramHandle.param<std::string>("tf_base_link", tf_state_topic, std::string("/state_base_link"));
	paramHandle.param<std::string>("tf_laser_link", tf_laser_link, std::string("/hokuyo1_link"));
	
	bool use_sensor_pose;
	paramHandle.param<bool>("use_sensor_pose", use_sensor_pose, false);
	double sensor_pose_x, sensor_pose_y, sensor_pose_th;
	paramHandle.param<double>("sensor_pose_x", sensor_pose_x, 0.);
	paramHandle.param<double>("sensor_pose_y", sensor_pose_y, 0.);
	paramHandle.param<double>("sensor_pose_th", sensor_pose_th, 0.);
	
	paramHandle.param<bool>("load_map_from_file", loadMap, false);
	paramHandle.param<std::string>("map_file_name", mapName, std::string("basement.ndmap"));

	paramHandle.param<bool>("save_output_map", saveMap, true);
	paramHandle.param<std::string>("output_map_file_name", output_map_name, std::string("ndt_mapper_output.ndmap"));
	
	paramHandle.param<double>("map_resolution", resolution , 0.2);
	bool forceSIR=false;
	paramHandle.param<bool>("forceSIR", forceSIR, false);
	
	std::string bagfilename="bagfile_unset.bag";
	paramHandle.param<std::string>("bagfile_name", bagfilename, std::string("bagfile_unset.bag"));
	
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	/// Prepare the map
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	fprintf(stderr,"USING RESOLUTION %lf\n",resolution);
	lslgeneric::NDTMap<pcl::PointXYZ> ndmap(new lslgeneric::LazyGrid<pcl::PointXYZ>(resolution));

	ndmap.setMapSize(80.0, 80.0, 1.0);
	
	if(loadMap){
		fprintf(stderr,"Loading Map from '%s'\n",mapName.c_str());
		ndmap.loadFromJFF(mapName.c_str());
	}

	ndtmcl = new NDTMCL<pcl::PointXYZ>(resolution,ndmap,-0.5);
	if(forceSIR) ndtmcl->forceSIR=true;

	fprintf(stderr,"*** FORCE SIR = %d****",forceSIR);
	
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	/// Open the bag file for reading
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	
	tfMessageReader<sensor_msgs::LaserScan> reader(bagfilename, 
																								 input_laser_topic, 
																								 std::string("/world"), 
																								 tf_odo_topic);

	///Set sensor offsets
	offa = sensor_pose_th;
	offx = sensor_pose_x;
	offy = sensor_pose_y;
	has_sensor_offset_set = true;
	
	fprintf(stderr,"Sensor Pose = (%lf %lf %lf)\n",offx, offy, offa);	
	
	///Loop while we have data in the bag
	while(!reader.bagEnd()){
		sensor_msgs::LaserScan s;
		tf::Transform odo_pose;
		bool hasOdo = false;
		bool hasState = false;
		
		if(reader.getNextMessage(s,  odo_pose)){
			hasOdo = true;
		}
		
		tf::Transform state_pose;
	
		if(reader.getTf(tf_state_topic, s.header.stamp, state_pose)){
			hasState = true;	
		}
		
		///If we have the data then lets run the localization
		if(hasState && hasOdo){
			 callback(s,odo_pose,state_pose);
		}
	}

	fprintf(stderr,"-- THE END --\n");
	
	return 0;
}
