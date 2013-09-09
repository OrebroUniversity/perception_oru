
//mrpt stuff
#include <ndt_viz.h>
//Ros stuff
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ndt_map.h>
#include <ndt_cell.h>
#include <pointcloud_utils.h>
#include <tf_conversions/tf_eigen.h>
#include <VelodyneBagReader.h>


///Global map
double TEST_RESOLUTION = 0.4;
lslgeneric::NDTMap<pcl::PointXYZI> ndmap(new lslgeneric::LazyGrid<pcl::PointXYZI>(TEST_RESOLUTION));
NDTViz<pcl::PointXYZI> ndt_viz;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/// Callback
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
Eigen::Affine3d Told;
bool continous_plot_enabled = false;

void callback(pcl::PointCloud<pcl::PointXYZI> &cloud, tf::Transform &sensor_pose, tf::Transform &base_pose)
{
	static int counter = 0;
	static double tot = 0;
	static double tr = 0;
	static double dist=0;
	static int map_cnt = 0;

	Eigen::Affine3d Ttot,Tbase;
	tf::TransformTFToEigen (sensor_pose, Ttot);
	tf::TransformTFToEigen (base_pose, Tbase);
	

	if(counter == 0){
		 Told = Ttot;
		 ndmap.initialize(Ttot.translation()[0], Ttot.translation()[1], Ttot.translation()[2], 60.0, 60.0, 4.0);
	}
	 
	Eigen::Affine3d Tmotion = Told.inverse() * Ttot;
	counter++;
	
	if(Tmotion.translation().norm()<0.05) return;
	
	Told = Ttot;
	dist += Tmotion.translation().norm();
	
	
	Eigen::Vector3d m = Ttot.translation();
	lslgeneric::transformPointCloudInPlace<pcl::PointXYZI>(Ttot,cloud);
		
	mrpt::utils::CTicTac	tictac;
	
	tictac.Tic();
	
	Eigen::Vector3d localmapsize;
	localmapsize<<60.0,60.0,4.0;
	ndmap.addPointCloudMeanUpdate(Ttot.translation(),cloud, localmapsize, 1e5, 255 ,2.1, 0.25);
	
	double T = tictac.Tac();
	
	fprintf(stderr,"Time elapsed %lfs\n",T);		

	
	if(continous_plot_enabled && counter%1==0){
		ndt_viz.clear();
		//ndt_viz.plotLocalNDTMap(cloud, 0.4, 1.0, 1.0, 1.0, true); ///Plots the current observation
		ndt_viz.plotLocalConflictNDTMap(&ndmap,cloud, 0.4 ,1.0,0,0,false,2.0); ///Should plot the parts that conflict with the map
		ndt_viz.plotNDTMap(&ndmap, 0,1.0,1.0, true, true ); ///Plot the occupied section of the map
		//ndt_viz.plotNDTTraversabilityMap(&ndmap); ///Plots the traversability map
	}
	
}


/////////////////////////////////////////////////////////////////////////////////7
/////////////////////////////////////////////////////////////////////////////////7
/// *!!MAIN!!*
/////////////////////////////////////////////////////////////////////////////////7
/////////////////////////////////////////////////////////////////////////////////7
int main(int argc, char **argv){
	ros::Time::init();
	int loopN =argc-1;
	int cnt = 0;
	
	/// Set up the sensor link
	tf::StampedTransform sensor_link; ///Link from /odom_base_link -> velodyne
	
	sensor_link.child_frame_id_ = "/velodyne";
	sensor_link.frame_id_ = "/state_base_link";
	tf::Quaternion quat; 
	
	quat.setRPY(0,0,-M_PI/2.0 -2.6*M_PI/180.0 );
	tf::Vector3 trans(0.3,0,1.45);		
	
	tf::Transform T = tf::Transform(quat,trans);
	sensor_link.setData(T);
	
	Eigen::Affine3d Ts;
	//ndmap.setMapSize(60.0, 60.0, 4.0);
	
	
	while(cnt<loopN){
		cnt++;
		std::string bagfilename = argv[cnt];
		
		VelodyneBagReader vreader("../conf/kollmorgen_db.yaml", 
											bagfilename,
											"/velodyne_packets", 
											"/velodyne", 
											"/world",
											"/tf",
											ros::Duration(30),
											&sensor_link, 70.0,2.0); 
		
		pcl::PointCloud<pcl::PointXYZI> cloud;	
		tf::Transform sensor_pose, basepose;
		bool cameraset = false;
		int numclouds = 0;
	
		while(vreader.readMultipleMeasurements(1,cloud,sensor_pose,basepose,std::string("/state_base_link"))){
			if(cloud.size()==0) continue;
			callback(cloud, sensor_pose, basepose);	
			
			cloud.clear();
				
			numclouds++;
			
			if (ndt_viz.win3D->keyHit())
			{
				 mrpt::gui::mrptKeyModifier kmods;
				int key = ndt_viz.win3D->getPushedKey(&kmods);
				if (key=='g'){
					fprintf(stderr,"STARTING IMAGE GRAPPING\n");
					ndt_viz.win3D->grabImagesStart 	( std::string("ndtomg_")	);
				}else if(key == 's'){
					fprintf(stderr,"STOP IMAGE GRAPPING\n");
					ndt_viz.win3D->grabImagesStop(); 
				}else if(key == 'm'){
					fprintf(stderr,"Saving Map!\n");
					ndmap.writeToJFF("out.ndmap");
				}else if(key == 'e'){
					if(continous_plot_enabled){
						continous_plot_enabled = false;
						fprintf(stderr,"PLOTTING DISABLED\n");
					}else{
						continous_plot_enabled = true;
						fprintf(stderr,"PLOTTING ENABLED\n");
					}
				}else if(key == 'h'){
					fprintf(stderr,"\nOptions:\n");
					fprintf(stderr,"\t[g] Start grapping\n");
					fprintf(stderr,"\t[s] Stop grapping\n");
					fprintf(stderr,"\t[m] Save map to file (out.ndmap)\n");
					fprintf(stderr,"\t[e] Toggle continous plotting\n");
				}else{
					ndt_viz.clear();
					ndt_viz.plotNDTMap(&ndmap,1,0,0);
				}

			}
		}
		
	}
	ndmap.writeToJFF("out_final.ndmap");
	fprintf(stderr,"- END! -");
	ndt_viz.clear();
	ndt_viz.plotNDTMap(&ndmap,0,1.0,1.0,true);
	while(ndt_viz.win3D->isOpen()){
	
		usleep(1000*1000*2);
	}
	
}



