#ifndef NDT_FUSER_HMT_HH
#define NDT_FUSER_HMT_HH
#ifndef NO_NDT_VIZ
#include <ndt_visualisation/ndt_viz.h>
#endif
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_map_hmt.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_map/pointcloud_utils.h>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <sys/time.h>

//#define BASELINE

namespace lslgeneric {
/**
  * \brief This class fuses new point clouds into a common ndt map reference, keeping tack of the 
  * camera postion.
  * \author Jari, Todor
  */
class NDTFuserHMT{
    public:
	Eigen::Affine3d Tnow, Tlast_fuse, Todom; ///< current pose
	lslgeneric::NDTMap *map;		 ///< da map
	bool checkConsistency;			 ///perform a check for consistency against initial estimate
	double max_translation_norm, max_rotation_norm;
	double sensor_range;
	bool be2D, doMultires, fuseIncomplete, beHMT, disableRegistration;
	int ctr;
	std::string prefix;
	std::string hmt_map_dir;
#ifndef NO_NDT_VIZ
	NDTViz *viewer;
#endif
	FILE *fAddTimes, *fRegTimes;

	NDTFuserHMT(double map_resolution, double map_size_x_, double map_size_y_, double map_size_z_, double sensor_range_ = 3, 
		    bool visualize_=false, bool be2D_=false, bool doMultires_=false, bool fuseIncomplete_=false, int max_itr=30, 
		    std::string prefix_="", bool beHMT_=true, std::string hmt_map_dir_="map", bool _step_control=true){
	    isInit = false;
	    disableRegistration=false;
	    resolution = map_resolution;
	    sensor_pose.setIdentity();
	    checkConsistency = false;
	    visualize = true;
	    translation_fuse_delta = 0.0;
	    rotation_fuse_delta = 0.0;
	    //translation_fuse_delta = 0.05;
	    //rotation_fuse_delta = 0.01;
	    max_translation_norm = 1.;
	    max_rotation_norm = M_PI/4;
	    map_size_x = map_size_x_;
	    map_size_y = map_size_y_;
	    map_size_z = map_size_z_;
	    visualize = visualize_;
	    be2D = be2D_;
	    sensor_range = sensor_range_;
	    prefix = prefix_;
	    doMultires = doMultires_;
	    ctr =0;
#ifndef NO_NDT_VIZ
        if(visualize_){
          viewer = new NDTViz(visualize);
          viewer->win3D->start_main_loop_own_thread(); // Very very ugly to start it here... FIX ME.
        }
#endif
	    localMapSize<<sensor_range_,sensor_range_,map_size_z_;
	    fuseIncomplete = fuseIncomplete_;
	    matcher.ITR_MAX = max_itr;
	    matcher2D.ITR_MAX = max_itr;
	    matcher.step_control=_step_control;
	    matcher2D.step_control=_step_control;
	    beHMT = beHMT_;
	    hmt_map_dir=hmt_map_dir_;

	    
	    char fname[1000];
	    snprintf(fname,999,"%s_addTime.txt",prefix.c_str());
	    fAddTimes = fopen(fname,"w");

	    std::cout<<"MAP: resolution: "<<resolution<<" size "<<map_size_x<<" "<<map_size_y<<" "<<map_size_z<<" sr "<<sensor_range<<std::endl;
	}
	~NDTFuserHMT()
	{
	    delete map;
#ifndef NO_NDT_VIZ
	    delete viewer;
#endif
	    if(fAddTimes!=NULL) fclose(fAddTimes);
	    if(fRegTimes!=NULL) fclose(fRegTimes);
	}

	double getDoubleTime()
	{
	    struct timeval time;
	    gettimeofday(&time,NULL);
	    return time.tv_sec + time.tv_usec * 1e-6;
	}
	void setSensorPose(Eigen::Affine3d spose){
	    sensor_pose = spose;
	}
	
	bool wasInit()
	{
	    return isInit;
	}

	bool saveMap() {
	    if(!isInit) return false;
	    if(map == NULL) return false;
	    if(beHMT) {
		lslgeneric::NDTMapHMT *map_hmt = dynamic_cast<lslgeneric::NDTMapHMT*> (map);
		if(map_hmt==NULL) return false;
		return (map_hmt->writeTo()==0);
	    } else {
		char fname[1000];
		snprintf(fname,999,"%s/%s_map.jff",hmt_map_dir.c_str(),prefix.c_str());
		return (map->writeToJFF(fname) == 0);
	    }
	}

	/**
	 * Set the initial position and set the first scan to the map
	 */
	void initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ> &cloud, bool preLoad=false);

	/**
	 *
	 *
	 */
	Eigen::Affine3d update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud);

    private:
	bool isInit;

	double resolution; ///< resolution of the map
	double map_size;
	
	double translation_fuse_delta, rotation_fuse_delta;
	double map_size_x;
	double map_size_y;
	double map_size_z;
	bool visualize;

	Eigen::Affine3d sensor_pose;
	lslgeneric::NDTMatcherD2D matcher;
	lslgeneric::NDTMatcherD2D_2D matcher2D;
	Eigen::Vector3d localMapSize;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
}
#endif
