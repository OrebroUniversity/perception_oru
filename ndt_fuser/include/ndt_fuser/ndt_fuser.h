#ifndef NDT_FUSER_HH
#define NDT_FUSER_HH
#ifndef NO_NDT_VIZ
#include <ndt_visualisation/ndt_viz.h>
#endif
#include <ndt_map/ndt_map.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_map/pointcloud_utils.h>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <sys/time.h>

namespace lslgeneric {
/**
  * \brief This class fuses new point clouds into a common ndt map reference, keeping tack of the 
  * camera postion.
  * \author Jari, Todor
  */
class NDTFuser{
    public:
	Eigen::Affine3d Tnow, Tlast_fuse; ///< current pose
	lslgeneric::NDTMap *map;  ///< da map
	bool checkConsistency; ///perform a check for consistency against initial estimate
	double max_translation_norm, max_rotation_norm;
	double sensor_range;
	bool be2D;
#ifndef NO_NDT_VIZ
	NDTViz *viewer;
#endif
	NDTFuser(double map_resolution, double map_size_x_, double map_size_y_, double map_size_z_, double sensor_range_ = 3, bool visualize_=false, bool be2D_=false){
	    isInit = false;
	    resolution = map_resolution;
	    sensor_pose.setIdentity();
	    checkConsistency = false;
	    visualize = true;
	    translation_fuse_delta = 0.05;
	    rotation_fuse_delta = 0.01;
	    max_translation_norm = 1.;
	    max_rotation_norm = M_PI/4;
	    map_size_x = map_size_x_;
	    map_size_y = map_size_y_;
	    map_size_z = map_size_z_;
	    visualize = visualize_;
	    be2D = be2D_;
	    sensor_range = sensor_range_;
#ifndef NO_NDT_VIZ
	    viewer = new NDTViz(visualize);
#endif
	    std::cout<<"MAP: resolution: "<<resolution<<" size "<<map_size_x<<" "<<map_size_y<<" "<<map_size_z<<std::endl;
	    map = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution));
	}
	~NDTFuser()
	{
	    delete map;
#ifndef NO_NDT_VIZ
	    delete viewer;
#endif
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

	/**
	 * Set the initial position and set the first scan to the map
	 */
	void initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ> &cloud);
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

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
}
#endif
