#ifndef NDT_MCL_HPP_
#define NDT_MCL_HPP_
//#include <pcl/ros/conversions.h> deprecated
#include <pcl/conversions.h> 
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf_conversions/tf_eigen.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>

#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/pointcloud_utils.h>

#include "ndt_mcl/CParticleFilter.h"
/**
* NDT MCL - Class implementation
*/
class NDTMCL{
    public:
	lslgeneric::NDTMap map; 		///<This is my map 
	mcl::CParticleFilter pf; 						///<This is the particle filter
	double resolution;
	int counter;
	double zfilt_min;
	bool forceSIR;
	/**
	 * Constructor
	 */
	NDTMCL(double map_resolution, lslgeneric::NDTMap &nd_map, double zfilter):
	    map(new lslgeneric::LazyGrid(map_resolution))
	{
	    isInit = false;
	    forceSIR = false;
	    resolution=map_resolution;
	    counter = 0;
	    zfilt_min = zfilter;
	    sinceSIR = 0;
	    ///First, lets make our target map match the given map
	    ///This is done because we want (possibly) lower resolution target map
	    double cx,cy,cz;
	    if(!nd_map.getCentroid(cx, cy, cz)){
		fprintf(stderr,"Centroid NOT Given-abort!\n");
		exit(1);
	    }else{
		fprintf(stderr,"Centroid(%lf,%lf,%lf)\n",cx,cy,cz);
	    }

	    double wx,wy,wz;

	    if(!nd_map.getGridSizeInMeters(wx, wy, wz)){
		fprintf(stderr,"Grid size NOT Given-abort!\n");
		exit(1);
	    }else{
		fprintf(stderr,"GridSize(%lf,%lf,%lf)\n",wx,wy,wz);
	    }

	    map.initialize(cx,cy,cz,wx,wy,wz);

	    std::vector<lslgeneric::NDTCell*> ndts;
	    ndts = nd_map.getAllCells();
	    fprintf(stderr,"NDT MAP with %zu components",ndts.size());
	    for(unsigned int i=0;i<ndts.size();i++){
		Eigen::Vector3d m = ndts[i]->getMean();	
		if(m[2]>zfilter){
		    Eigen::Matrix3d cov = ndts[i]->getCov();
		    unsigned int nump = ndts[i]->getN();
		    //fprintf(stderr,"-%d-",nump);
		    map.addDistributionToCell(cov, m,nump);
		}
	    }
	}

	/**
	 * Intialize filter to some pose and with some number of particles 
	 *
	 */
	void initializeFilter(double x, double y, double yaw,double x_e, double y_e, double yaw_e, int numParticles){
	    pf.initializeNormalRandom(mcl::pose(x,y,yaw), mcl::pose(x_e,y_e,yaw_e),numParticles);
	}

	void updateAndPredict(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud); 

	Eigen::Vector3d getMean(){
	    mcl::pose m = pf.getDistributionMean(true);
	    Eigen::Vector3d mm; mm<<m.x,m.y,m.a;
	    return mm;
	}


    private:
	bool isInit;
	int sinceSIR;
	Eigen::Affine3d getAsAffine(int i){
	    Eigen::Matrix3d m;
	    m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(pf.Particles[i].a, Eigen::Vector3d::UnitZ());
	    Eigen::Translation3d v(pf.Particles[i].x,pf.Particles[i].y,0);
	    Eigen::Affine3d T = v*m;
	    return T;
	}

};

#endif

