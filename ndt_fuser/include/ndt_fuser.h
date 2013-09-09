#ifndef NDT_FUSER_HH
#define NDT_FUSER_HH

#include <ndt_viz.h>
#include <ndt_map.h>
#include <ndt_matcher_d2d_2d.h>
#include <ndt_matcher_d2d.h>
#include <pointcloud_utils.h>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>

namespace lslgeneric {
/**
  * \brief This class fuses new point clouds into a common ndt map reference, keeping tack of the 
  * camera postion.
  * \author Jari, Todor
  */
template <typename PointT>
class NDTFuser{
    public:
	Eigen::Affine3d Tnow, Tlast_fuse; ///< current pose
	lslgeneric::NDTMap<PointT> *map;  ///< da map
	bool checkConsistency; ///perform a check for consistency against initial estimate
	double max_translation_norm, max_rotation_norm;
	double sensor_range;
	bool be2D;
	NDTViz<PointT> *viewer;

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
	    viewer = new NDTViz<PointT>(visualize);
	    std::cout<<"MAP: resolution: "<<resolution<<" size "<<map_size_x<<" "<<map_size_y<<" "<<map_size_z<<std::endl;
	    map = new lslgeneric::NDTMap<PointT>(new lslgeneric::LazyGrid<PointT>(resolution));
	}
	~NDTFuser()
	{
	    delete map;
	    delete viewer;
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
	void initialize(Eigen::Affine3d initPos, pcl::PointCloud<PointT> &cloud){
	    std::cout<<"Initializing NDT FUSER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
	    ///Set the cloud to sensor frame with respect to base
	    lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
	    lslgeneric::transformPointCloudInPlace(initPos, cloud);
	    Tnow = initPos;
	    map->initialize(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),map_size_x,map_size_y,map_size_z);
	    map->addPointCloud(Tnow.translation(),cloud, 0.1, 100.0, 0.1);
	    map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tnow.translation(), 0.1);
	    isInit = true;
	    Tlast_fuse = Tnow;
	    if(visualize) 
	    {
		viewer->plotNDTSAccordingToOccupancy(-1,map); 
		viewer->plotLocalNDTMap(cloud,resolution); 
	    }
	}

	/**
	 *
	 *
	 */
	Eigen::Affine3d update(Eigen::Affine3d Tmotion, pcl::PointCloud<PointT> &cloud){
	    if(!isInit){
		fprintf(stderr,"NDT-Fuser: Call Initialize first!!\n");
		return Tnow;
	    }
	    double t1,t2,t3,t4;
	    ///Set the cloud to sensor frame with respect to base
	    lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
	    t1 = getDoubleTime();
	    ///Create local map
	    lslgeneric::NDTMap<PointT> ndlocal(new lslgeneric::LazyGrid<PointT>(resolution));
	    //ndlocal.guessSize(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),sensor_range,sensor_range,sensor_range);
	    ndlocal.addPointCloudSimple(cloud);
	    ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
	    t2 = getDoubleTime();
	    t3 = t4 = t2;
	    //Tmotion.setIdentity();        
	    Eigen::Affine3d Tinit = Tnow * Tmotion;
	    if(be2D) {
		if(matcher2D.match( *map, ndlocal,Tinit,true)){
		    Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * Tinit;
		    if((diff.translation().norm() > max_translation_norm || 
				diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm) && checkConsistency){
			fprintf(stderr,"****  NDTFuser -- ALMOST DEFINATELY A REGISTRATION FAILURE *****\n");
			Tnow = Tnow * Tmotion;
		    }else{
			Tnow = Tinit;
			lslgeneric::transformPointCloudInPlace(Tnow, cloud);
			Eigen::Affine3d spose = Tnow*sensor_pose;
			Eigen::Affine3d diff_fuse = Tlast_fuse.inverse()*Tnow;

			if(diff_fuse.translation().norm() > translation_fuse_delta ||
				diff_fuse.rotation().eulerAngles(0,1,2).norm() > rotation_fuse_delta)
			{
			    t3 = getDoubleTime();
			    map->addPointCloud(spose.translation(),cloud, 0.06, 2.5);
			    t4 = getDoubleTime();
			    map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);
			    Tlast_fuse = Tnow;
			    if(visualize) 
			    {
				viewer->plotNDTSAccordingToOccupancy(-1,map); 
				viewer->plotLocalNDTMap(cloud,resolution); 
			    }
			}
		    }
		    /*
		    std::cout<<"load: "<<t2-t1<<" match: "<<t3-t2<<" fuse: "<<t4-t3<<" total: "<<t4-t1<<std::endl;
		    FILE *ftmp = fopen("tmp.txt","a");
		    fprintf(ftmp,"%lf, ",t4-t3);
		    fclose(ftmp);*/
		}else{
		    Tnow = Tnow * Tmotion;
		}

	    }
	    else
	    {
		if(matcher.match( *map, ndlocal,Tinit,true)){
		    Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * Tinit;
		    if((diff.translation().norm() > max_translation_norm || 
				diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm) && checkConsistency){
			fprintf(stderr,"****  NDTFuser -- ALMOST DEFINATELY A REGISTRATION FAILURE *****\n");
			Tnow = Tnow * Tmotion;
			//save offending map:
			//map->writeToJFF("map.jff");
			//ndlocal.writeToJFF("local.jff");
		    }else{
			Tnow = Tinit;
			lslgeneric::transformPointCloudInPlace(Tnow, cloud);
			std::cout<<"Tnow :" << Tnow.matrix()<<std::endl;
			Eigen::Affine3d spose = Tnow*sensor_pose;
			Eigen::Affine3d diff_fuse = Tlast_fuse.inverse()*Tnow;

			if(diff_fuse.translation().norm() > translation_fuse_delta ||
				diff_fuse.rotation().eulerAngles(0,1,2).norm() > rotation_fuse_delta)
			{
			    t3 = getDoubleTime();
			    map->addPointCloud(spose.translation(),cloud, 0.06, 2.5);
			    t4 = getDoubleTime();
			    map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);
			    Tlast_fuse = Tnow;
			    if(visualize) 
			    {
				viewer->plotNDTSAccordingToOccupancy(-1,map); 
				viewer->plotLocalNDTMap(cloud,resolution); 
			    }

			}
		    }
		    std::cout<<"load: "<<t2-t1<<" match: "<<t3-t2<<" fuse: "<<t4-t3<<" total: "<<t4-t1<<std::endl;
		    /*FILE *ftmp = fopen("tmp.txt","a");
		    fprintf(ftmp,"%lf, ",t4-t3);
		    fclose(ftmp);*/
		}else{
		    Tnow = Tnow * Tmotion;
		}
	    }
	    
	    return Tnow;
	}

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
	lslgeneric::NDTMatcherD2D<PointT,PointT> matcher;
	lslgeneric::NDTMatcherD2D_2D<PointT,PointT> matcher2D;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
}
#endif
