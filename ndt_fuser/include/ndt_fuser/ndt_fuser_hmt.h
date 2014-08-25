#ifndef NDT_FUSER_HMT_HH
#define NDT_FUSER_HMT_HH

#include <ndt_visualisation/ndt_viz.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_map_hmt.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>
#include <ndt_registration/ndt_matcher_d2d.h>
#include <pointcloud_vrml/pointcloud_utils.h>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>

//#define BASELINE

namespace lslgeneric {
/**
  * \brief This class fuses new point clouds into a common ndt map reference, keeping tack of the 
  * camera postion.
  * \author Jari, Todor
  */
template <typename PointT>
class NDTFuserHMT{
    public:
	Eigen::Affine3d Tnow, Tlast_fuse, Todom; ///< current pose
	//lslgeneric::NDTMapHMT<PointT> *map;  ///< da map
	lslgeneric::NDTMap<PointT> *map;  ///< da map
	bool checkConsistency;		  ///perform a check for consistency against initial estimate
	double max_translation_norm, max_rotation_norm;
	double sensor_range;
	bool be2D, doMultires, fuseIncomplete, beHMT;
	int ctr;
	std::string prefix;
	std::string hmt_map_dir;
	NDTViz<PointT> *viewer;
	FILE *fAddTimes, *fRegTimes;

	NDTFuserHMT(double map_resolution, double map_size_x_, double map_size_y_, double map_size_z_, double sensor_range_ = 3, 
		    bool visualize_=false, bool be2D_=false, bool doMultires_=false, bool fuseIncomplete_=false, int max_itr=30, 
		    std::string prefix_="", bool beHMT_=true, std::string hmt_map_dir_="map", bool _step_control=true){
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
	    prefix = prefix_;
	    doMultires = doMultires_;
	    ctr =0;
	    viewer = new NDTViz<PointT>(visualize);
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
	    delete viewer;
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
		lslgeneric::NDTMapHMT<PointT> *map_hmt = dynamic_cast<lslgeneric::NDTMapHMT<PointT>*> (map);
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
	void initialize(Eigen::Affine3d initPos, pcl::PointCloud<PointT> &cloud, bool preLoad=false){
	    ///Set the cloud to sensor frame with respect to base
	    lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
	    lslgeneric::transformPointCloudInPlace(initPos, cloud);
	    Tnow = initPos;
//#ifdef BASELINE
//#else
	    if(beHMT) {
		map = new lslgeneric::NDTMapHMT<PointT>(resolution,
			Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),
			map_size_x,map_size_y,map_size_z,sensor_range,hmt_map_dir,true);
		if(preLoad) {
		    lslgeneric::NDTMapHMT<PointT> *map_hmt = dynamic_cast<lslgeneric::NDTMapHMT<PointT>*> (map);
		    std::cout<<"Trying to pre-load maps at "<<initPos.translation()<<std::endl;
		    map_hmt->tryLoadPosition(initPos.translation());
		}
	    } else {
		map = new lslgeneric::NDTMap<PointT>(new lslgeneric::LazyGrid<PointT>(resolution));
		map->initialize(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),map_size_x,map_size_y,map_size_z);
	    }
//#endif
	    map->addPointCloud(Tnow.translation(),cloud, 0.1, 100.0, 0.1);
	    //map->addPointCloudMeanUpdate(Tnow.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);
	    //map->addPointCloudMeanUpdate(Tnow.translation(),cloud,localMapSize, 0.1, 100.0, 0.1);
	    map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tnow.translation(), 0.1);
	    isInit = true;
	    Tlast_fuse = Tnow;
	    Todom = Tnow;
	    if(visualize) 
	    {
		viewer->plotNDTSAccordingToOccupancy(-1,map); 
		viewer->plotLocalNDTMap(cloud,resolution);
	        viewer->addTrajectoryPoint(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+0.2,1,0,0);
	        viewer->addTrajectoryPoint(Todom.translation()(0),Todom.translation()(1),Todom.translation()(2)+0.2,0,1,0);
		viewer->displayTrajectory();
		viewer->setCameraPointing(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+3);
		viewer->repaint();	
	    }
	}

	/**
	 *
	 *
	 */
	Eigen::Affine3d update(Eigen::Affine3d Tmotion, pcl::PointCloud<PointT> &cloud){
	    if(!isInit){
		fprintf(stderr,"NDT-FuserHMT: Call Initialize first!!\n");
		return Tnow;
	    }
	    Todom = Todom * Tmotion; //we track this only for display purposes!
	    double t0=0,t1=0,t2=0,t3=0,t4=0,t5=0,t6=0;
	    ///Set the cloud to sensor frame with respect to base
	    lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
	    t0 = getDoubleTime();
	    ///Create local map
	    lslgeneric::NDTMap<PointT> ndlocal(new lslgeneric::LazyGrid<PointT>(resolution));
	    ndlocal.guessSize(0,0,0,sensor_range,sensor_range,map_size_z);
	    ndlocal.loadPointCloud(cloud,sensor_range);
	    ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
	    //pass through ndlocal and set all cells with vertically pointing normals to non-gaussian :-O
	    /*SpatialIndex<PointT> *index = ndlocal.getMyIndex();
	    typename SpatialIndex<PointT>::CellVectorItr it = index->begin();
	    while (it != index->end())
	    {
		NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
		if(cell!=NULL)
		{
		    if(cell->hasGaussian_)
		    {
			if(cell->getClass() == NDTCell<PointT>::HORIZONTAL) {
			    cell->hasGaussian_ = false;
			}
		    }
		}
		it++;
	    }*/

	    t1 = getDoubleTime();
	    Eigen::Affine3d Tinit = Tnow * Tmotion;
	    if(doMultires) {
		//create two ndt maps with resolution = 3*resolution (or 5?)
		lslgeneric::NDTMap<PointT> ndlocalLow(new lslgeneric::LazyGrid<PointT>(3*resolution));
		ndlocalLow.guessSize(0,0,0,sensor_range,sensor_range,map_size_z);
		ndlocalLow.loadPointCloud(cloud,sensor_range);
		ndlocalLow.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

		lslgeneric::NDTMap<PointT> mapLow(new lslgeneric::LazyGrid<PointT>(3*resolution));
		//add distros
		double cx,cy,cz;
		if(!map->getCentroid(cx, cy, cz)){
		    fprintf(stderr,"Centroid NOT Given-abort!\n");
		}
		mapLow.initialize(cx,cy,cz,3*map_size_x,3*map_size_y,map_size_z);

		std::vector<lslgeneric::NDTCell<PointT>*> ndts;
		ndts = map->getAllCells(); //this copies cells?
	
		for(int i=0; i<ndts.size(); i++)	
		{
		    NDTCell<PointT> *cell = ndts[i];
		    if(cell!=NULL)
		    {
			if(cell->hasGaussian_)
			{
			    Eigen::Vector3d m = cell->getMean();	
			    Eigen::Matrix3d cov = cell->getCov();
			    unsigned int nump = cell->getN();
			    mapLow.addDistributionToCell(cov, m,nump);
			}
		    }
		    delete cell;
		}
		//do match
		if(matcher2D.match( mapLow, ndlocalLow,Tinit,true)){
		    //if success, set Tmotion to result
		    t2 = getDoubleTime();
		    //std::cout<<"success: new initial guess! t= "<<t2-t1<<std::endl;
		} else {
		    Tinit = Tnow * Tmotion;
		}	    
	    }
	    
	    if(be2D) {
		t2 = getDoubleTime();
		if(matcher2D.match( *map, ndlocal,Tinit,true) || fuseIncomplete){
		    t3 = getDoubleTime();
		    Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * Tinit;
		    if((diff.translation().norm() > max_translation_norm || 
				diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm) && checkConsistency){
			fprintf(stderr,"****  NDTFuserHMT -- ALMOST DEFINATELY A REGISTRATION FAILURE *****\n");
			Tnow = Tnow * Tmotion;
		    }else{
			Tnow = Tinit;
			lslgeneric::transformPointCloudInPlace(Tnow, cloud);
			Eigen::Affine3d spose = Tnow*sensor_pose;
			Eigen::Affine3d diff_fuse = Tlast_fuse.inverse()*Tnow;
			if(diff_fuse.translation().norm() > translation_fuse_delta ||
				diff_fuse.rotation().eulerAngles(0,1,2).norm() > rotation_fuse_delta)
			{
			    //std::cout<<"F: "<<spose.translation().transpose()<<" "<<spose.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
			    t4 = getDoubleTime();
			    map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);
			    t5 = getDoubleTime();
			    //map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 25, 2*map_size_z, 0.06);
			    //map->addPointCloud(spose.translation(),cloud, 0.06, 25);
			    //map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);
			    //t4 = getDoubleTime();
			    //std::cout<<"match: "<<t3-t2<<" addPointCloud: "<<t5-t4<<" ndlocal "<<t1-t0<<" total: "<<t5-t0<<std::endl;
			    Tlast_fuse = Tnow;
			    if(visualize) //&&ctr%20==0) 
			    {
				if(ctr%20==0) {
				    viewer->plotNDTSAccordingToOccupancy(-1,map); 
				    viewer->plotLocalNDTMap(cloud,resolution); 
				}
				viewer->addTrajectoryPoint(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),1,0,0);
				viewer->addTrajectoryPoint(Todom.translation()(0),Todom.translation()(1),Todom.translation()(2),0,1,0);
				viewer->displayTrajectory();
				viewer->setCameraPointing(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+3);
				viewer->repaint();	
			    }
			    ctr++;
			}
		    }
		}else{
		    t3 = getDoubleTime();
		    Tnow = Tnow * Tmotion;
		}

	    }
	    else
	    {
		
		t2 = getDoubleTime();
		if(matcher.match( *map, ndlocal,Tinit,true) || fuseIncomplete){
		    t3 = getDoubleTime();
		    Eigen::Affine3d diff = (Tnow * Tmotion).inverse() * Tinit;
		    
		    if((diff.translation().norm() > max_translation_norm || 
				diff.rotation().eulerAngles(0,1,2).norm() > max_rotation_norm) && checkConsistency){
			fprintf(stderr,"****  NDTFuserHMT -- ALMOST DEFINATELY A REGISTRATION FAILURE *****\n");
			Tnow = Tnow * Tmotion;
			//save offending map:
			//map->writeToJFF("map.jff");
			//ndlocal.writeToJFF("local.jff");
		    }else{
			Tnow = Tinit;
			//Tnow = Tnow * Tmotion;
			lslgeneric::transformPointCloudInPlace(Tnow, cloud);
			Eigen::Affine3d spose = Tnow*sensor_pose;
			Eigen::Affine3d diff_fuse = Tlast_fuse.inverse()*Tnow;
			if(diff_fuse.translation().norm() > translation_fuse_delta ||
				diff_fuse.rotation().eulerAngles(0,1,2).norm() > rotation_fuse_delta)
			{
			    //std::cout<<"F: "<<spose.translation().transpose()<<" "<<spose.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
			    t4 = getDoubleTime();
			    map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);
			    t5 = getDoubleTime();
			    //map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 25, 2*map_size_z, 0.06);
			    //map->addPointCloud(spose.translation(),cloud, 0.06, 25);
			    //map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);
			    //t4 = getDoubleTime();
			    //std::cout<<"match: "<<t3-t2<<" addPointCloud: "<<t5-t4<<" ndlocal "<<t1-t0<<" total: "<<t5-t0<<std::endl;
			    Tlast_fuse = Tnow;
			    if(visualize) //&&ctr%20==0) 
			    {
				if(ctr%20==0) {
				    viewer->plotNDTSAccordingToOccupancy(-1,map); 
				    viewer->plotLocalNDTMap(cloud,resolution); 
				}
				viewer->addTrajectoryPoint(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),1,0,0);
				viewer->addTrajectoryPoint(Todom.translation()(0),Todom.translation()(1),Todom.translation()(2),0,1,0);
				viewer->displayTrajectory();
				viewer->setCameraPointing(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+3);
				viewer->repaint();	
			    }
			    ctr++;
			}
		    }
		}else{
		    t3 = getDoubleTime();
		    Tnow = Tnow * Tmotion;
		}
	    }
	    
	    t6 = getDoubleTime();
	    if(fAddTimes!=NULL) {
		fprintf(fAddTimes,"%lf %lf %lf\n",t3-t2,t5-t4,t6-t0);
		fflush(fAddTimes);
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
	Eigen::Vector3d localMapSize;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
}
#endif
