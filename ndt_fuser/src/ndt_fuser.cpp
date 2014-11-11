#include <ndt_fuser/ndt_fuser.h>

namespace lslgeneric {
    void NDTFuser::initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
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
#ifndef NO_NDT_VIZ
	    viewer->plotNDTSAccordingToOccupancy(-1,map); 
	    //viewer->plotLocalNDTMap(cloud,resolution); 
#endif
	}
    }
    Eigen::Affine3d NDTFuser::update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud){
	if(!isInit){
	    fprintf(stderr,"NDT-Fuser: Call Initialize first!!\n");
	    return Tnow;
	}
	double t1,t2,t3,t4;
	///Set the cloud to sensor frame with respect to base
	lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
	t1 = getDoubleTime();
	///Create local map
	lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(resolution));
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
#ifndef NO_NDT_VIZ
			    viewer->plotNDTSAccordingToOccupancy(-1,map); 
			    //viewer->plotLocalNDTMap(cloud,resolution); 
#endif
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
			    diff_fuse.rotation().eulerAngles(0,1,2).norm() > 2*rotation_fuse_delta)
		    {
			t3 = getDoubleTime();
			map->addPointCloud(spose.translation(),cloud, 0.06, 2.5);
			t4 = getDoubleTime();
			map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);
			Tlast_fuse = Tnow;
			if(visualize) 
			{
#ifndef NO_NDT_VIZ
			    viewer->plotNDTSAccordingToOccupancy(-1,map); 
			    //viewer->plotLocalNDTMap(cloud,resolution); 
#endif
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
}
