#include <ndt_fuser/ndt_fuser_hmt.h>

namespace lslgeneric {
    void NDTFuserHMT::initialize(Eigen::Affine3d initPos, pcl::PointCloud<pcl::PointXYZ> &cloud, bool preLoad)
    {
	///Set the cloud to sensor frame with respect to base
	lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
	lslgeneric::transformPointCloudInPlace(initPos, cloud);
	Tnow = initPos;
	//#ifdef BASELINE
	//#else
	if(beHMT) {
	    map = new lslgeneric::NDTMapHMT(resolution,
		    Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),
		    map_size_x,map_size_y,map_size_z,sensor_range,hmt_map_dir,true);
	    if(preLoad) {
		lslgeneric::NDTMapHMT *map_hmt = dynamic_cast<lslgeneric::NDTMapHMT*> (map);
		std::cout<<"Trying to pre-load maps at "<<initPos.translation()<<std::endl;
		map_hmt->tryLoadPosition(initPos.translation());
	    }
	} else {
	    map = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution));
	    if(preLoad) {
		char fname[1000];
		snprintf(fname,999,"%s/%s_map.jff",hmt_map_dir.c_str(),prefix.c_str());
		std::cerr<<"Loading "<<fname<<std::endl;
		map->loadFromJFF(fname);
	    } else {
              //		map = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution));
		map->initialize(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2),map_size_x,map_size_y,map_size_z);
	    }
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
#ifndef NO_NDT_VIZ
      //      # error compiling with visualization
	    viewer->plotNDTSAccordingToOccupancy(-1,map); 
	    //viewer->plotLocalNDTMap(cloud,resolution);
	    viewer->addTrajectoryPoint(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+1,1,0,0);
	    viewer->addTrajectoryPoint(Todom.translation()(0),Todom.translation()(1),Todom.translation()(2)+0.5,0,1,0);
	    viewer->displayTrajectory();
	    viewer->setCameraPointing(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+3);
	    viewer->repaint();	
#endif
        }
    }

    /**
     *
     *
     */
    Eigen::Affine3d NDTFuserHMT::update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud)
    {
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
	lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(resolution*resolution_local_factor));
	ndlocal.guessSize(0,0,0,sensor_range,sensor_range,map_size_z);
	ndlocal.loadPointCloud(cloud,sensor_range);
	ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
	//pass through ndlocal and set all cells with vertically pointing normals to non-gaussian :-O
	/*SpatialIndex *index = ndlocal.getMyIndex();
	  typename SpatialIndexctorItr it = index->begin();
	  while (it != index->end())
	  {
	  NDTCell *cell = dynamic_cast<NDTCell*> (*it);
	  if(cell!=NULL)
	  {
	  if(cell->hasGaussian_)
	  {
	  if(cell->getClass() == NDTCell::HORIZONTAL) {
	  cell->hasGaussian_ = false;
	  }
	  }
	  }
	  it++;
	  }*/

	t1 = getDoubleTime();
	Eigen::Affine3d Tinit = Tnow * Tmotion;
	if(disableRegistration) {
	    Tnow = Tinit;
	    lslgeneric::transformPointCloudInPlace(Tnow, cloud);
	    Eigen::Affine3d spose = Tnow*sensor_pose;
	    map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 25, 2*map_size_z, 0.06);
	    if(visualize) //&&ctr%20==0) 
	    {
#ifndef NO_NDT_VIZ
		if(ctr%50==0) {

		    viewer->plotNDTSAccordingToOccupancy(-1,map); 
		    //viewer->plotLocalNDTMap(cloud,resolution); 
		}
		viewer->addTrajectoryPoint(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+0.2,0,1,0);
		viewer->addTrajectoryPoint(Todom.translation()(0),Todom.translation()(1),Todom.translation()(2)+0.2,0.5,0,0.5);
		viewer->displayTrajectory();
		viewer->setCameraPointing(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+3);
		viewer->repaint();	
#endif
	    }
	    ctr++;
	    return Tnow;
	}

	if(doMultires) {
	    //create two ndt maps with resolution = 3*resolution (or 5?)
	    lslgeneric::NDTMap ndlocalLow(new lslgeneric::LazyGrid(3*resolution));
	    ndlocalLow.guessSize(0,0,0,sensor_range,sensor_range,map_size_z);
	    ndlocalLow.loadPointCloud(cloud,sensor_range);
	    ndlocalLow.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

	    lslgeneric::NDTMap mapLow(new lslgeneric::LazyGrid(3*resolution));
	    //add distros
	    double cx,cy,cz;
	    if(!map->getCentroid(cx, cy, cz)){
		fprintf(stderr,"Centroid NOT Given-abort!\n");
	    }
	    mapLow.initialize(cx,cy,cz,3*map_size_x,3*map_size_y,map_size_z);

	    std::vector<lslgeneric::NDTCell*> ndts;
	    ndts = map->getAllCells(); //this copies cells?

	    for(int i=0; i<ndts.size(); i++)	
	    {
		NDTCell *cell = ndts[i];
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
			//TSV: originally this!
			//map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 1250, map_size_z/2, 0.06);
			map->addPointCloudMeanUpdate(spose.translation(),cloud,localMapSize, 1e5, 25, 2*map_size_z, 0.06);
			t5 = getDoubleTime();
			//map->addPointCloud(spose.translation(),cloud, 0.06, 25);
			//map->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, spose.translation(), 0.1);
			//t4 = getDoubleTime();
			//std::cout<<"match: "<<t3-t2<<" addPointCloud: "<<t5-t4<<" ndlocal "<<t1-t0<<" total: "<<t5-t0<<std::endl;
			Tlast_fuse = Tnow;
			if(visualize) //&&ctr%20==0) 
			{
#ifndef NO_NDT_VIZ
			    if(ctr%30==0) {
				viewer->plotNDTSAccordingToOccupancy(-1,map); 
				//viewer->plotLocalNDTMap(cloud,resolution); 
			    }
			    viewer->addTrajectoryPoint(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+0.2,0,1,0);
			    viewer->addTrajectoryPoint(Todom.translation()(0),Todom.translation()(1),Todom.translation()(2)+0.2,0.5,0,0.5);
			    viewer->displayTrajectory();
			    viewer->setCameraPointing(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+3);
			    viewer->repaint();
			    //viewer->win3D->process_events();
#endif
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
            bool match_ret = false;
            if (doSoftConstraints) {
              // Local covariance matrix in vehicle frame.
              Eigen::MatrixXd local_cov = motionModel2D.getCovMatrix6(Tmotion, 1., 1., 1.); //0.0000000000001, 0.0000000000001, 0.0000000000001);
              
              Eigen::MatrixXd local_cov_pos = local_cov.block(0,0,3,3);

              // Convert it into the clobal frame.
              Eigen::MatrixXd global_cov_pos = Tinit.rotation()*local_cov_pos*Tinit.rotation().transpose();
              if (visualize) {
#ifndef NO_NDT_VIZ
                viewer->setPoseCov(Tinit.translation(), global_cov_pos);
                viewer->displayPoseCov();
#endif
              }

              Eigen::MatrixXd global_cov = local_cov;
              global_cov.block<3,3>(0,0) = global_cov_pos;

              // // Include global constraints? -> roll, pitch and z (height).
              // // Force initialization to be only yaw. Set roll, pitch and height to zero + put in a covariance with relative low variance in height, roll and pitch...
              // Tinit.translation()(2) = 0.;
              // // Get "robust yaw"..., set roll/pitch to zero.
              // double yaw = Tinit.rotation().eulerAngles(0,1,2)(2);
              // Tinit.rotation() = Eigen::AngleAxis<double>(0.,Eigen::Vector3d::UnitX()) *
              //   Eigen::AngleAxis<double>(0.,Eigen::Vector3d::UnitY()) *
              //   Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;

              matcherSC.only_xy_motion = false;//true;
              //              matcherSC.lock_zrp_motion = true;
              match_ret = matcherSC.match(*map, ndlocal,Tinit,global_cov);
              std::cout << matcherSC.nb_match_calls << " successes : " << matcherSC.nb_success_reg << std::endl;
            }
            else {
              match_ret = matcher.match( *map, ndlocal,Tinit,true);
            }
	    if(match_ret || fuseIncomplete){
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
#ifndef NO_NDT_VIZ
			    if(ctr%2==0) {
				viewer->plotNDTSAccordingToOccupancy(-1,map); 
				//viewer->plotLocalNDTMap(cloud,resolution); 
			    }
			    viewer->addTrajectoryPoint(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+2.2,0,1,0);
			    viewer->addTrajectoryPoint(Todom.translation()(0),Todom.translation()(1),Todom.translation()(2)+2.2,0.5,0,0.5);
			    viewer->displayTrajectory();
			    viewer->setCameraPointing(Tnow.translation()(0),Tnow.translation()(1),Tnow.translation()(2)+3);
			    viewer->repaint();
                viewer->win3D->process_events();
#endif
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
}
