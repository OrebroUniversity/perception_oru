mrpt::gui::CDisplayWindow3D  win3D("2D NDT-MCL Visualization",800,600);
mrpt::opengl::CGridPlaneXYPtr plane = mrpt::opengl::CGridPlaneXY::Create();
mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
mrpt::opengl::CPointCloudColouredPtr gl_particles = mrpt::opengl::CPointCloudColoured::Create();



void initializeScene(){
	win3D.setCameraAzimuthDeg(00);
	win3D.setCameraElevationDeg(00);
	win3D.setCameraZoom(1.0);
	win3D.setCameraPointingToPoint(0,0,0);
	mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();
	gl_points->setPointSize(10.5);
	plane->setPlaneLimits (-250, 250, -250, 250);
	scene->insert( plane);
	scene->insert( gl_points );
	scene->insert( gl_particles );
	win3D.unlockAccess3DScene();
	win3D.repaint();
}

/**
* Draw the ellipses in the map to the scene
*/
void addMap2Scene(lslgeneric::NDTMap<pcl::PointXYZ> &map, Eigen::Vector3d &origin, mrpt::opengl::COpenGLScenePtr scene){
	///And plot the results
	std::vector<lslgeneric::NDTCell<pcl::PointXYZ>*> ndts;
	ndts = map.getAllCells();
	
	for(unsigned int i=0;i<ndts.size();i++){
		mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		Eigen::Matrix3d cov = ndts[i]->getCov();
		Eigen::Vector3d m = ndts[i]->getMean();
		
		if((m-origin).norm()>20.0) continue;
		
		mrpt::math::CMatrixDouble M = cov;
		objEllip->setCovMatrix(M);
		objEllip->setLocation(m[0], m[1], m[2]);
		float g=0,b=0;
		if(ndts[i]->getOccupancy()<0) objEllip->setColor(1.0,g,b,0.4);
		else objEllip->setColor(1.0,1.0,b,1.0);
		objEllip->enableDrawSolid3D(true);
		scene->insert( objEllip );
	}
	
	for(unsigned int i=0;i<ndts.size();i++){
		delete ndts[i];
	}
	
}

void addPoseCovariance(double x, double y, Eigen::Matrix3d cov, mrpt::opengl::COpenGLScenePtr scene){
	Eigen::Matrix2d c2d;
	c2d << cov(0,0), cov(0,1),
				 cov(1,0), cov(1,1);
	mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
	mrpt::math::CMatrixDouble M = c2d;
	objEllip->setCovMatrix(M);
	objEllip->setLocation(x, y, 0.01);
	objEllip->setColor(1.0,0,0,0.4);
	scene->insert( objEllip );
}


/**
* Add the laser scan to the scen 
*/
void addScanToScene(mrpt::opengl::COpenGLScenePtr scene, Eigen::Vector3d orig, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	
	mrpt::opengl::CSetOfLinesPtr obj = mrpt::opengl::CSetOfLines::Create();
	
	for(unsigned int i=0;i<cloud->points.size();i+=2){
			obj->appendLine(orig(0),orig(1),orig(2), cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
	}
	scene->insert(obj);
}

void addPointCloud2Scene(mrpt::opengl::COpenGLScenePtr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr points){
	gl_points->clear();
	fprintf(stderr,"Conflict Points Size = %d\n",points->size());
	for(unsigned int i=0;i<points->size();i++){
		gl_points->push_back(points->points[i].x, points->points[i].y, points->points[i].z,0,1.0,1.0) ;
	}
	scene->insert(gl_points);

}

/**
* Draw particles
**/
void addParticlesToWorld(mcl::CParticleFilter &pf, Eigen::Vector3d gt,Eigen::Vector3d dist_mean, Eigen::Vector3d odometry){
	gl_particles->clear();
	gl_particles->setPointSize(2.0);
	for(int i=0;i<pf.NumOfParticles;i++){
		gl_particles->push_back(pf.Particles[i].x, pf.Particles[i].y,0, 0,1,0);
	}
	gl_points->push_back(gt[0], gt[1],0.02, 1.0 ,0,0);
	//gl_points->push_back(odo[0], odo[1],0.02, 0 ,0,1.0);
	
	gl_points->push_back(dist_mean[0], dist_mean[1],0.02, 1.0 ,0,1.0);
	
	gl_points->push_back(odometry[0], odometry[1],0.02, 1.0 ,1.0,1.0);

}
