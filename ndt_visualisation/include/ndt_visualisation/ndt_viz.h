#ifndef NDT_VIZ_HH
#define NDT_VIZ_HH

#include <mrpt/gui.h>
#include <mrpt/base.h>
#include <mrpt/opengl.h>

#include <ndt_map/ndt_map.h>
#include <ndt_visualisation/CMyEllipsoid.h>

#warning "ALWAYS PLACE THE ndt_viz.h BEFORE THE ROS HEADERS!!!"

class NDTViz {

    public:
	mrpt::gui::CDisplayWindow3D *win3D;
	mrpt::opengl::CPointCloudColouredPtr gl_points;
	mrpt::opengl::CPointCloudColouredPtr gl_particles;

	NDTViz(bool allocate_new_window=true)

	{
	    if(allocate_new_window) 
	    {
		win3D = new mrpt::gui::CDisplayWindow3D("NDT Viz",800,600);
	    }
	    else 
	    {
		win3D = NULL;
	    }

	    gl_points = mrpt::opengl::CPointCloudColoured::Create();
	    gl_particles = mrpt::opengl::CPointCloudColoured::Create();
	    gl_points->setPointSize(4.5);
	    gl_particles->setPointSize(2.5);

	}
	void repaint(){
	    win3D->repaint();
	}

	void clear(){
	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
	    scene->clear();
	    win3D->unlockAccess3DScene();
	}

	void clearTrajectoryPoints(){
	    gl_points->clear();
	}

	void addTrajectoryPoint(float x, float y, float z, float R=1.0, float G = 1.0, float B = 1.0){
	    win3D->get3DSceneAndLock();
	    gl_points->push_back(x, y, z, R ,G,B);
	    win3D->unlockAccess3DScene();
	}
	void displayTrajectory(){
	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
	    scene->insert(gl_points);
	    win3D->unlockAccess3DScene();
	}

	void clearParticles(){ gl_particles->clear();}
	void addParticle(float x, float y, float z, float R=1.0, float G = 1.0, float B = 1.0){
	    win3D->get3DSceneAndLock();
	    gl_particles->push_back(x, y, z, R ,G,B);
	    win3D->unlockAccess3DScene();
	}
	void displayParticles(){
	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
	    scene->insert(gl_particles);
	    win3D->unlockAccess3DScene();
	}
	void setCameraPointing(double x, double y, double z) {
	    win3D->get3DSceneAndLock();
	    win3D->setCameraPointingToPoint(x,y,z);
	    win3D->unlockAccess3DScene();
	}

	/**
	  * Add the laser scan to the scen 
	  */
	void addScan(Eigen::Vector3d orig, pcl::PointCloud<pcl::PointXYZ> &cloud, double R=1.0,double G=1.0,double B=1.0){

	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
	    mrpt::opengl::CSetOfLinesPtr obj = mrpt::opengl::CSetOfLines::Create();
	    for(unsigned int i=0;i<cloud.points.size();i+=2){
		obj->appendLine(orig(0),orig(1),orig(2), cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
	    }
	    obj->setColor(R,G,B);
	    scene->insert(obj);
	    win3D->unlockAccess3DScene();
	}
	void addPointCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, double R=1.0,double G=1.0,double B=1.0){

	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
	    mrpt::opengl::CPointCloudColouredPtr obj = mrpt::opengl::CPointCloudColoured::Create();
	    obj->setPointSize(2.5);
	    for(unsigned int i=0;i<cloud.points.size();i+=2){
		obj->push_back(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z,R,G,B);
	    }
	    scene->insert(obj);
	    win3D->unlockAccess3DScene();
	}



	void plotNDTMap(lslgeneric::NDTMap *map, double R=1.0,double G=1.0,double B=1.0, bool heightCoding=false, bool setCameraPos = true ){
	    if(win3D == NULL) return;
	    std::vector<lslgeneric::NDTCell*> global_ndts;
	    global_ndts = map->getAllCells();

	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
	    unsigned int accepted_ndts=1;
	    double x = 0,y=0,s=0;
	    fprintf(stderr,"-NDT:%lu-",global_ndts.size());
	    for(unsigned int i=0;i<global_ndts.size();i++){
		Eigen::Vector3d m = global_ndts[i]->getMean();
		if(!global_ndts[i]->hasGaussian_) continue;
		x+=m[0];
		y+=m[1];
		s+=1;

		accepted_ndts++;
		mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		Eigen::Matrix3d cov = global_ndts[i]->getCov();
		mrpt::math::CMatrixDouble M = 0.5*cov;
		objEllip->setCovMatrix(M);
		objEllip->setLocation(m[0], m[1], m[2]);
		double hval=1.0;
		if(heightCoding){
		    hval = fabs(m[2]+1.5)/6.0;
		}
		if(global_ndts[i]->getOccupancy() > 0){
		    objEllip->setColor(R/hval,G/hval,B/hval,1.0);
		}else{
		    objEllip->setColor(1.0,0,0,1.0);
		}

		objEllip->enableDrawSolid3D(true);
		scene->insert( objEllip );

	    }
	    if(setCameraPos) win3D->setCameraPointingToPoint(x/s,y/s,3.0);
	    win3D->unlockAccess3DScene();
	    win3D->repaint();
	    //fprintf(stderr,"(%lf %lf) s=%lf\n",x/s,y/s,s);
	    for(unsigned int i=0;i<global_ndts.size();i++) delete global_ndts[i];

	}

	void plotNDTSAccordingToCost(float occupancy, double MAX_COST, lslgeneric::NDTMap *map){

	    if(win3D == NULL) return;
	    std::vector<lslgeneric::NDTCell*> global_ndts;
	    global_ndts = map->getAllCells();
	    fprintf(stderr," NUM NDT: %lu ", global_ndts.size());

	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
	    scene->clear();
	    unsigned int accepted_ndts=1;
	    double x = 0,y=0,s=0;
	    for(unsigned int i=0;i<global_ndts.size();i++){
		Eigen::Vector3d m = global_ndts[i]->getMean();
		if(!global_ndts[i]->hasGaussian_) continue;
		x+=m[0];
		y+=m[1];
		s+=1;
		if(global_ndts[i]->getOccupancy()>occupancy){
		    accepted_ndts++;
		    mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		    Eigen::Matrix3d cov = global_ndts[i]->getCov();
		    mrpt::math::CMatrixDouble M = cov;
		    objEllip->setCovMatrix(M);
		    objEllip->setLocation(m[0], m[1], m[2]);
		    double c = global_ndts[i]->cost;
		    if(c > MAX_COST) c=MAX_COST;
		    c = 1 - c/MAX_COST; 
		    objEllip->setColor(c,c,c,0.6);

		    objEllip->enableDrawSolid3D(true);
		    scene->insert( objEllip );
		}else if(global_ndts[i]->getOccupancy()<-0){
		    /*
		       mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		       Eigen::Matrix3d cov = global_ndts[i]->getCov();
		       cov = cov;
		    //Eigen::Vector3d m = global_ndts[i]->getMean();

		    mrpt::math::CMatrixDouble M = cov;
		    objEllip->setCovMatrix(M);
		    objEllip->setLocation(m[0], m[1], m[2]);
		    objEllip->setColor(1.0,0,0,1.0);
		    objEllip->enableDrawSolid3D(true);
		    scene->insert( objEllip );*/
		}
	    }
	    //win3D->setCameraPointingToPoint(x/s,y/s,3.0);
	    win3D->unlockAccess3DScene();
	    win3D->repaint();
	    fprintf(stderr,"(%lf %lf) s=%lf\n",x/s,y/s,s);
	    for(unsigned int i=0;i<global_ndts.size();i++) delete global_ndts[i];

	}
	/** plots ndts according to the cell class, with an occupancy cutoff */
	void plotNDTSAccordingToClass(float occupancy, lslgeneric::NDTMap *map){

	    if(win3D == NULL) return;
	    std::vector<lslgeneric::NDTCell*> global_ndts;
	    global_ndts = map->getAllCells();
	    fprintf(stderr," NUM NDT: %lu ", global_ndts.size());

	    Eigen::Vector3d cFlat, cInclined, cRough, cVert, cUnknown, color;
	    cFlat<<0,0.9,0;
	    cInclined<<0.1,0.2,0.5;
	    cRough<<0.9,0,0;
	    cVert<<0,0,0.9;
	    cUnknown<<0,0,0;

	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
	    scene->clear();
	    unsigned int accepted_ndts=1;
	    double x = 0,y=0,s=0;
	    for(unsigned int i=0;i<global_ndts.size();i++){
		Eigen::Vector3d m = global_ndts[i]->getMean();
		if(!global_ndts[i]->hasGaussian_) continue;
		x+=m[0];
		y+=m[1];
		s+=1;
		if(global_ndts[i]->getOccupancy()>occupancy){
		    accepted_ndts++;
		    mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		    Eigen::Matrix3d cov = global_ndts[i]->getCov();
		    mrpt::math::CMatrixDouble M = cov;
		    objEllip->setCovMatrix(M);
		    objEllip->setLocation(m[0], m[1], m[2]);
		    switch(global_ndts[i]->getClass()) {
			case lslgeneric::NDTCell::HORIZONTAL : 
			    color = cFlat;
			    break;
			case lslgeneric::NDTCell::VERTICAL :
			    color = cVert;
			    break;
			case lslgeneric::NDTCell::INCLINED :
			    color = cInclined;
			    break;
			case lslgeneric::NDTCell::ROUGH :
			    color = cRough;
			    break;
			default:
			    color = cUnknown;
		    }
		    objEllip->setColor(color(0),color(1),color(2),0.6);

		    objEllip->enableDrawSolid3D(true);
		    scene->insert( objEllip );
		}else if(global_ndts[i]->getOccupancy()<-0){
		    /*
		       mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		       Eigen::Matrix3d cov = global_ndts[i]->getCov();
		       cov = cov;
		    //Eigen::Vector3d m = global_ndts[i]->getMean();

		    mrpt::math::CMatrixDouble M = cov;
		    objEllip->setCovMatrix(M);
		    objEllip->setLocation(m[0], m[1], m[2]);
		    objEllip->setColor(1.0,0,0,1.0);
		    objEllip->enableDrawSolid3D(true);
		    scene->insert( objEllip );*/
		}
	    }
	    //win3D->setCameraPointingToPoint(x/s,y/s,3.0);
	    win3D->unlockAccess3DScene();
	    win3D->repaint();
	    fprintf(stderr,"(%lf %lf) s=%lf\n",x/s,y/s,s);
	    for(unsigned int i=0;i<global_ndts.size();i++) delete global_ndts[i];

	}

	void plotNDTSAccordingToOccupancy(float occupancy, lslgeneric::NDTMap *map){
	    if(win3D == NULL) return;
	    std::vector<lslgeneric::NDTCell*> global_ndts;
	    global_ndts = map->getAllCells();
	    fprintf(stderr," NUM NDT: %lu ", global_ndts.size());

	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
	    scene->clear();
	    unsigned int accepted_ndts=1;
	    double x = 0,y=0,s=0;
	    for(unsigned int i=0;i<global_ndts.size();i++){
		Eigen::Vector3d m = global_ndts[i]->getMean();
		if(!global_ndts[i]->hasGaussian_) continue;
		x+=m[0];
		y+=m[1];
		s+=1;
		if(global_ndts[i]->getOccupancy()>occupancy){
		    accepted_ndts++;
		    mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		    Eigen::Matrix3d cov = global_ndts[i]->getCov();
		    mrpt::math::CMatrixDouble M = cov;
		    objEllip->setCovMatrix(M);
		    objEllip->setLocation(m[0], m[1], m[2]);

		    objEllip->setColor((m[2]+2.0)/10.0,0,0,0.6);

		    objEllip->enableDrawSolid3D(true);
		    scene->insert( objEllip );
		}else if(global_ndts[i]->getOccupancy()<-0){
		    /*
		       mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		       Eigen::Matrix3d cov = global_ndts[i]->getCov();
		       cov = cov;
		    //Eigen::Vector3d m = global_ndts[i]->getMean();

		    mrpt::math::CMatrixDouble M = cov;
		    objEllip->setCovMatrix(M);
		    objEllip->setLocation(m[0], m[1], m[2]);
		    objEllip->setColor(1.0,0,0,1.0);
		    objEllip->enableDrawSolid3D(true);
		    scene->insert( objEllip );*/
		}
	    }
	    //win3D->setCameraPointingToPoint(x/s,y/s,3.0);
	    win3D->unlockAccess3DScene();
	    win3D->repaint();
	    fprintf(stderr,"(%lf %lf) s=%lf\n",x/s,y/s,s);
	    for(unsigned int i=0;i<global_ndts.size();i++) delete global_ndts[i];

	}

	void plotLocalNDTMap(pcl::PointCloud<pcl::PointXYZ> &cloud, double resolution, double R=0, double G=1, double B=0, bool heightCoding=true){
	    if(win3D == NULL) return;

	    lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(resolution));
	    ndlocal.addPointCloudSimple(cloud);
	    ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

	    std::vector<lslgeneric::NDTCell*> ndts;
	    ndts = ndlocal.getAllCells();
	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
	    std::cout<<"LOCAL: "<<ndts.size()<<std::endl;

	    for(unsigned int i=0;i<ndts.size();i++){
		Eigen::Vector3d m = ndts[i]->getMean();
		//if(m[2]>3.0) continue;

		mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		Eigen::Matrix3d cov = ndts[i]->getCov();
		cov = cov;

		mrpt::math::CMatrixDouble M = 0.5*cov;
		objEllip->setCovMatrix(M);
		objEllip->setLocation(m[0], m[1], m[2]);
		double hval=1.0;
		if(heightCoding){
		    hval = fabs(m[2]+1.5)/20.0;
		}
		objEllip->setColor(R/hval,G/hval,B/hval,0.4);
		objEllip->enableDrawSolid3D(true);
		scene->insert( objEllip );
	    }
	    win3D->unlockAccess3DScene();
	    for(unsigned int i=0;i<ndts.size();i++){
		delete ndts[i];
	    }

	}

	void plotLocalConflictNDTMap(lslgeneric::NDTMap *map, pcl::PointCloud<pcl::PointXYZ> &cloud, 
		double resolution, double R=1, double G=0, double B=0, bool heightCoding=false, double maxz=0){
	    if(win3D == NULL) return;

	    lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(resolution));

	    ndlocal.addPointCloudSimple(cloud);
	    ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

	    std::vector<lslgeneric::NDTCell*> ndts;
	    ndts = ndlocal.getAllCells();

	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
	    std::cout<<"LOCAL: "<<ndts.size()<<std::endl;

	    for(unsigned int i=0;i<ndts.size();i++){


		Eigen::Vector3d m = ndts[i]->getMean();
		if(m[2]>maxz) continue;
		pcl::PointXYZ p;
		p.x = m[0]; p.y=m[1]; p.z = m[2];
		lslgeneric::NDTCell *map_cell=NULL;
		map->getCellAtPoint(p, map_cell);
		if(map_cell == NULL) continue;

		if(map_cell->getOccupancy()>0.5) continue;

		//	if(m[2]>3.0) continue;

		mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		Eigen::Matrix3d cov = ndts[i]->getCov();
		cov = cov;

		mrpt::math::CMatrixDouble M = 0.5*cov;
		objEllip->setCovMatrix(M);
		objEllip->setLocation(m[0], m[1], m[2]);
		double hval=1.0;
		if(heightCoding){
		    hval = fabs(m[2]+1.5)/20.0;
		}
		objEllip->setColor(R/hval,G/hval,B/hval,0.6);
		objEllip->enableDrawSolid3D(true);
		scene->insert( objEllip );
	    }
	    win3D->unlockAccess3DScene();
	    for(unsigned int i=0;i<ndts.size();i++){
		delete ndts[i];
	    }

	}


	void plotNDTTraversabilityMap(lslgeneric::NDTMap *map){
	    if(win3D == NULL) return;
	    std::vector<lslgeneric::NDTCell*> global_ndts;
	    global_ndts = map->getAllCells();

	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();

	    for(unsigned int i=0;i<global_ndts.size();i++){
		Eigen::Vector3d m = global_ndts[i]->getMean();
		if(!global_ndts[i]->hasGaussian_) continue;

		mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		Eigen::Matrix3d cov = global_ndts[i]->getCov();
		mrpt::math::CMatrixDouble M = 0.5*cov;
		objEllip->setCovMatrix(M);
		objEllip->setLocation(m[0], m[1], m[2]);

		//lslgeneric::CellClass CC;
		//lslgeneric::NDTCell<PointT>::CellClass CC;

		//CC = global_ndts[i]->getClass();
		// {HORIZONTAL=0, VERTICAL, INCLINED, ROUGH, UNKNOWN};
		if(global_ndts[i]->getClass() == lslgeneric::NDTCell::HORIZONTAL){
		    objEllip->setColor(0,1.0,0,1.0);
		}else if(global_ndts[i]->getClass() == lslgeneric::NDTCell::VERTICAL){
		    objEllip->setColor(1.0,0,0,1.0);
		}else if(global_ndts[i]->getClass() == lslgeneric::NDTCell::INCLINED){
		    objEllip->setColor(1.0,1.0,0,1.0);
		}else if(global_ndts[i]->getClass() == lslgeneric::NDTCell::ROUGH){
		    objEllip->setColor(0,0,1.0,1.0);
		}else{
		    objEllip->setColor(1.0,1.0,1.0,1.0);
		}

		objEllip->enableDrawSolid3D(true);
		scene->insert( objEllip );

	    }

	    win3D->unlockAccess3DScene();
	    win3D->repaint();
	    for(unsigned int i=0;i<global_ndts.size();i++) delete global_ndts[i];

	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *  Computes and visualizes an NDT depth image, based on maximum likelihood estimation
	 * 
	 */
	void ndtCoolCam(lslgeneric::NDTMap *map ,const Eigen::Affine3d &spos, double maxDist=70.0, 
		unsigned int Nx=800, unsigned int Ny=600, double fx=800, double fy=600){
	    Eigen::Matrix3d K;
	    K << fx,0,(double)Nx/2.0,
	      0, fy, (double)Ny/2.0,
	      0,0,1;

	    Eigen::Matrix3d invK;
	    double det=0;
	    bool exists=false;
	    K.computeInverseAndDetWithCheck(invK,det,exists);

	    if(!exists){
		fprintf(stderr,"ndtCoolCam::ERROR: NO INVERSE!!!!\n");
		return;
	    }

	    Eigen::Matrix3d m1,m2; ///Orientation matrix, for rotating the camera to world frame
	    m1 = Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

	    m2 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ());

	    mrpt::opengl::CTexturedPlanePtr gl_img 		=  mrpt::opengl::CTexturedPlane::Create(0.5,-0.5,-0.5,0.5);
	    mrpt::opengl::COpenGLViewportPtr viewInt;
	    //mrpt::utils::CImage  img(Nx,Ny,CH_GRAY);
	    mrpt::utils::CImage  img(Nx,Ny,CH_RGB);


	    //mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();	

	    //mrpt::opengl::CSetOfLinesPtr obj = mrpt::opengl::CSetOfLines::Create();		
	    for(unsigned int i=0; i<Nx*Ny;i++){
		Eigen::Vector3d pix(i%Nx,(int)i/Nx,1);
		Eigen::Vector3d dirv = invK*pix; ///< direction vector based on camera model
		dirv.normalize();

		Eigen::Vector3d dirv_w = spos.rotation()*(m2*(m1*dirv)); ///Direction vector in world frame
		//double max_depth = 20;
		double depth = map->getDepth(spos.translation(), dirv_w, maxDist);

		if(depth>maxDist) img.setPixel(i%Nx,i/Nx,0);
		else{
		    float x = (depth/maxDist);
		    float r = ((x >= 3.0/8.0) & (x < 5.0/8.0))*(4. * x - 3./2.)+((x >= 5./8.) & (x < 7./8.))+(x >= 7./8.) * (-4. * x + 9./2.);
		    float g = ((x >= 1./8.) & (x < 3./8.))*(4. * x - 1./2.)+((x >= 3./8.) & (x < 5./8.))+((x >= 5./8.) & (x < 7./8.))*(-4. * x + 7./2.);
		    float b = (x < 1./8.)*(4. * x + 1./2.)+((x >= 1./8.) & (x < 3./8.))+((x >= 3./8.) & (x < 5./8.))*(-4. * x + 5./2.);
		    size_t color=((unsigned char)(r*255))*255*255 + ((unsigned char)(g*255))*255 + ((unsigned char)(b*255));

		    img.setPixel(i%Nx,i/Nx,color);
		    //fprintf(stderr,"(%.2lf) ",depth);
		}
		/*
		   Eigen::Vector3d ray_endpos=spos.translation() + dirv_w * max_depth;
		   obj->appendLine(spos.translation()(0),spos.translation()(1),spos.translation()(2),
		   ray_endpos(0), ray_endpos(1), ray_endpos(2));*/

	    }
	    //scene->insert(obj);

	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();	
	    scene->clear();

	    viewInt = scene->createViewport("view2d");
	    scene->insert( gl_img, "view2d");

	    viewInt->setViewportPosition(0, 0, -1.0, -1.0);

	    viewInt->setTransparent(false);

	    viewInt->getCamera().setOrthogonal(true);
	    viewInt->getCamera().setAzimuthDegrees(90);
	    viewInt->getCamera().setElevationDegrees(90);
	    viewInt->getCamera().setZoomDistance(1.0);

	    gl_img->assignImage_fast(img);

	    win3D->unlockAccess3DScene();
	    win3D->repaint();
	}	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



};

#endif
