//mrpt stuff
#include <ndt_viz.h>
#include <mrpt/utils/CTicTac.h>

#include "ParticleFilter3D.h"
#include "3d_ndt_mcl.hpp"
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;

NDTViz<pcl::PointXYZI> ndt_viz;

int main(void){
	ParticleFilter3D filt;
	
	filt.initializeNormalRandom(1000, 1, 2, 3, 0.1, 0.2, 0.3, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1);
	
	fprintf(stderr,"Initilized a filter with %d Particles\n", filt.size());
	double dx = 0.05;
	double dy = 0;
	double dz = 0.01;
	
	double droll = 0;
	double dpitch =-0.001;
	double dyaw = 0.01;
	
	Eigen::Affine3d Tmotion = filt.xyzrpy2affine(dx, dy, dz, droll,dpitch,dyaw);
	
	mrpt::opengl::CGridPlaneXYPtr plane = mrpt::opengl::CGridPlaneXY::Create();
	mrpt::opengl::COpenGLScenePtr &scene = ndt_viz.win3D->get3DSceneAndLock();
	//plane->setPlaneLimits (-100, 150, -100, 150);
	scene->insert( plane);
	ndt_viz.win3D->unlockAccess3DScene();
	
	ndt_viz.displayTrajectory();
	for(int i=0;i<1000;i++){
		filt.SIRUpdate();
		filt.predict(Tmotion, dx*0.1, dy*0.1, dz*0.1, droll*0.1, dpitch*0.1, dyaw*0.1);
		
		ndt_viz.clearParticles();
		
		for(int i=0;i<filt.size();i++){
			double x,y,z;
			filt.pcloud[i].getXYZ(x,y,z);
			ndt_viz.addParticle(x, y,z, 1.0, 1.0, 1.0);
		}
		
		Eigen::Affine3d mean = filt.getMean();
		ndt_viz.addTrajectoryPoint(mean.translation()[0],mean.translation()[1],mean.translation()[2],1.0,0,0);
		
		ndt_viz.displayParticles();
		
		ndt_viz.repaint();
	}
	
	fprintf(stderr,"-- The End --\n");
	return 0;
}
