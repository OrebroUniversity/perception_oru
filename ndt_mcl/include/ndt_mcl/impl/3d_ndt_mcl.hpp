#ifndef NDT_MCL_3D_HPP_
#define NDT_MCL_3D_HPP_
#include <mrpt/utils/CTicTac.h> ///Timing
//#include <pcl/ros/conversions.h> //deprecated
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <pointcloud_vrml/pointcloud_utils.h>
#include "ndt_mcl/ParticleFilter3D.h"

/**
* NDT MCL - Class implementation
*/
template <typename PointT>
class NDTMCL3D{
	public:
		lslgeneric::NDTMap<PointT> map; 		///<This is my map 
		ParticleFilter3D pf; 						///<This is the particle filter
		double resolution;
		double resolution_sensor;
		int counter;
		double zfilt_min;
		bool forceSIR;
		/**
		* Constructor
		*/
		NDTMCL3D(double map_resolution, lslgeneric::NDTMap<PointT> &nd_map, double zfilter):
			map(new lslgeneric::LazyGrid<PointT>(map_resolution))
		{
				isInit = false;
				forceSIR = false;
				resolution=map_resolution;
				resolution_sensor = resolution;
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
				
				std::vector<lslgeneric::NDTCell<PointT>*> ndts;
				ndts = nd_map.getAllCells();
				fprintf(stderr,"NDT MAP with %d components",ndts.size());
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
		void initializeFilter(double x, double y, double z, double roll, double pitch, double yaw,
													double x_e, double y_e, double z_e, double roll_e, double pitch_e, double yaw_e, 
													unsigned int numParticles)
		{
			pf.initializeNormalRandom(numParticles, x,y,z,roll,pitch, yaw, x_e,y_e,z_e,roll_e,pitch_e,yaw_e);
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		void updateAndPredict(Eigen::Affine3d Tmotion, pcl::PointCloud<PointT> &cloud){
			Eigen::Vector3d tr = Tmotion.translation();
			Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);
			
			mrpt::utils::CTicTac	tictac;
			tictac.Tic();	
			
			pf.predict(Tmotion, tr[0]*0.1, tr[1]*0.1, tr[2]*0.1, rot[0]*0.1, rot[1]*0.1, rot[2]*0.1);
			double t_pred = tictac.Tac();	
			
			
			//pf.predict(mcl::pose(tr[0],tr[1],rot[2]), mcl::pose(tr[0]*0.1 + 0.005,tr[1]*0.1+ 0.005,rot[2]*0.1+0.001));

			lslgeneric::NDTMap<PointT> local_map(new lslgeneric::LazyGrid<PointT>(resolution_sensor));
			
			local_map.addPointCloudSimple(cloud);
			local_map.computeNDTCells();
			//local_map.computeNDTCells(CELL_UPDATE_MODE_STUDENT_T);
			int Nn = 0;
//		#pragma omp parallel for
			double t_pseudo = 0;

			for(int i=0;i<pf.size();i++){
				Eigen::Affine3d T = pf.pcloud[i].T;
				
				std::vector<lslgeneric::NDTCell<PointT>*> ndts;
				tictac.Tic();	
				ndts = local_map.pseudoTransformNDT(T);
				t_pseudo += tictac.Tac();
				double score=1;
				
				if(ndts.size()==0) fprintf(stderr,"ERROR no gaussians in measurement!!!\n");
				Nn = ndts.size();
				
				for(int n=0;n<ndts.size();n++){
					Eigen::Vector3d m = ndts[n]->getMean();	
					if(m[2]<zfilt_min) continue;
				
					lslgeneric::NDTCell<PointT> *cell;
					PointT p;
					p.x = m[0];p.y=m[1];p.z=m[2];
					
					if(map.getCellAtPoint(p,cell)){
					//if(map.getCellForPoint(p,cell)){
						if(cell == NULL) continue;
						if(cell->hasGaussian_){
							Eigen::Matrix3d covCombined = cell->getCov() + ndts[n]->getCov();
							Eigen::Matrix3d icov;
							bool exists;
							double det = 0;
							covCombined.computeInverseAndDetWithCheck(icov,det,exists);
							if(!exists) continue;
							double l = (cell->getMean() - m).dot(icov*(cell->getMean() - m));
							if(l*0 != 0) continue;
							score += 0.1 + 0.9 * exp(-0.05*l/2.0);
						}else{
						}
					}
				}
				
				pf.pcloud[i].lik = score;
				for(unsigned int j=0;j<ndts.size();j++){
					delete ndts[j];
				}
				
			}
			
			pf.normalize();
			

			if(forceSIR){	
				fprintf(stderr, "forceSIR(%d) ",forceSIR);
				pf.SIRUpdate();
			}else{
			
				double varP=0;
				for(int i = 0; i<pf.size();i++){
					varP += (pf.pcloud[i].p - 1.0/pf.size())*(pf.pcloud[i].p - 1.0/pf.size());
				}
				varP /= pf.size();
				varP = sqrt(varP); 
				fprintf(stderr,"Var P=%lf (Npf=%d, Nm=%d) (t_pred = %.3lf t_pseudo=%.3lf)",varP,pf.size(), Nn, t_pred,t_pseudo);
				if(varP > 0.006 || sinceSIR >25){
					fprintf(stderr,"-SIR- ");
					sinceSIR = 0;
					pf.SIRUpdate();
				}else{
					sinceSIR++;
				}
				
			}
		}
		
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			
		void updateAndPredictEff(Eigen::Affine3d Tmotion, pcl::PointCloud<PointT> &cloud){
			Eigen::Vector3d tr = Tmotion.translation();
			Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);
			
			mrpt::utils::CTicTac	tictac;
			tictac.Tic();	
			
			//pf.predict(Tmotion, tr[0]*0.1, tr[1]*0.1, tr[2]*0.1, rot[0]*0.1, rot[1]*0.1, rot[2]*0.1);
			if(rot[2]<(0.5 * M_PI/180.0) && tr[0]>=0){ 
				pf.predict(Tmotion, tr[0]*0.2 + 0.005, tr[1]*0.1+ 0.005, tr[2]*0.1+0.005 ,rot[0]*0.2+0.001,rot[1]*0.2+0.001, rot[2]*0.2+0.001);
			}else if(tr[0]>=0){
				pf.predict(Tmotion,tr[0]*0.5 + 0.005, tr[1]*0.1+ 0.005, tr[2]*0.1+0.005 ,rot[0]*0.2+0.001,rot[1]*0.2+0.001, rot[2]*0.4+0.001);
			}else{
				pf.predict(Tmotion, tr[0]*0.2 + 0.005, tr[1]*0.1+ 0.005, tr[2]*0.1+0.005 ,rot[0]*0.2+0.001,rot[1]*0.2+0.001, rot[2]*0.2+0.001);
			}
			
			
			double t_pred = tictac.Tac();	
	
			lslgeneric::NDTMap<PointT> local_map(new lslgeneric::LazyGrid<PointT>(resolution_sensor));		
			local_map.addPointCloudSimple(cloud);
			//local_map.computeNDTCells();
			local_map.computeNDTCellsSimple();
			
			std::vector<lslgeneric::NDTCell<PointT>*> ndts = local_map.getAllCells();
			
			
			
			int Nn = 0;
//		#pragma omp parallel for
			double t_pseudo = 0;
		#pragma omp parallel num_threads(4)
    {
     #pragma omp for
			for(int i=0;i<pf.size();i++){
				Eigen::Affine3d T = pf.pcloud[i].T;
				
				
				//tictac.Tic();	
				//ndts = local_map.pseudoTransformNDT(T);
				//t_pseudo += tictac.Tac();
				double score=1;
				
				if(ndts.size()==0) fprintf(stderr,"ERROR no gaussians in measurement!!!\n");
				Nn = ndts.size();
				
				for(int n=0;n<ndts.size();n++){
					Eigen::Vector3d m = T*ndts[n]->getMean();	
					
					if(m[2]<zfilt_min) continue;
				
					lslgeneric::NDTCell<PointT> *cell;
					PointT p;
					p.x = m[0];p.y=m[1];p.z=m[2];
					
					if(map.getCellAtPoint(p,cell)){
					//if(map.getCellForPoint(p,cell)){
						if(cell == NULL) continue;
						if(cell->hasGaussian_){
							Eigen::Matrix3d covCombined = cell->getCov() + T.rotation()*ndts[n]->getCov() *T.rotation().transpose();
							Eigen::Matrix3d icov;
							bool exists;
							double det = 0;
							covCombined.computeInverseAndDetWithCheck(icov,det,exists);
							if(!exists) continue;
							double l = (cell->getMean() - m).dot(icov*(cell->getMean() - m));
							if(l*0 != 0) continue;
							score += 0.1 + 0.9 * exp(-0.05*l/2.0);
						}else{
						}
					}
				}
				
				pf.pcloud[i].lik = score;
				
				
			}
		}///#pragma
			for(unsigned int j=0;j<ndts.size();j++){
					delete ndts[j];
			}
			
			
			pf.normalize();
			

			if(forceSIR){	
				fprintf(stderr, "forceSIR(%d) ",forceSIR);
				pf.SIRUpdate();
			}else{
			
				double varP=0;
				for(int i = 0; i<pf.size();i++){
					varP += (pf.pcloud[i].p - 1.0/pf.size())*(pf.pcloud[i].p - 1.0/pf.size());
				}
				varP /= pf.size();
				varP = sqrt(varP); 
				fprintf(stderr,"Var P=%lf (Npf=%d, Nm=%d) (t_pred = %.3lf t_pseudo=%.3lf)",varP,pf.size(), Nn, t_pred,t_pseudo);
				if(varP > 0.006 || sinceSIR >25){
					fprintf(stderr,"-SIR- ");
					sinceSIR = 0;
					pf.SIRUpdate();
				}else{
					sinceSIR++;
				}
				
			}
		}
		
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		Eigen::Vector3d getMeanVector(){
			return (pf.getMean().translation());
			//mcl::pose m = pf.getDistributionMean(true);
			//Eigen::Vector3d mm; mm<<m.x,m.y,m.a;
			//return mm;
		}
		Eigen::Affine3d getMean(){
			return (pf.getMean());
			//mcl::pose m = pf.getDistributionMean(true);
			//Eigen::Vector3d mm; mm<<m.x,m.y,m.a;
			//return mm;
		}
		
		
	private:
		bool isInit;
		int sinceSIR;
		/*
		Eigen::Affine3d getAsAffine(int i){
			Eigen::Matrix3d m;
			m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
					* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
					* Eigen::AngleAxisd(pf.Particles[i].a, Eigen::Vector3d::UnitZ());
			Eigen::Translation3d v(pf.Particles[i].x,pf.Particles[i].y,0);
			Eigen::Affine3d T = v*m;
			return T;
		}*/
		
};

#endif

