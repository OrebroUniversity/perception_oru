#ifndef NDT_MCL_HPP_
#define NDT_MCL_HPP_
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ndt_map.h>
#include <ndt_cell.h>
#include <pointcloud_utils.h>
#include <tf_conversions/tf_eigen.h>
#include "CParticleFilter.h"
#include <pointcloud_utils.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>

/**
* NDT MCL - Class implementation
*/
template <typename PointT>
class NDTMCL{
	public:
		lslgeneric::NDTMap<PointT> map; 		///<This is my map 
		mcl::CParticleFilter pf; 						///<This is the particle filter
		double resolution;
		int counter;
		double zfilt_min;
		bool forceSIR;
		/**
		* Constructor
		*/
		NDTMCL(double map_resolution, lslgeneric::NDTMap<PointT> &nd_map, double zfilter):
			map(new lslgeneric::LazyGrid<PointT>(map_resolution))
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
		void initializeFilter(double x, double y, double yaw,double x_e, double y_e, double yaw_e, int numParticles){
			pf.initializeNormalRandom(mcl::pose(x,y,yaw), mcl::pose(x_e,y_e,yaw_e),numParticles);
		}
		
		void updateAndPredict(Eigen::Affine3d Tmotion, pcl::PointCloud<PointT> &cloud){
			Eigen::Vector3d tr = Tmotion.translation();
			Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);
			
			
			/** Arla Motion model
			if(rot[2]<(0.5 * M_PI/180.0) && tr[0]>=0){ 
				pf.predict(mcl::pose(tr[0],tr[1],rot[2]), mcl::pose(tr[0]*0.2 + 0.005,tr[1]*0.1+ 0.005,rot[2]*0.2+0.001));
			}else if(tr[0]>=0){
				pf.predict(mcl::pose(tr[0],tr[1],rot[2]), mcl::pose(tr[0] + 0.01,tr[1] + 0.01,rot[2]*0.5+0.001));
			}else{
				pf.predict(mcl::pose(tr[0],tr[1],rot[2]), mcl::pose(tr[0] + 0.02,tr[1] + 0.01,rot[2]*0.8+0.001));
			}
			**/
			pf.predict(mcl::pose(tr[0],tr[1],rot[2]), mcl::pose(tr[0]*0.1 + 0.005,tr[1]*0.1+ 0.005,rot[2]*0.1+0.001));
			
			lslgeneric::NDTMap<PointT> local_map(new lslgeneric::LazyGrid<PointT>(resolution));
			
			/*
			pcl::PointCloud<PointT> cl_f;
			pcl::PointCloud<PointT> cl_z;
			for(int i=0;i<cloud.size();i++){
				if(cloud.points[i].z > 1.95 && cloud.points[i].z <2.05 ) cl_z.push_back(cloud.points[i]);
				if(cloud.points[i].z > zfilt_min ) cl_f.push_back(cloud.points[i]);
			}
			
			
			fprintf(stderr,"2D scan = %d Points\n",cl_z.size());
			
			*/
			//fprintf(stderr,"Could = %d Points\n",cloud.size());
			local_map.addPointCloudSimple(cloud);
			//local_map.computeNDTCells();
			local_map.computeNDTCells(CELL_UPDATE_MODE_STUDENT_T);
		 int Nn = 0;
//			#pragma omp parallel for
			for(int i=0;i<pf.NumOfParticles;i++){
				Eigen::Affine3d T = getAsAffine(i);
				
				std::vector<lslgeneric::NDTCell<PointT>*> ndts;
				ndts = local_map.pseudoTransformNDT(T);
				double score=1;
				
				if(ndts.size()==0) fprintf(stderr,"ERROR no gaussians in measurement!!!\n");
				Nn = ndts.size();
				for(int n=0;n<ndts.size();n++){
					Eigen::Vector3d m = ndts[n]->getMean();	
					if(m[2]<zfilt_min) continue;
				
					lslgeneric::NDTCell<PointT> *cell;
					PointT p;
					p.x = m[0];p.y=m[1];p.z=m[2];
					
					//if(map.getCellAtPoint(p,cell)){
					if(map.getCellForPoint(p,cell)){
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
							//if(l > 120) continue;
							score += 0.1 + 0.9 * exp(-0.05*l/2.0);
						}else{
						}
					}
				}
				
				 /*  -lfd1*(exp(-lfd2*l/2));*/
				
				pf.Particles[i].lik = score;
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
				for(int i = 0; i<pf.NumOfParticles;i++){
					varP += (pf.Particles[i].p - 1.0/pf.NumOfParticles)*(pf.Particles[i].p - 1.0/pf.NumOfParticles);
				}
				varP /= pf.NumOfParticles;
				varP = sqrt(varP); 
				fprintf(stderr,"Var P=%lf (Npf=%d, Nm=%d)",varP,pf.NumOfParticles, Nn);
				if(varP > 0.006 || sinceSIR >25){
					fprintf(stderr,"-SIR- ");
					sinceSIR = 0;
					pf.SIRUpdate();
				}else{
					sinceSIR++;
				}
				
			}
		}
		
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

