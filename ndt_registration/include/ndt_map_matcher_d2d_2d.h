////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// NDT2NDT - Map based localization in 2D
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AASS Research Center nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
* @brief NDT-to-NDT based localization using 3D NDT map and observation, but the pose is estimated in 2D (x,y,yaw)
* This wraps the NDTMatcherD2D_2D and provides Frame-to-Model registration, that is, map based localization estimate. 
*
* A static map is given for the class in initialization and then one can ask location estimates given a initial pose 
* in static map frame and measurement as point cloud.
* 
* @version 0.1-experimental
* @author Jari Saarinen (firstname.lastname@aalto.fi)
*
**/


#ifndef NDT_MAP_MATCHER_D2D_2D_HH
#define NDT_MAP_MATCHER_D2D_2D_HH

#include "ndt_map.h"
#include <ndt_matcher_d2d_2d.h>
#include "pcl/point_cloud.h"
#include "Eigen/Core"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>


namespace lslgeneric{

template <typename PointT>
class NDTMapMatcherD2D_2D{
	public:
		lslgeneric::NDTMap<PointT> map;  ///< da map
		
		/**
		* Constructor
		* @param map_resolution the desired map resolution of the map that is used for localization (can be equal or lower than given one)
		* @param &nd_map Map used for localization
		* @param miniz The minimum accepted z-value (components lower than this will be disregarded)
		* @param maxiz The maximum accepted z-value
		*/
		NDTMapMatcherD2D_2D(double map_resolution,  lslgeneric::NDTMap<PointT> &nd_map ,double miniz = 0.0, double maxiz = 0.0) : 
			map(new lslgeneric::LazyGrid<PointT>(map_resolution))
		{
			isInit = false;
			resolution = map_resolution;
			min_z = miniz;
			max_z = maxiz;
			sensor_pose.setIdentity();
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
			fprintf(stderr,"NDTMapMatcherD2D_2D::Inserting %d gaussians to map\n",ndts.size());
			for(unsigned int i=0;i<ndts.size();i++){
				Eigen::Vector3d m = ndts[i]->getMean();	
				if(m[2]>min_z && m[2] < max_z){
					Eigen::Matrix3d cov = ndts[i]->getCov();
					unsigned int nump = ndts[i]->getN();
					map.addDistributionToCell(cov, m,nump);
				}
			}	
		}
		/**
		* Set sensor pose with respect to base
		* The localization is done in base coordinates!
		*/
		void setSensorPose(Eigen::Affine3d spose){
			sensor_pose = spose;
		}
		
		/**
		* Update function
		* NOTE: this function will transform the cloud to base coordinates and does the min-max filtering!
		* Tinit is the global pose expressed as Eigen::Affine3d
		* The result is saved to Tinit!
		*/
		bool update(Eigen::Affine3d Tinit, pcl::PointCloud<PointT> &cloud){
			///Set the cloud to sensor frame with respect to base
			lslgeneric::transformPointCloudInPlace(sensor_pose, cloud);
			pcl::PointCloud<PointT> cloud_tmp;
			///Min max filter
			for(unsigned int i=0;i<cloud.size();i++){
				bool add = true;
				if(cloud.points[i].z > max_z || cloud.points[i].z<min_z){
					add=false;
				}
				if(add) cloud_tmp.push_back(cloud.points[i]);
			}
			cloud = cloud_tmp;			

			///Create local map
			lslgeneric::NDTMap<PointT> ndlocal(new lslgeneric::LazyGrid<PointT>(resolution));
			ndlocal.addPointCloudSimple(cloud);
			ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
			
			lslgeneric::NDTMatcherD2D_2D<PointT,PointT> matcher; //(false,false);
			return(matcher.match( map, ndlocal,Tinit,true));
		}
		
		/**
		* The same as update, but assumes that the scan already is in the base frame and the filtration has been done
		*/
		bool updateNoFilt(Eigen::Affine3d &Tinit, pcl::PointCloud<PointT> &cloud){
			
			///Create local map
			lslgeneric::NDTMap<PointT> ndlocal(new lslgeneric::LazyGrid<PointT>(resolution));
			ndlocal.addPointCloudSimple(cloud);
			ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
			
			lslgeneric::NDTMatcherD2D_2D<PointT,PointT> matcher; //(false,false);
			return matcher.match( map, ndlocal,Tinit,true);
		}
		
	private:
		bool isInit;
		
		double resolution; ///< resolution of the map
		double min_z; 		///<values less than this are removed
		double max_z;     ///<values larger than this are removed		
		
		Eigen::Affine3d sensor_pose;
		
		
};
}
#endif
