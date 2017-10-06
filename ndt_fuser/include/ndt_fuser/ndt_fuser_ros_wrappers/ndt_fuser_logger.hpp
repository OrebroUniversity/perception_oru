#ifndef NDT_FUSER_HMT_LOGGER_HH_04052017
#define NDT_FUSER_HMT_LOGGER_HH_04052017

#include <iostream>
#include <fstream>
#include <sys/stat.h>

#include "ros/time.h"

#include "ndt_fuser/ndt_fuser_hmt.h"

//#define BASELINE

namespace perception_oru {
	
	namespace ndt_fuser{
		/**
		* \brief This class fuses new point clouds into a common ndt map reference, keeping tack of the 
		* camera postion. It also log the position and orientation of each pose along with a time stamp a file
		* \author Malcolm
		*/
		class NDTFuserHMTLogger : public perception_oru::NDTFuserHMT{
			
		protected:
			std::string _file_out_logger;
			
		public:
			NDTFuserHMTLogger(const std::string& file_out_logger,double map_resolution, double map_size_x_, double map_size_y_, double map_size_z_, double sensor_range_ = 3, bool visualize_ = false, bool be2D_ = false, bool doMultires_ = false, bool fuseIncomplete_ = false, int max_itr = 30, std::__cxx11::string prefix_ = "", bool beHMT_ = true, std::__cxx11::string hmt_map_dir_ = "map", bool _step_control = true, bool doSoftConstraints_ = false, int nb_neighbours = 2, double resolutionLocalFactor = 1.) : 
				NDTFuserHMT(map_resolution, map_size_x_, map_size_y_, map_size_z_, sensor_range_, visualize_, be2D_, doMultires_, fuseIncomplete_, max_itr, prefix_, beHMT_, hmt_map_dir_, _step_control, doSoftConstraints_, nb_neighbours, resolutionLocalFactor), _file_out_logger(file_out_logger) {};
				
			
			/**
			*
			*
			*/
			Eigen::Affine3d update(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Time time = ros::Time::now()){
				std::cout << "Update in the logger" << std::endl;
				Eigen::Affine3d Tnow_out = perception_oru::NDTFuserHMT::update(Tmotion, cloud);
				
				logT(Tnow_out, time);
				return Tnow_out;
				
			}
			
			void logT(const Eigen::Affine3d& T_out, ros::Time time = ros::Time::now()){
				std::cout <<"Log in " << _file_out_logger << std::endl;
				Eigen::Affine2d T2d = eigenAffine3dTo2d(T_out);
				
				Eigen::Rotation2Dd R(0);
				Eigen::Vector2d t(T2d.translation());
				R.fromRotationMatrix(T2d.linear());
				
				
				std::ofstream myfile;
				if(!exists_test3(_file_out_logger)){
					myfile.open (_file_out_logger.c_str());
				}
				else{
					myfile.open (_file_out_logger.c_str(), std::ios::out | std::ios::app);
				}
				
				if (myfile.is_open())
				{
					std::cout << t(0) << " " << t(1) << " " << R.angle() << " " << time << std::endl;
					myfile << t(0) << " " << t(1) << " " << R.angle() << " " << time << "\n";
					myfile.close();
				}
				else std::cout << "Unable to open file";
				
			}
			
		private:
			Eigen::Affine2d eigenAffine3dTo2d(const Eigen::Affine3d &a3d) {
				return Eigen::Translation2d(a3d.translation().topRows<2>()) *
					Eigen::Rotation2D<double>(getRobustYawFromAffine3d(a3d));//a3d.linear().topLeftCorner<2,2>();
			}
			
			double getRobustYawFromAffine3d(const Eigen::Affine3d &a) {
				// To simply get the yaw from the euler angles is super sensitive to numerical errors which will cause roll and pitch to have angles very close to PI...
				Eigen::Vector3d v1(1,0,0);
				Eigen::Vector3d v2 = a.rotation()*v1;
				double dot = v1(0)*v2(0)+v1(1)*v2(1); // Only compute the rotation in xy plane...
				double angle = acos(dot);
				// Need to find the sign
				if (v1(0)*v2(1)-v1(1)*v2(0) > 0)
					return angle;
				return -angle;
			}
			
			inline bool exists_test3 (const std::string& name) {
				struct stat buffer;   
				return (stat (name.c_str(), &buffer) == 0); 
			}
			
			
		};
	}
}

#endif