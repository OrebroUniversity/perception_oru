#ifndef DAS_UTILSNDTCELL_06112017
#define DAS_UTILSNDTCELL_06112017

#include "Eigen/Core"
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"

#include "ndt_feature_finder/utils.hpp"

namespace perception_oru{
	namespace ndt_feature_finder{
		
		
		
		inline void getEigenVectors2D(const lslgeneric::NDTCell& cell, Eigen::Vector3d& eigenval, Eigen::Matrix3d& eigenvec){
			eigenval = cell.getEvals();
			eigenvec = cell.getEvecs();
			EigenSort2D(eigenval, eigenvec);
		}
		
		inline int getBiggestEigenVector2D(const lslgeneric::NDTCell& cell, Eigen::Vector3d& eigenval, Eigen::Matrix3d& eigenvec)
		{
			
			getEigenVectors2D(cell, eigenval, eigenvec);
			
// 			eigenval = cell.getEvals();
// 			eigenvec = cell.getEvecs();
		// 	std::cout << "Eigen Vec : "<< std::endl << eigenvec << std::endl;
		// 	std::cout << "Eigen sort" << std::endl;
// 			EigenSort2D(eigenval, eigenvec);
		// 	std::cout << "Eigen sorted" << std::endl;
				
			Eigen::Vector3d biggest_eigen = eigenvec.col(0);
			Eigen::Vector3d biggest_eigen1 = eigenvec.col(1);
			
			if(eigenval(1) > eigenval(0)){
				return  1;
			}
			return 0;
		}
		
		inline Eigen::Vector3d getNDTCellOrientation2D(const lslgeneric::NDTCell& cell1){
			Eigen::Vector3d eigenval; 
			Eigen::Matrix3d eigenvec;
			int index = getBiggestEigenVector2D(cell1, eigenval, eigenvec);
			return eigenvec.col(index);
		}
		
		///Get the angle of a cell compared to a reference vector
		inline double NDTCellAngle(const lslgeneric::NDTCell& cell, const Eigen::Vector2d& reference = Eigen::Vector2d(0,1)) {
			
			auto orientation = getNDTCellOrientation2D(cell);
			double angle = atan2(orientation(1), orientation(0)) - atan2(0, 1);
			if (angle < 0) angle += 2 * M_PI;
			return angle;
			
		}
		
		
		
		inline Eigen::Vector3d collisionNDTCells(const lslgeneric::NDTCell& cell1, const lslgeneric::NDTCell& cell2){
// 			Eigen::Vector3d eigenval; 
// 			Eigen::Matrix3d eigenvec;
// 			int index = getBiggestEigenVector2D(cell1, eigenval, eigenvec);
			Eigen::Vector3d orientation = getNDTCellOrientation2D(cell1);
// 			Eigen::Vector3d eigenval_tmp;
// 			Eigen::Matrix3d eigenvec_tmp;
// 			int index_tmp = getBiggestEigenVector2D(cell2, eigenval_tmp, eigenvec_tmp);
			Eigen::Vector3d orientation_tmp = getNDTCellOrientation2D(cell2);
			
			return collisionRay(orientation_tmp, cell2.getMean(), orientation, cell1.getMean());
		}
		
		/**
		 * @brief return the angle between two ndt cell and its direction.
		 * @param[in] cell1 : first cell
		 * @param[in] cell2 : second cell
		 * @param[in] collision_point : collision point of both cells
		 * @param[out] angle : actual angle
		 * @param[out] direction : direction of the angle.
		 */
		inline void angleNDTCells(const lslgeneric::NDTCell& cell1, const lslgeneric::NDTCell& cell2, const Eigen::Vector3d& collision_point, double& angle, double& angle_direction){
			
			auto mean_cell = cell1.getMean();
			auto mean_cell_tmp = cell2.getMean();
			
			auto vector1 = mean_cell - collision_point;
			auto vector2 = mean_cell_tmp - collision_point;
			
			//Caluclate the angle's direction.
			double angle_from = atan2(vector1(1), vector1(0)) - atan2(0, 1);
			if (angle_from < 0) angle_from += 2 * M_PI;
			double angle_to = atan2(vector2(1), vector2(0)) - atan2(0, 1);
			if (angle_to < 0) angle_to += 2 * M_PI;
			angle_direction = (angle_to + angle_from) / 2;
			
			//Calculate the actual angle
			double angle_between = atan2(vector1(1), vector1(0)) - atan2(vector2(1), vector2(0));
			if (angle_between < 0) angle_between += 2 * M_PI;
			
			//If the angle is more than PI we return the non obtuse angle so we need to redirect it.
			if(angle_between > M_PI){
				angle_between = (M_PI * 2) - angle_between;
				angle_direction - M_PI;
				if(angle_direction < 0){
					angle_direction = angle_direction + (2 * M_PI);
				}
			}
			
			//
			angle = angle_between;
			if(angle > M_PI){
				throw( std::runtime_error("Making sure that this get never hits. It's in Utils.hpp ndt_feature_finder") );
				angle = (M_PI * 2) - angle;
			}
			
// 			return angle_tmp;
			
		}
		
		
		
		
	}
	
}

#endif