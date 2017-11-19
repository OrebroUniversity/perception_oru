#ifndef DAS_UTILSNDTCELL_06112017
#define DAS_UTILSNDTCELL_06112017

#include "Eigen/Core"
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"

#include "ndt_feature_finder/utils.hpp"

namespace perception_oru{
	namespace ndt_feature_finder{
		
		inline void getClosestCells(const perception_oru::NDTMap& map, const perception_oru::NDTCell& cell, int neig_size, std::vector< boost::shared_ptr< perception_oru::NDTCell > >& cells_gaussian, std::vector< boost::shared_ptr< perception_oru::NDTCell > >& cells_initialized);
		
		
		inline void getEigenVectors2D(const perception_oru::NDTCell& cell, Eigen::Vector3d& eigenval, Eigen::Matrix3d& eigenvec){
			eigenval = cell.getEvals();
			eigenvec = cell.getEvecs();
			EigenSort2D(eigenval, eigenvec);
		}
		
		inline int getBiggestEigenVector2D(const perception_oru::NDTCell& cell, Eigen::Vector3d& eigenval, Eigen::Matrix3d& eigenvec)
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
		
		inline Eigen::Vector3d getNDTCellOrientation2D(const perception_oru::NDTCell& cell1){
			Eigen::Vector3d eigenval; 
			Eigen::Matrix3d eigenvec;
			int index = getBiggestEigenVector2D(cell1, eigenval, eigenvec);
			return eigenvec.col(index);
		}
		
		///Get the angle of a cell compared to a reference vector
		inline double NDTCellAngle(const perception_oru::NDTCell& cell, const Eigen::Vector2d& reference = Eigen::Vector2d(0,1)) {
			
			auto orientation = getNDTCellOrientation2D(cell);
			double angle = atan2(orientation(1), orientation(0)) - atan2(0, 1);
			if (angle < 0) angle += 2 * M_PI;
			return angle;
			
		}
		
		
		
		inline Eigen::Vector3d collisionNDTCells(const perception_oru::NDTCell& cell1, const perception_oru::NDTCell& cell2){
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
		 * @param[out] direction : one possible direction of the angle.
		 */

		inline void angleNDTCells(const perception_oru::NDTCell& cell1, const perception_oru::NDTCell& cell2, const Eigen::Vector3d& collision_point, double& angle, double& angle_orientation){
			
			auto mean_cell = cell1.getMean();
			auto mean_cell_tmp = cell2.getMean();
			
			auto vector1 = mean_cell - collision_point;
			auto vector2 = mean_cell_tmp - collision_point;
			
			//Caluclate the angle's direction.
			double angle_from = atan2(vector1(1), vector1(0)) - atan2(0, 1);
			if (angle_from < 0) angle_from += 2 * M_PI;
			double angle_to = atan2(vector2(1), vector2(0)) - atan2(0, 1);
			if (angle_to < 0) angle_to += 2 * M_PI;
			angle_orientation = (angle_to + angle_from) / 2;
			
			//Calculate the actual angle
			double angle_between = atan2(vector1(1), vector1(0)) - atan2(vector2(1), vector2(0));
			if (angle_between < 0) angle_between += 2 * M_PI;
			if (angle_orientation < 0) angle_orientation += 2 * M_PI;
			if (angle_between >= 2 * M_PI) angle_between -= 2 * M_PI;
			if (angle_orientation >= 2 * M_PI) angle_orientation -= 2 * M_PI;
			
			//If the angle is more than PI we return the non obtuse angle so we need to redirect it.
// 			if(angle_between > M_PI){
// 				angle_between = (M_PI * 2) - angle_between;
// 				angle_direction - M_PI;
// 				if(angle_direction < 0){
// 					angle_direction = angle_direction + (2 * M_PI);
// 				}
// 			}
			
			//
			angle = angle_between;
			if(angle >= 2 * M_PI){
				throw( std::runtime_error("Making sure that this get never hits. It's in Utils.hpp ndt_feature_finder") );
			}
			if(angle < 0){
				throw( std::runtime_error("Making sure that this get never hits. It's in Utils.hpp ndt_feature_finder") );
			}
			if(angle_orientation >= 2 * M_PI){
				throw( std::runtime_error("Making sure that this get never hits. It's in Utils.hpp ndt_feature_finder") );
			}
			if(angle_orientation < 0){
				throw( std::runtime_error("Making sure that this get never hits. It's in Utils.hpp ndt_feature_finder") );
			}
			
// 			return angle_tmp;
			
		}
		
		/**
		 * @brief return the angle between two ndt cell and all possible direction found using initialized but unoccupied cells.
		 * @param[in] cell1 : first cell
		 * @param[in] cell2 : second cell
		 * @param[in] collision_point : collision point of both cells
		 * @param[in] initialized_cells : initialized_cells around corner
		 * @param[in] angle : actual angle
		 * @param[out] angle_directions : all possible direction of the angle depending on empty cells around it.
		 */
		inline void orientationNDTCells(const perception_oru::NDTMap& map, const perception_oru::NDTCell& cell1, const perception_oru::NDTCell& cell2, const Eigen::Vector3d& collision_point, double angle, double orientation, std::vector<double>& angles, std::vector<double>& angle_orientations){
			
			auto point_center = cell1.getCenter();
			auto point_center2 = cell2.getCenter();
			
// 			std::cout << "center point " << point_center.x << " " << point_center.y << " " << point_center.z << std::endl;
			
// 			std::cout << "angle ndt cell angle multiple" << std::endl;
			double x_cell_size, y_cell_size, z_cell_size;
			map.getCellSizeInMeters(x_cell_size, y_cell_size, z_cell_size);
			
			auto mean_cell1 = cell1.getMean();
			auto mean_cell2 = cell2.getMean();
			
// 			std::cout << "point mean1 " << mean_cell1(0) <<" " << mean_cell1(1) << " " << mean_cell1(2) <<  std::endl;
// 			std::cout << "point mean2 " << mean_cell2(0) <<" " << mean_cell2(1) << " " << mean_cell2(2) <<  std::endl;
			
			auto vector1 = mean_cell1 - collision_point;
			auto vector2 = mean_cell2 - collision_point;
			
// 			Might not work in some specific situations so we check that the direction is more or less in between the gaussians after.
// 			for(int i = 0; i < 2 ; ++i){
// 			
// // 				std::cout << "collision data:  " << x_cell_size << " and i " << i << " orientation " << orientation << std::cos(orientation + (i * M_PI) ) <<  std::endl;
// 				
// 				pcl::PointXYZ forward_point;
// 				forward_point.x = collision_point(0) + (2 * x_cell_size * std::cos(orientation + (i * M_PI) ) );
// 				forward_point.y = collision_point(1) + (2 * x_cell_size *  std::sin(orientation + (i * M_PI) ) );
// 				forward_point.z = 0;
				
// 				std::cout << collision_point(0) << " + (" << x_cell_size << " * " << std::cos(orientation + (i * M_PI) ) << " ) " << std::endl;
// 				std::cout << collision_point(1) << " + (" << x_cell_size << " * " << std::sin(orientation + (i * M_PI) ) << " ) " << std::endl;
				
// 				std::cout << "point forward " << forward_point.x <<" " << forward_point.y << " " << forward_point.z <<  std::endl;
// 				std::cout << "point collision " << collision_point(0) <<" " << collision_point(1) << " " << collision_point(2) <<  std::endl;
				
// 				Eigen::Vector3d forward_point_eigen; forward_point_eigen << forward_point.x, forward_point.y, forward_point.z;
// 				auto orientation_vector = forward_point_eigen - collision_point;
				
// 				std::cout << "orienation vector " << orientation_vector(0) <<" " << orientation_vector(1) << " " << orientation_vector(2) <<  std::endl;
				
// 				perception_oru::NDTCell* cell;
// 				map.getCellAtPoint(forward_point, cell);
				
				/**TEST*/
				perception_oru::NDTCell* cellt;
				map.getCellAtPoint(point_center, cellt);
				assert(cellt != NULL);
				perception_oru::NDTCell* cellt2;
				map.getCellAtPoint(point_center2, cellt2);
				assert(cellt2 != NULL);
				
				int x = point_center.x;
				int y = point_center.y;
				
				perception_oru::NDTCell* cell_collision;
				pcl::PointXYZ collision_pcl;
				collision_pcl.x = collision_point(0) ;
				collision_pcl.y = collision_point(1) ;
				collision_pcl.z = collision_point(2) ;
				map.getCellAtPoint(collision_pcl, cell_collision);
				
				//TODO TAKE CARE OF THAT CASE
				if(cell_collision != NULL){
				
					std::vector< boost::shared_ptr< perception_oru::NDTCell > > cells_gaussian;
					std::vector< boost::shared_ptr< perception_oru::NDTCell > > cells_initialized;
					std::cout << "Get closest cells" << std::endl;
					getClosestCells(map, *cell_collision, 2, cells_gaussian, cells_initialized );
					std::cout << "Get closest cells done : " << cells_initialized.size() << std::endl;
					
					//CHecking all intialized cells
					for(auto it = cells_initialized.begin() ; it != cells_initialized.end() ; ++it){
					std::cout << "Get closest cells center" << std::endl;
						auto init_center = (*it)->getCenter();
					std::cout << "Get closest cells center" << std::endl;
						Eigen::Vector3d init_center_eigen; init_center_eigen << init_center.x, init_center.y, init_center.z;
						auto vector_init = init_center_eigen - collision_point;
						
						//CHecking both possible orientation to see which one is aligned with the uninitialized cell
						for(int i = 0; i < 2 ; ++i){
				
							std::cout << "collision data:  " << x_cell_size << " and i " << i << " orientation " << orientation << std::cos(orientation + (i * M_PI) ) <<  std::endl;
							
							pcl::PointXYZ forward_point;
							forward_point.x = collision_point(0) + (2 * x_cell_size * std::cos(orientation + (i * M_PI) ) );
							forward_point.y = collision_point(1) + (2 * x_cell_size *  std::sin(orientation + (i * M_PI) ) );
							forward_point.z = 0;
							
							std::cout << collision_point(0) << " + (" << x_cell_size << " * " << std::cos(orientation + (i * M_PI) ) << " ) " << std::endl;
							std::cout << collision_point(1) << " + (" << x_cell_size << " * " << std::sin(orientation + (i * M_PI) ) << " ) " << std::endl;
							
							std::cout << "point forward " << forward_point.x <<" " << forward_point.y << " " << forward_point.z <<  std::endl;
							std::cout << "point collision " << collision_point(0) <<" " << collision_point(1) << " " << collision_point(2) <<  std::endl;
							
							Eigen::Vector3d forward_point_eigen; forward_point_eigen << forward_point.x, forward_point.y, forward_point.z;
							auto orientation_vector = forward_point_eigen - collision_point;
							
							
							//Calculating the angle between both vector
							Eigen::Vector2d vector_init_2d; vector_init_2d << vector_init(0), vector_init(1);
							Eigen::Vector2d  orientation_vector_2d; orientation_vector_2d <<  orientation_vector(0),  orientation_vector(1);
// 							double angle_init_orientation = getAngleDirected(vector_init_2d, orientation_vector_2d);
							double angle_init_orientation = getAngle(vector_init, orientation_vector);
							
// 							std::cout << angle_init_orientation << " == " << angle_init_orientation_3dtest << std::endl;
// 							assert(angle_init_orientation == angle_init_orientation_3dtest);
// 							double angle_init_orientation = atan2(vector_init(1), vector_init(0)) - atan2(orientation_vector(1), orientation_vector(0));
							
							//Angle between 0 and 2pi
							if (angle_init_orientation < 0) angle_init_orientation += 2 * M_PI;
							if (angle_init_orientation >= 2 * M_PI) angle_init_orientation -= 2 * M_PI;
							//Angle between 0 and pi Because we don't care about orientation as in how close they are
							if (angle_init_orientation >= M_PI) angle_init_orientation = (2 * M_PI) - angle_init_orientation;
							
							
							//Are they pointing the same direction? Thus is the angle between the orientation and the initialised cell less than 0.78rad (45deg)
							if(angle_init_orientation < 0.78){
								
								//Check that the direction is in the middle of the gaussians
								double angle_between = atan2(vector1(1), vector1(0)) - atan2(orientation_vector(1), orientation_vector(0));
								double angle_between2 = atan2(orientation_vector(1), orientation_vector(0)) - atan2(vector2(1), vector2(0) );
								
								
								
								//Angle between 0 and 2pi
								if (angle_between < 0) angle_between += 2 * M_PI;
								if (angle_between2 < 0) angle_between2 += 2 * M_PI;
								if (angle_between >= 2 * M_PI) angle_between -= 2 * M_PI;
								if (angle_between2 >= 2 * M_PI) angle_between2 -= 2 * M_PI;
								
								//Angle between 0 and pi
								if (angle_between >= M_PI) angle_between = (2 * M_PI) - angle_between;
								if (angle_between2 >= M_PI) angle_between2 = (2 * M_PI) - angle_between2;
								
								//Actual angle between the gaussian and passing by the orientation.
								double angle_final = angle_between + angle_between2;
								while(angle_final >= 2 * M_PI){
									angle_between -= 2 * M_PI;
								}
								
								std::cout << " ANGLE "  << atan2(vector1(1), vector1(0)) - atan2(orientation_vector(1), orientation_vector(0)) << " and " << std::abs( atan2(vector2(1), vector2(0)) - atan2(orientation_vector(1), orientation_vector(0)) ) << std::endl;
								std::cout << "FINAL ANGLES " << angle_between << " " << angle_between2<<std::endl;
								
								//If the angles are roughly equal then it's in the middle just fine
								if(	(angle_between < ( angle_between2 + 0.1 ) ) && ( angle_between > (angle_between2 - 0.1 ) ) ){
									std::cout << "Pushing back " << orientation + (i * M_PI)<< " with " << angle_between << " " << angle_between2 << std::endl;
									angle_orientations.push_back(orientation + (i * M_PI));
									angles.push_back(angle_final);
								}
								else{
									std::cout << "Wrong angle " << angle_between << " " << angle_between2 << std::endl;
								}
							}
							else{
								std::cout << "NOT ALIGNED" << std::endl;
							}
							
						}
					}
				
				}
				
// 				std::cout << "check" << std::endl; 
				
				//Change the check on cell initialized by checking all cells between the gaussians that gave the corner and passing by the orientation are initialized.
// 				if(cell != NULL){
// 					
// 					
// 					
// 					auto center_forward = cell->getCenter();
// 					
// 					std::cout << "center point1 " << point_center.x << " " << point_center.y << " " << point_center.z << std::endl;
// 					std::cout << "center point2 " << point_center2.x << " " << point_center2.y << " " << point_center2.z << std::endl;
// 					std::cout << "center forward " << center_forward.x << " " << center_forward.y << " " << center_forward.z << std::endl;
// 					
// 					if(center_forward.x == point_center.x && center_forward.y == point_center.y && center_forward.z == point_center.z){
// 						std::cout << "SAME CELL AS CELL1" << std::endl;
// 					}
// 					else if(center_forward.x == point_center2.x && center_forward.y == point_center2.y && center_forward.z == point_center2.z){
// 						std::cout << "SAME CELL AS CELL2" << std::endl;
// 					}
// 					if(cell_collision != NULL){
// 						
// 						auto collision_cell_pcl = cell_collision->getCenter();
// 						std::cout << "center forward " << collision_cell_pcl.x << " " << collision_cell_pcl.y << " " << collision_cell_pcl.z << std::endl;
// 						if(center_forward.x == collision_cell_pcl.x && center_forward.y == collision_cell_pcl.y && center_forward.z == collision_cell_pcl.z){
// 							std::cout << "SAME CELL AS COLLISION CELL" << std::endl;
// 						}
// 					}
// 					else{
// 						std::cout << "Cell collision is NULL " << std::endl;
// 					}
// 				//Good cell
// // 					if(cell->isEmpty == 1){
// 						if(cell->hasGaussian_ == false){
// 							
// 							//Check that the direction in the middle of the angle
// 							double angle_between = atan2(vector1(1), vector1(0)) - atan2(orientation_vector(1), orientation_vector(0));
// 							double angle_between2 = atan2(orientation_vector(1), orientation_vector(0)) - atan2(vector2(1), vector2(0) );
// 							
// 							//Angle between 0 and 2pi
// 							if (angle_between < 0) angle_between += 2 * M_PI;
// 							if (angle_between2 < 0) angle_between2 += 2 * M_PI;
// 							if (angle_between >= 2 * M_PI) angle_between -= 2 * M_PI;
// 							if (angle_between2 >= 2 * M_PI) angle_between2 -= 2 * M_PI;
// 							
// 							//Angle between 0 and pi
// 							if (angle_between >= M_PI) angle_between = (2 * M_PI) - angle_between;
// 							if (angle_between2 >= M_PI) angle_between2 = (2 * M_PI) - angle_between2;
// 							
// 							
// 							std::cout << " ANGLE "  << atan2(vector1(1), vector1(0)) - atan2(orientation_vector(1), orientation_vector(0)) << " and " << std::abs( atan2(vector2(1), vector2(0)) - atan2(orientation_vector(1), orientation_vector(0)) ) << std::endl;
// 							std::cout << "FINAL ANGLES " << angle_between << " " << angle_between2<<std::endl;
// 							
// 							if(	(angle_between < ( angle_between2 + 0.1 ) ) && ( angle_between > (angle_between2 - 0.1 ) ) ){
// 								std::cout << "Pushing back " << orientation + (i * M_PI)<< " with " << angle_between << " " << angle_between2 << std::endl;
// 								angle_orientations.push_back(orientation + (i * M_PI));
// 							}
// 							else{
// 								std::cout << "Wrong angle " << angle_between << " " << angle_between2 << std::endl;
// 							}
// 						}
// 						else{
// 							std::cout << "Cell has gaussian" << std::endl;
// 						}
// // 					}
// // 					else{
// // 						std::cout << "Cell is empty is not 1 " << std::endl;
// // 					}
// 				}
// 				else{
// 					std::cout << "CELL NULL" << std::endl;
// 				}
// 				
// 				std::cout << "check end" << std::endl; 
// 			}
// 			
// // 			assert(angle_orientations.size() >= 1);
// 			
// // 			exit(0);
// 			//CHeck that line between the mean does not contain any Gaussian unless it's the corner cell itself. Only work for the small angle
// // 			double x_cell_size, y_cell_size, z_cell_size;
// // 			map.getCellSizeInMeters(x_cell_size, y_cell_size, z_cell_size);
// // 			
// // 			auto mean_cell1 = cell1.getMean();
// // 			auto mean_cell2 = cell2.getMean();
// // 			
// // 			perception_oru::NDTCell* cell_collision;
// // 			pcl::PointXYZ collision_point_cell;
// // 			collision_point_cell.x = collision_point(0);
// // 			collision_point_cell.y = collision_point(1);
// // 			collision_point_cell.z = 0;
// // 			map.getCellAtPoint(collision_point, cell_collision);
// // 			
// // 			Eigen::Vector2d mean_cell1_2d; mean_cell1_2d << mean_cell1(0), mean_cell1(1);
// // 			Eigen::Vector2d mean_cell2_2d; mean_cell2_2d << mean_cell2(0), mean_cell2(1);
// // 			
// // 			Eigen::Vector2d v = mean_cell1_2d - mean_cell2_2d;
// // 			Eigen::Vector2d u = v / v.norm();
// // 			
// // 			//Check for all point along the line at a distance of half a cell every time:
// // 			double distance = x_cell_size / 2;
// // 			Eigen::Vector2d p_test = mean_cell2_2d + (distance * u);
// // 			int i = 1 ;
// // 			bool flag_empty = true;
// // 			while( (p_test -  mean_cell1_2d).norm() < (p_test - mean_cell2_2d).norm() ){
// // 				p_test = mean_cell2_2d + (distance * i * u);
// // 				
// // 				pcl::PointXYZ forward_point;
// // 				forward_point.x = p_test(0);
// // 				forward_point.y = p_test(1);
// // 				forward_point.z = 0;
// // 				
// // 				perception_oru::NDTCell* cell;
// // 				map.getCellAtPoint(forward_point, cell);
// // 				
// // 				//Good
// // 				if(cell->isEmpty != 1 && cell->hasGaussian_ != false 
// // 					&& cell->getCenter() != cell1.getCenter()
// // 					&& cell->getCenter() != cell2.getCenter()
// // 					&& cell->getCenter() != cell_collision.getCenter()
// // 				){
// // 					flag_empty = false;
// // 				}
// // 			}
			

			
		}
		
		/**
		 * @brief return the angle between two ndt cell and all possible orientations found using initialized but unoccupied cells.
		 * @param[in] cell1 : first cell
		 * @param[in] cell2 : second cell
		 * @param[in] collision_point : collision point of both cells
		 * @param[in] initialized_cells : initialized_cells around corner
		 * @param[out] angle : actual angle
		 * @param[out] angle_directions : all possible direction of the angle depending on empty cells around it.
		 */
		inline void angleNDTCells(const perception_oru::NDTMap& map, const perception_oru::NDTCell& cell1, const perception_oru::NDTCell& cell2, const Eigen::Vector3d& collision_point, double& angle, std::vector<double>& angles, std::vector<double>& angle_orientations){
			
			std::cout << "angle ndt cell multiple" << std::endl;
			double orientation;
			angleNDTCells(cell1, cell2, collision_point, angle, orientation);
			orientationNDTCells(map, cell1, cell2, collision_point, angle, orientation, angles, angle_orientations);
			
		}
		
		
		/**
		* @brief return all the cells around cell that possess a gaussian and are maximum the resolution of the map away.
		*/
		void getClosestCells(const perception_oru::NDTMap& map, const perception_oru::NDTCell& cell, int neig_size, std::vector< boost::shared_ptr< perception_oru::NDTCell > >& cells_gaussian, std::vector< boost::shared_ptr< perception_oru::NDTCell > >& cells_initialized){
// 			std::cout << "RADIUS " << radius << "*" << neig_size << std::endl;
			double x_cell_size, y_cell_size, z_cell_size;
			map.getCellSizeInMeters(x_cell_size, y_cell_size, z_cell_size);
// 			std::cout << "RADIUS " << radius << "*" << neig_size << std::endl;
			double max = x_cell_size;
			if(max < y_cell_size){
				max = y_cell_size;
			}
			if(max < z_cell_size){
				max = z_cell_size;
			}
// 			std::cout << "RADIUS " << radius << "*" << neig_size << std::endl;
			auto lazygrid = dynamic_cast<perception_oru::LazyGrid*>(map.getMyIndex());
// 			std::cout << "RADIUS " << radius << "*" << neig_size << std::endl;
			pcl::PointXYZ point = cell.getCenter();
			double radius = max;
			std::vector< boost::shared_ptr< perception_oru::NDTCell > > cells;
			
			std::cout << "RADIUS " << radius << "*" << neig_size << std::endl;
			
		// 	lazygrid->getNeighborsShared(point, radius * neig_size, cells);
			//Get all closest cell and then classify to keep the one with gaussians and the initialized ones
			cells = lazygrid->getClosestNDTCellsShared(point, neig_size, false);
			
			std::cout << "OUT /*SIZE*/ " << cells.size() << " " << max <<std::endl;
		// 	exit(0);
			
		// 	std::vector<boost::shared_ptr< perception_oru::NDTCell> > cells_gaussian;
			cells_gaussian.clear();
			cells_initialized.clear();
			
			for(size_t j = 0 ; j < cells.size() ; ++j){
		// 		std::cout << "Celly " << cells[j]->getMean() << std::endl;
				if(cells[j]->hasGaussian_ == true){
					std::cout << "PUSHED" << std::endl;
					cells_gaussian.push_back(cells[j]);
				}
				else{
					std::cout << "PUSHED init" << std::endl;
					cells_initialized.push_back(cells[j]);
				}
			}
			
		// 	std::cout << cells_gaussian.size() << " == " << cells.size() << std::endl;
		// 	assert(cells_gaussian.size() == cells.size());
			
		// 	std::cout << "Center " << cell.getMean() << std::endl;
		// 	std::cout << "Nei " << cells_out.size() << " " << max <<std::endl;
			
		// 	return cells_gaussian;
		}
		 
		
		
		
	}
	
}

#endif
