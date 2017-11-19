#ifndef DAS_NDTCORNER_10072016
#define DAS_NDTCORNER_10072016

#include <math.h>
#include <pcl/point_types.h>
#include "ndt_corner_bundle.hpp"

namespace perception_oru{
	
	namespace ndt_feature_finder{
		
		
		class NDTCorner{
			
		protected:
			
			
			double _x_cell_size, _y_cell_size, _z_cell_size;
			//CHange to NDTCornerBundle
			std::vector< NDTCornerBundle > _corners;
// 			std::vector< Eigen::Vector3d > _corners_position;
// 			std::vector< cv::Point2d > _opencv_corners;
// 			std::vector< cv::Point2d > _opencv_corners_position;
			/**
			 * Pair of angle size between edges + direction of each corner
			 */
// 			std::vector<std::pair<double,double> > _angles;
			int _neighbor_size;
			
		public:
			NDTCorner() : _neighbor_size(2){};
			
			/**
			 * @brief Return the orientation of the cell in deg compared to [SOMETHING]
			 */
// 			double getCellOrientation(const perception_oru::NDTCell& cell) const ;
			
			/**
			 * @brief return true if the cell correspond to a corner
			 */
			bool cellIsCorner(const perception_oru::NDTMap& map, const perception_oru::NDTCell& cell, const std::vector< boost::shared_ptr< perception_oru::NDTCell > >& allCells, NDTCornerBundle& corner) ;
			
			/**
			 * @brief calculate and return a vector of all corners in ndt_map. Return the direction of the smallest angle of the corner
			 */
			std::vector< NDTCornerBundle > getAllCorners(const perception_oru::NDTMap& map);
			
			
			/**
			 * @brief calculate and return a vector of all corners in ndt_map. Use the robot pose to detect correct robot orientation
			 */

			std::vector< NDTCornerBundle > getAllCorners(const perception_oru::NDTMap& map, const Eigen::Vector3d& robot_pose) ;
			
			/**
			 * @brief return all the cells around cell that possess a gaussian and are maximum the resolution of the map away.
			 */
// 			void getClosestCells(const perception_oru::NDTMap& map, const perception_oru::NDTCell& cell, int neig_size, std::vector< boost::shared_ptr< perception_oru::NDTCell > >& cells_gaussian, std::vector< boost::shared_ptr< perception_oru::NDTCell > >& cells_initialized) const ;
			
			/**
			 * @brief return the angle between two Vectors.
			 */
// 			double getAngle(const Eigen::Vector3d& vec, const Eigen::Vector3d& vec2) const;
			
			/**
			 * @brief return the index of the biggest eigen vector in the 2D plane xy
			 */
// 			int getBiggestEigenVector2D(const perception_oru::NDTCell& cell, Eigen::Vector3d& eigenval, Eigen::Matrix3d& eigenvec) const;
			
			/**
			 * @brief return the angle width and direction of all corner detected by it
			 */
			
			
			
			
			void clear(){
				_x_cell_size = _y_cell_size = _z_cell_size = 0;
				_corners.clear();
// 				_corners_position.clear();
// 				_opencv_corners.clear();
// 				_opencv_corners_position.clear();
// 				_angles.clear();
			}
			
			size_t size() const {return _corners.size();}
			std::vector< NDTCornerBundle >& getCorners(){return _corners;}
			const std::vector< NDTCornerBundle >& getCorners() const {return _corners;}
			
			void setNeighborSize(int n){_neighbor_size = n;}
			int getNeighborSize() const {return _neighbor_size;}
// 			std::vector< cv::Point2d >& getCvCorners(){return _opencv_corners;}
// 			const std::vector< cv::Point2d >& getCvCorners() const {return _opencv_corners;}
// 			std::vector< Eigen::Vector3d >& getAccurateCorners(){return _corners_position;}
// 			const std::vector< Eigen::Vector3d >& getAccurateCorners() const {return _corners_position;}
// 			std::vector< cv::Point2d >& getAccurateCvCorners(){return _opencv_corners_position;}
// 			const std::vector< cv::Point2d >& getAccurateCvCorners() const {return _opencv_corners_position;}
// 			const std::vector<std::pair<double,double> >& getAngles() const {assert(_angles.size() ==_corners.size()); return _angles;}
// 			std::vector<std::pair<double,double> >& getAngles(){assert(_angles.size() == _corners.size()); return _angles;}
			
			
// 			void exportCorners(std::ostream& out){
// 				for(size_t i = 0 ; i < _opencv_corners.size() ; ++i){
// 					std::cout << _opencv_corners[i].x << " " << _opencv_corners[i].y << std::endl;
// 					out << _opencv_corners[i].x << " " << _opencv_corners[i].y << std::endl;
// 				}
// 			}
// 			void exportAccurateCorners(std::ostream& out){
// 				for(size_t i = 0 ; i < _opencv_corners_position.size() ; ++i){
// 					std::cout << _opencv_corners_position[i].x << " " << _opencv_corners_position[i].y << std::endl;
// 					out << _opencv_corners_position[i].x << " " << _opencv_corners_position[i].y << std::endl;
// 				}
// 			}
			
// 			void readCorners(std::ifstream& in){
// 				while(true){
// 					cv::Point2d p;
// // 					std::cout << "f"<<std::endl;
// // 					if(in.eof()) break;
// 					in >> p.x;
// // 					std::cout << "s : "<< p.x << std::endl;
// // 					if(in.eof()) break;
// 					in >> p.y;
// // 					std::cout << "Pushing " << p << std::endl;
// 					if(in.eof()) break;
// 					_opencv_corners.push_back(p);
// 				}
// 			}
// 			
// 			void readAccurateCorners(std::ifstream& in){
// 				while(true){
// 					cv::Point2d p;
// // 					std::cout << "f"<<std::endl;
// // 					if(in.eof()) break;
// 					in >> p.x;
// // 					std::cout << "s : "<< p.x << std::endl;
// // 					if(in.eof()) break;
// 					in >> p.y;
// // 					std::cout << "Pushing " << p << std::endl;
// 					if(in.eof()) break;
// 					_opencv_corners_position.push_back(p);
// 				}
// 			}
			
		private:
			
			
			bool gotAngledNDT(const perception_oru::NDTMap& map, const std::vector< boost::shared_ptr< perception_oru::NDTCell > >& neighbor, const std::vector< boost::shared_ptr< perception_oru::NDTCell > >& cells_initialized, NDTCornerBundle& corner) ;
			/**
			 * @brief remove doubles of corners. ATTENTION : it does not update the openCV corners
			 */
			void clearAccurateCorner();
			/**
			 * @brief transform ndt_cell mean and eigen point to cv::Point2d
			 */
// 			void toOpenCV();
			/**
			 * @brief calculate the angles of each corners
			 */
			void calculateAngles(const perception_oru::NDTMap& map);
			
			std::vector< perception_oru::NDTCell* > getCellsPointingToward(std::vector< perception_oru::NDTCell* >& neighbor, const perception_oru::NDTCell& cell) const;
			bool AlignedNDT(const std::vector< perception_oru::NDTCell* >& neighbor, const perception_oru::NDTCell& cell);
			
		};
		
	}
}



#endif
