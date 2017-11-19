#ifndef DAS_NDTCORNERBUNDLE_13072016
#define DAS_NDTCORNERBUNDLE_13072016

#include <math.h>
#include "ndt_map/ndt_map.h"

#include "ndt_cell_2d_utils.hpp"

namespace perception_oru{
	
	namespace ndt_feature_finder{

		/**
		 * A class that combines information about a found corner:
		 * The cells that gave it
		 * The _angle
		 * The _orientation
		 * The variance: eigen vector and values + mean position
		 */
		class NDTCornerBundle{
		protected:
			
			struct EigenValVec{
				Eigen::Vector3d eigenvec;
				double eigenval;
				Eigen::Vector3d NormEigen(){
					return eigenvec * eigenval;
				}
			};
			
			bool _mean_set;
			
			std::vector<boost::shared_ptr< perception_oru::NDTCell > > _cell1;
			std::vector<boost::shared_ptr< perception_oru::NDTCell > > _cell2;
			std::vector<double> _angle;
			std::vector<double> _orientation;
			
			Eigen::Matrix3d _eigen_vector;
			Eigen::Vector3d _eigen_values;
			Eigen::Vector3d _mean;
			
		public:
		
			NDTCornerBundle(): _mean_set(false){}
			NDTCornerBundle(const Eigen::Vector3d& m) :
				_mean(m), _mean_set(true)
			{}
			void push_back_cell1(const boost::shared_ptr< perception_oru::NDTCell >& c1){_cell1.push_back(c1);}
			void push_back_cell2(const boost::shared_ptr< perception_oru::NDTCell >& c2){_cell2.push_back(c2);}
			void setMean(const Eigen::Vector3d& m){
				_mean_set = true;
				_mean = m;
				
			}
			void setAngles(const std::vector<double>& a){_angle = a;}
			void setOrientations(const std::vector<double>& d){_orientation = d;}
			cv::Point2d getMeanOpenCV(){
				cv::Point2d p;
				p.x = _mean(0);
				p.y = _mean(1);
				return p;
			}
			
			const Eigen::Vector3d& getMean() const {return _mean;}
			Eigen::Vector3d getMean(){return _mean;}
			const Eigen::Matrix3d& getEigenVectors() const {return _eigen_vector;}
			const Eigen::Vector3d& getEigenValues() const {return _eigen_values;}
			Eigen::Matrix3d getEigenVectors() {return _eigen_vector;}
			Eigen::Vector3d getEigenValues() {return _eigen_values;}
			const std::vector<double>& getOrientations() const {return _orientation;}
			const std::vector<double>& getAngles() const {return _angle;}
			std::vector<boost::shared_ptr< perception_oru::NDTCell > >& getCells1(){return _cell1;}
			const std::vector<boost::shared_ptr< perception_oru::NDTCell > >& getCells1() const {return _cell1;}
			std::vector<boost::shared_ptr< perception_oru::NDTCell > >& getCells2(){return _cell2;}
			const std::vector<boost::shared_ptr< perception_oru::NDTCell > >& getCells2() const {return _cell2;}
			
			void inverseDistanceWeighting(){
				
			}
			
			void gaussian(){
				assert(_mean_set == true);
				
				auto cell1 = _cell1[0];
				auto cell2 = _cell2[0];
				
				EigenValVec biggest_cell1, smallest_cell1, biggest_cell2, smallest_cell2;
				
				getEigenVectors(*cell1, biggest_cell1, smallest_cell1);
				getEigenVectors(*cell2, biggest_cell2, smallest_cell2);
				
				Eigen::Vector3d side_point1_cell1;
				Eigen::Vector3d side_point2_cell1;
				side_point1_cell1 = cell1->getMean() + smallest_cell1.NormEigen();
				side_point2_cell1 = cell1->getMean() - smallest_cell1.NormEigen();
				
				Eigen::Vector3d side_point1_cell2;
				Eigen::Vector3d side_point2_cell2;
				side_point1_cell2 = cell2->getMean() + smallest_cell2.NormEigen();
				side_point2_cell2 = cell2->getMean() - smallest_cell2.NormEigen();
				
// 				std::cout << "All points " << side_point1_cell1 << " \n\n " << side_point1_cell2 << " \n\n " << side_point2_cell2 << std::endl <<  smallest_cell2.NormEigen() << std::endl;

				//First collision line
				auto collision = collisionRay(biggest_cell1.eigenvec, side_point1_cell1, biggest_cell2.eigenvec, side_point1_cell2);
				
// 				std::cout << "Collision " << collision << " nesxtr " << biggest_cell1.eigenvec << " nesxtr " << side_point1_cell1 << " nesxtr " << biggest_cell2.eigenvec << " nesxtr " << side_point1_cell2 << " " << std::endl;
				//Second collision line with only one of the two line moved. It doesn't matter which but it needs to be only one.
				auto collision1 = collisionRay(biggest_cell1.eigenvec, side_point1_cell1, biggest_cell2.eigenvec, side_point2_cell2);
// 				std::cout << "Collision2 " << collision1 << " nesxtr " << biggest_cell1.eigenvec << " nesxtr " << side_point1_cell1 << " nesxtr " << biggest_cell2.eigenvec << " nesxtr " << side_point2_cell2 << " " << std::endl;
				
// 				std::cout << "Mean " << _mean << std::endl;
// 				std::cout << "RES1 \n " << collision - _mean << std::endl;
// 				std::cout << "RES2\n " << collision1 - _mean << std::endl;
				
				Eigen::Vector3d v1(0,0,0);
				_eigen_vector << collision - _mean , collision1 - _mean, v1 ;
				_eigen_values << 1, 1, 1;
				
// 				std::cout << "FINAl\n " << _eigen_vector << std::endl;
				
			}
			
		private:
			
			void getEigenVectors(const perception_oru::NDTCell& cell, EigenValVec& biggest, EigenValVec& smallest) const {
				
				Eigen::Vector3d eigenval;
				Eigen::Matrix3d eigenvec;
				getEigenVectors2D(cell, eigenval, eigenvec);
			// 	std::cout << "Eigen sorted" << std::endl;
				if(eigenval(1) > eigenval(0)){
					biggest.eigenvec = eigenvec.col(1);
					smallest.eigenvec = eigenvec.col(0);
					biggest.eigenval = eigenval(1);
					smallest.eigenval = eigenval(0);
				}
				else{
					biggest.eigenvec = eigenvec.col(0);
					smallest.eigenvec = eigenvec.col(1);
					biggest.eigenval = eigenval(0);
					smallest.eigenval = eigenval(1);
				}
			}
			
		};
		
	}
}



#endif
