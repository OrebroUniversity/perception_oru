#ifndef DAS_UTILS_12072016
#define DAS_UTILS_12072016

#include "Eigen/Core"
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"

#include "ndt_map/ndt_map.h"

namespace perception_oru{
	namespace ndt_feature_finder{
		
		inline std::string type2str(int type) {
			std::string r;

			uchar depth = type & CV_MAT_DEPTH_MASK;
			uchar chans = 1 + (type >> CV_CN_SHIFT);

			switch ( depth ) {
				case CV_8U:  r = "8U"; break;
				case CV_8S:  r = "8S"; break;
				case CV_16U: r = "16U"; break;
				case CV_16S: r = "16S"; break;
				case CV_32S: r = "32S"; break;
				case CV_32F: r = "32F"; break;
				case CV_64F: r = "64F"; break;
				default:     r = "User"; break;
			}

			r += "C";
			r += (chans+'0');

			return r;
		}
		
		/**
		 * @brief return the abgle between two vector in rad. clockwise
		 */
		inline double getAngle(const Eigen::Vector3d& base, const Eigen::Vector3d& toward)
		{

		// 	std::cout << "Minor Axis " << std::endl << vec << std::endl;
		// 	std::cout << "Compared to  " << std::endl << vec2 << std::endl;
			//dot product with vertical axis gives us the angle
			double d = base.dot(toward);
			double l = base.norm() * toward.norm();
			
		// 	std::cout << " d : " << d << " norm " << l << std::endl;
			double ac = d/l;
			ac = std::acos(ac);
			return ac;

		}
		
		/**
		 * @brief return the abgle between two vector. Vector are considered centered on 0
		 */
		inline double getAngleDirected(const Eigen::Vector2d& base, const Eigen::Vector2d& toward)
		{
		
			double ac = atan2(toward(1),toward(0)) - atan2(base(1), base(0));
			return ac;

		}
		
		/**
		 * @brief return the abgle between two vector.
		 */
		inline double getAngleG(const Eigen::VectorXd& base, const Eigen::VectorXd& toward)
		{

			if(base == toward ){
// 				std::cout << "vectors base : " << base << std::endl << " toward : " << toward << std::endl;
				//LINE REMOVED TO ALLOW ASSERT
// 				throw std::runtime_error("Point are the same. no angle between them");
			}
		// 	std::cout << "Minor Axis " << std::endl << vec << std::endl;
		// 	std::cout << "Compared to  " << std::endl << vec2 << std::endl;
			//dot product with vertical axis gives us the angle
			double d = base.dot(toward);
			double l = base.norm() * toward.norm();
			
// 			std::cout << " d : " << d << " norm " << l << std::endl;
			double ac = d/l;
			ac = std::floor( ac * 10 + 0.5)/10;
// 			std::cout << " AC " << ac << " angle has to be " << std::acos(ac) << " " << std::acos(1) <<std::endl;
			ac = std::acos(ac);
// 			std::cout << " AC " << ac << std::endl;
			
// 			if(ac > 2*M_PI || ac < 0){
// 				std::cout << "vectors base : " << base << std::endl << " toward : " << toward << std::endl;
// 				std::cout << " d : " << d << " norm " << l << std::endl;
// 				std::cout << "Bad angle " << ac << std::endl;
// 				exit(0); 
// 			}
// 			std::cout << "Angle " << ac << std::endl;
			assert(ac <= 2*M_PI);
			assert(ac >= 0);
			
			return ac;

		}
		
		
		
		inline void EigenSort2D( Eigen::Vector3d &eigenvalues, Eigen::Matrix3d &eigenvectors ){
			Eigen::Vector3d ez;
			ez << 0.0, 0.0, 1.0;
			double dist = -1;
			int z_id;
			Eigen::Vector3d normal_vec;
			Eigen::Vector3d tangent_vec;
			Eigen::Vector3d xy_vec;
			double normal_val;
			double tangent_val;
			double xy_val;
			for(int i = 0; i < 3; ++i){
				
// 				std::cout << "Vect : " << eigenvectors.col(i) << std::endl << " dot " << fabs(eigenvectors.col(i).dot(ez)) << std::endl;
				if(dist == -1 ){
					z_id = i;
					dist = fabs(eigenvectors.col(i).dot(ez));
				}
				if(fabs(eigenvectors.col(i).dot(ez)) > dist){
					z_id = i;
					dist = fabs(eigenvectors.col(i).dot(ez));
				}
			}
			xy_vec=eigenvectors.col(z_id);
			xy_val=eigenvalues(z_id);
			if(z_id == 0){
				normal_vec = eigenvectors.col(1);
				tangent_vec = eigenvectors.col(2);
				normal_val = eigenvalues(1);
				tangent_val = eigenvalues(2);
			}
			if(z_id == 1){
				normal_vec = eigenvectors.col(0);
				tangent_vec = eigenvectors.col(2);
				normal_val = eigenvalues(0);
				tangent_val = eigenvalues(2);
			}
			if(z_id == 2){
				normal_vec = eigenvectors.col(0);
				tangent_vec = eigenvectors.col(1);
				normal_val = eigenvalues(0);
				tangent_val = eigenvalues(1);
			}
			if(normal_val > tangent_val){
				double temp_val=normal_val;
				normal_val=tangent_val;
				tangent_val=temp_val;
				Eigen::Vector3d temp_vec=normal_vec;
				normal_vec=tangent_vec;
				tangent_vec=temp_vec;
			}

			eigenvectors.col(0)=tangent_vec;
			eigenvalues(0)=tangent_val;
			eigenvectors.col(1)=normal_vec;
			eigenvalues(1)=normal_val;
			eigenvectors.col(2)=xy_vec;
			eigenvalues(2)=xy_val;

		}

	
		inline auto distancePoints (const cv::Point p1, const cv::Point p2) -> double{
			
			double x_max = p1.x, x_min = p2.x;
			if(x_max < x_min){
				x_max = p2.x;
				x_min = p1.x;
			}
			double y_max = p1.y, y_min = p2.y;
			if(y_max < y_min){
				y_max = p2.y;
				y_min = p1.y;
			}
			
			double tmpp = x_max - x_min;
			if(tmpp > 1000000){
				tmpp = 1000000;
			}
			
			double x_tmp = tmpp*tmpp;
	// 					assert(x_tmp >=0);
			
			tmpp = (y_max - y_min);
			if(tmpp > 1000000){
				tmpp = 1000000;
			}
			double y_tmp = tmpp * tmpp;
	// 					assert(y_tmp >= 0);
			double dst = x_tmp + y_tmp;
	// 					assert(dst >= 0);
	// 					std::cout << "points : " << p1 << " " << p2 << std::endl;
	// 					std::cout << dst << std::endl;
			
	// 					if(p1 == p2){
	// 						exit(0);
	// 					}
			if(dst > 0){
				return std::sqrt(dst);
			}
			return 0;
		};
		
		inline auto getSmallestDistance (const cv::Point2d& pt, const std::vector<cv::Point2d>& model, double& distance) -> size_t{
			size_t i, index = -1 ;
			distance = -1;
			for(i = 0 ; i < model.size() ; ++i){
	// 			std::cout << "TEST " <<distancePoints(pt, model[i])<<std::endl;
				assert(distancePoints(pt, model[i]) >= 0);
				
				if(distance == -1){
					distance = distancePoints(pt, model[i]);
					index = i;
				}
				else{
	// 				std::cout << distance << " > " << distancePoints(pt, model[i]) << std::endl;
					if(distance > distancePoints(pt, model[i]) ){
						distance = distancePoints(pt, model[i]);
						index = i;
					}
					
				}
			}
	// 					std::cout << "return index " << index << std::endl;
			return index;
		};
		
		inline Eigen::Vector3d collisionRay(
			const Eigen::Vector3d& ray_direction, 
			const Eigen::Vector3d& ray_point, 
			const Eigen::Vector3d& ray_direction_second, 
			const Eigen::Vector3d& ray_point_second){
			
			assert(ray_direction != Eigen::Vector3d(0,0,0));
			assert(ray_direction_second != Eigen::Vector3d(0,0,0));
			
			Eigen::Matrix3d A ;//THree rows and 2 cols ;
			Eigen::Vector3d b;
			
			A << - ray_direction(0), ray_direction_second(0), 0,
				- ray_direction(1), ray_direction_second(1), 0,
				- ray_direction(2), ray_direction_second(2), 0;
			b << ray_point(0) - ray_point_second(0),
				ray_point(1) - ray_point_second(1),
				ray_point(2) - ray_point_second(2);
				
// 			std::cout << "Here is the matrix A:\n" << A << std::endl;
// 			std::cout << "Here is the vector b:\n" << b << std::endl;
			Eigen::Vector3d t_n = A.colPivHouseholderQr().solve(b);
// 			std::cout << "The solution is:\n" << t_n << std::endl;
			
			//Calculate collision point using running parameter t_n
// 			Eigen::Vector3d x;
// 			x << std::floor( (( ray_direction(0) * t_n(0) ) + ray_point(0) ) * 10 + 0.5)/10,
// 				 std::floor( (( ray_direction(1) * t_n(1) ) + ray_point(1) ) * 10 + 0.5)/10,
// 				 std::floor( (( ray_direction(2) * t_n(2) ) + ray_point(2) ) * 10 + 0.5)/10;
			
			Eigen::Vector3d x;
// 			x << std::floor( (( ray_direction(0) * t_n(0) ) + ray_point(0) ) * 10 + 0.5)/10,
// 				 std::floor( (( ray_direction(1) * t_n(0) ) + ray_point(1) ) * 10 + 0.5)/10,
// 				 std::floor( (( ray_direction(2) * t_n(0) ) + ray_point(2) ) * 10 + 0.5)/10;
				 
			x <<  ( ray_direction(0) * t_n(0) ) + ray_point(0) ,
				  ( ray_direction(1) * t_n(0) ) + ray_point(1) ,
				  ( ray_direction(2) * t_n(0) ) + ray_point(2) ;
				 
// 			Eigen::Vector3d x_second;
// 			x_second << std::floor( (( ray_direction_second(0) * t_n(1) ) + ray_point_second(0)) * 10 + 0.5)/10,
// 						std::floor( (( ray_direction_second(1) * t_n(1) ) + ray_point_second(1)) * 10 + 0.5)/10,
// 						std::floor( (( ray_direction_second(2) * t_n(1) ) + ray_point_second(2)) * 10 + 0.5)/10;
				 
// 			std::cout <<std::endl << x << " and \n " << x2 << " and\n " << x_second << std::endl;
			
			//TODO: Make this test better :S
// 			assert(x == x_second);
				 
			return x;
			
		}
		
		
		
	}
	
}

#endif