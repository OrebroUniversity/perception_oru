#ifndef DAS_CONVERSION_19072016
#define DAS_CONVERSION_19072016

#include "Eigen/Core"
#include "opencv2/opencv.hpp"
#include "ndt_map/ndt_map.h"
#include <math.h>

#include "utils.hpp"

namespace perception_oru{
	namespace ndt_feature_finder{
		
		inline cv::Point2d scalePoint(const cv::Point2d& p, int scale){
			cv::Point2d center;
			center.x = p.x * scale;
			center.y = p.y * scale;
			return center;
		}
		
		inline cv::Point2i scalePoint(const cv::Point2d& p, double max , double min, double size_image_max){
			cv::Point2i center;
			double oldRange = (max - min);
			double newRange = (size_image_max - 0);  
			center.x = (((p.x - min) * newRange) / oldRange) + 0;
			center.y = (((p.y - min) * newRange) / oldRange) + 0;
			
			if(center.x >= size_image_max || center.y >= size_image_max || center.x <= 0 || center.y <= 0){
				std::cout << "center " << center << " size_ image " << size_image_max << " min " << min << " max " << max << " for point " << p << std::endl;
			}
			
			//HACK
			if(center.x <0) center.x = 0;
			if(center.y <0) center.y = 0;
			if(center.x >=size_image_max) center.x = size_image_max-1;
			if(center.y >=size_image_max) center.y = size_image_max-1;
			
			assert(center.x <= size_image_max);
			assert(center.y <= size_image_max);
			assert(center.x >= 0);
			assert(center.y >= 0);
			
			return center;
		}
		
		
		inline void drawCell(const perception_oru::NDTCell& cell, cv::Mat& dst, int scale = 50){
			Eigen::Vector3d mean = cell.getMean();
			Eigen::Vector3d evals = cell.getEvals();
			Eigen::Matrix3d evecs = cell.getEvecs();
			
			//Assuming 2D
			evals = cell.getEvals();
			evecs = cell.getEvecs();
			EigenSort2D(evals, evecs);	
			Eigen::Vector3d eigen_vec = evecs.col(0);
			Eigen::Vector3d eigen_vec1 = evecs.col(1);
			//HACK : magic number
			double eigen_val = 3 * evals(0) * 1000;
			double eigen_val1 = 3 *  evals(1) * 1000;
			
			cv::Point2d center = scalePoint(cv::Point2d(mean(0), mean(1)), scale);
			//The 3 was found in the rviz visualization package
// 			center.x = mean(0) * scale;
// 			center.y = mean(1) * scale;
			
// 			std::cout << "CENTER " << center.x << " " << center.y << " for " <<mean(0) << " " << mean(1)<< std::endl;
			
			assert(center.x >= 0);
			assert(center.y >= 0);
			
			Eigen::Vector3d horizon;
			horizon << 1, 0, 0 ;
			
// 			std::cout << "Size : " << cv::Size(eigen_val, eigen_val1) << " " << evals(0) << " " << evals(1) << std::endl;
// 			std::cout << "Positin " << center << std::endl;
// 			std::cout << "Size of image: " << dst.rows << " " << dst.cols << std::endl;
			
			double rotation_degree = getAngle(horizon, eigen_vec);
			rotation_degree = rotation_degree * (180 / M_PI);
			cv::ellipse(dst, center, cv::Size(eigen_val, eigen_val1), rotation_degree, 0, 360, cv::Scalar(255, 255, 255), -1);
			
		}
		
		inline void drawCell(const perception_oru::NDTCell& cell, cv::Mat& dst, double max , double min, double size_image_max){
			Eigen::Vector3d mean = cell.getMean();
			Eigen::Vector3d evals = cell.getEvals();
			Eigen::Matrix3d evecs = cell.getEvecs();
			
			//Assuming 2D
			evals = cell.getEvals();
			evecs = cell.getEvecs();
			EigenSort2D(evals, evecs);	
			Eigen::Vector3d eigen_vec = evecs.col(0);
			Eigen::Vector3d eigen_vec1 = evecs.col(1);
			//HACK : magic number
			double eigen_val = 3 * evals(0) * 100;
			double eigen_val1 = 3 *  evals(1) * 100;
			
			
			
			cv::Point2d center = scalePoint(cv::Point2d(mean(0), mean(1)), max, min, size_image_max);
			//The 3 was found in the rviz visualization package
// 			center.x = mean(0) * scale;
// 			center.y = mean(1) * scale;
			
// 			std::cout << "CENTER " << center.x << " " << center.y << " for " <<mean(0) << " " << mean(1)<< " max min " << max << "  " << min << " size image max " << size_image_max << std::endl;
			
			assert(center.x >= 0);
			assert(center.y >= 0);
			
			Eigen::Vector3d horizon;
			horizon << 1, 0, 0 ;
			
// 			std::cout << "Size : " << cv::Size(eigen_val, eigen_val1) << " " << evals(0) << " " << evals(1) << std::endl;
// 			std::cout << "Positin " << center << std::endl;
// 			std::cout << "Size of image: " << dst.rows << " " << dst.cols << std::endl;
			
			double rotation_degree = getAngle(horizon, eigen_vec);
			rotation_degree = rotation_degree * (180 / M_PI);
			cv::ellipse(dst, center, cv::Size(eigen_val, eigen_val1), rotation_degree, 0, 360, cv::Scalar(255, 255, 255), -1);
			
		}
		
		
// 		inline void toCvMat(const lslgeneric::NDTMap& map, cv::Mat& dst, double scale = 50){
// 			double cx, cy, cz;
// 			//Should it be in meters ?
// 			map.getGridSizeInMeters(cx, cy, cz);
// 			std::cout << " Size grid " << cx << " " << cy << " " << cz << std::endl;
// 			cx = cx * scale;
// 			cy = cy * scale;
// 			cz = cz * scale;
// 			
// 			std::cout << "DRAWING " << cx << " " << cy << std::endl;
// 			
// 			//Initializing the Mat
// 			dst = cv::Mat::zeros(cv::Size(cx, cy), CV_32FC3);
// 			
// 			cv::imshow("tmp", dst);
// 			cv::waitKey(0);
// 			
// 			//Drawing all cells
// 			std::vector<lslgeneric::NDTCell*> cells = map.getAllCells();
// 			for(size_t i = 0 ; i < cells.size() ; ++i){
// // 				std::cout << "Drawing cell "<< i << std::endl;
// 				drawCell(*cells[i], dst, scale);
// 				
// // 				cv::imshow("tmp", dst);
// // 				cv::waitKey(0);
// 			}
// 		}
		
		
		
		/**
		 * To find ndt pointin openCV image, use 
		 * cv::Point2d center = scalePoint(cv::Point2d(mean(0), mean(1)), max, min, size_image_max);
		 * 
		 * @param[in] map NDTmap in
		 * @param[out] dst Cv::Mat out
		 * @param[in] size_image_max size of cv::math
		 * @param[out] max max value of mean in cells
		 * @param[out] min min value of mean in cells
		 */
		inline void toCvMat(const perception_oru::NDTMap& map, cv::Mat& dst, double size_image_max, double& max, double& min){
			
			std::vector<perception_oru::NDTCell*> cells = map.getAllCells();
			Eigen::Vector3d mean = (*cells[1]).getMean();
			max = mean(0), min = mean(1);
			for(size_t i = 0 ; i < cells.size() ; ++i){
				mean = (*cells[i]).getMean();
				if(max < mean(0)){
					max = mean(0);
				}
				if(max < mean(1)){
					max = mean(1);
				}
				if(min > mean(0)){
					min = mean(0);
				}
				if(min > mean(1)){
					min = mean(1);
				}
			}
				
			double oldRange = (max - min);
			double newRange = (size_image_max - 0);  
	
			double cx, cy, cz;
			//Should it be in meters ?
			map.getGridSizeInMeters(cx, cy, cz);
			std::cout << " Size grid " << cx << " " << cy << " " << cz << " and max min " << max << " " << min << std::endl;
			cx = (((cx - min) * newRange) / oldRange) + 0;
			cy = (((cy - min) * newRange) / oldRange) + 0;
			cz = (((cz - min) * newRange) / oldRange) + 0;
			
			std::cout << "DRAWING " << cx << " " << cy << std::endl;
			
			//Initializing the Mat
			dst = cv::Mat::zeros(cv::Size(size_image_max, size_image_max), CV_32FC3);
			
// 			cv::imshow("tmp", dst);
// 			cv::waitKey(0);
			
			//Drawing all cells
			
			
			
			for(size_t i = 0 ; i < cells.size() ; ++i){
// 				std::cout << "Drawing cell "<< i << std::endl;
				drawCell(*cells[i], dst, max , min, size_image_max);
				
// 				cv::imshow("tmp", dst);
// 				cv::waitKey(0);
			}
			
			std::cout << "Returning " << max << " " << min << std::endl;
		}
		
		
		/**
		 * Create an openCV image of an NDT
		 * 
		 * @param[in] map NDTmap in
		 * @param[out] dst Cv::Mat out
		 * @param[in] size_image_max size of cv::math
		 */
		inline void toCvMat(const perception_oru::NDTMap& map, cv::Mat& dst, double size_image_max){
			double max, min;
			toCvMat(map, dst, size_image_max, max, min);
		}
		
		
	}
	
}

#endif