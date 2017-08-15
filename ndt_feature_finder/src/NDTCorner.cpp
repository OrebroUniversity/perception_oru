#include "ndt_feature_finder/NDTCorner.hpp"


std::vector< boost::shared_ptr< lslgeneric::NDTCell > > perception_oru::ndt_feature_finder::NDTCorner::getAllCorners(const lslgeneric::NDTMap& map)
{
	
	clear();
	//Go throught the mpa and test every cell
	map.getCellSizeInMeters(_x_cell_size, _y_cell_size, _z_cell_size);
	
// 	std::cout << "Getting all cells " << map.numberOfActiveCells() << std::endl;
	auto allCells = map.getAllCellsShared();
	
// 	std::cout << "vector" << std::endl;
	assert(allCells.size() == map.numberOfActiveCells());
// 	std::cout << "vector" << std::endl;
	std::vector< boost::shared_ptr< lslgeneric::NDTCell > > out;
// 	std::cout << "vector" << std::endl;
	for(size_t i = 0 ; i < allCells.size() ; ++i){
// 		std::cout << "Searching cell nb " << i << std::endl;
		Eigen::Vector3d corner_out;
// 	std::cout << "vector" << std::endl;
		if(cellIsCorner(map, *allCells[i], allCells, corner_out) == true){
// 			std::cout << "It's a corner :)" << std::endl;
			out.push_back(allCells[i]);
			_corners_position.push_back(corner_out);
			assert(_angles.size() == _corners_position.size());
		}
		else{
// 			std::cout << "Not a corner" << std::endl;
		}
	}
	
	assert(_angles.size() == _corners_position.size());
	_corners = out;
	
	clearAccurateCorner();
	//Translate the corner into openCV structure
	toOpenCV();
	
	return out;

}

bool perception_oru::ndt_feature_finder::NDTCorner::cellIsCorner(const lslgeneric::NDTMap& map, const lslgeneric::NDTCell& cell, const std::vector< boost::shared_ptr< lslgeneric::NDTCell > >& allCells, Eigen::Vector3d& corner)
{
// 	std::cout << "is it a corner?" << std::endl;
	double angle;
	int count_of_good_wall = 0;
	
	try{
		//Calculate vector
		//Get all cells with a gaussian
		//For every cell get all other cells around and test if at least two are pointing toward the center cell
		//Get neighbor of the cell
// 		std::cout << "Getting closest cells" <<std::endl;
		auto neighbor = getClosestCells(map, cell, _neighbor_size);
		
// 		std::cout << "Got the closest cells : " << neighbor.size() <<std::endl;
		assert(neighbor.size() <= allCells.size());
		
		if(neighbor.size() >= 8){
// 			std::cout << "CEnter " << cell.getMean() << std::endl;
			for(size_t j = 0 ; j < neighbor.size() ; ++j){
// 				std::cout << "Cells " << neighbor[j]->getMean() << std::endl;
// 				std::cout << "CEnter " << cell.getMean() << std::endl;
			
				for(size_t jj = j + 1 ; jj < neighbor.size() ; ++jj){
					
// 					std::cout << "Cells " << neighbor[jj]->getMean() << std::endl;
					if (neighbor[j]->getMean() == neighbor[jj]->getMean() && j != jj){
// 						std::cout << "Already SEEN " <<j << " and " << jj << std::endl;
						assert(neighbor[j]->getMean() != neighbor[jj]->getMean());
					}
				}
			}
		}
		
// 		std::cout << "Test same cell twice: " << std::endl;
// 		std::cout << std::endl << "ALL CELLS" << std::endl << std::endl;
		if(allCells.size() >= 8){
// 			std::cout << "CEnter " << cell.getMean() << std::endl;
			for(size_t j = 0 ; j < allCells.size() ; ++j){
// 				std::cout << "Cells " << allCells[j]->getMean() << std::endl;
				for(size_t jj = j + 1 ; jj < allCells.size() ; ++jj){
					if (allCells[j]->getMean() == allCells[jj]->getMean() && j != jj){
// 						std::cout << "Already SEEN " <<j << " and " << jj << std::endl;
						assert(allCells[j]->getMean() != allCells[jj]->getMean());
					}
				}
			}
		}
		
// 		exit(0);
		
		
// 		assert(neighbor.size() <= 8);
		
		//If there is more than one neighbor == Crossing
		if(neighbor.size() > 1){
// 			auto possible_neighbor = getCellsPointingToward(neighbor, cell);
			auto possible_neighbor = neighbor;
			if(possible_neighbor.size() > 0){
// 				std::cout << "nieghbor size " << possible_neighbor.size() << std::endl;
				if(gotAngledNDT(map, possible_neighbor, corner)){
					return true;
				}
			}
			else{
// 				std::cout << "Not neighbor facing the right direction" << std::endl;
			}
		}
		else{
// 			std::cout << "Neighbor is too small : " << neighbor.size() << std::endl;
		}
	}
	catch(const std::exception& ex){
		// speciffic handling for all exceptions extending std::exception, except
		// std::runtime_error which is handled explicitly
		std::cerr << "Error occurred: " << ex.what() << std::endl;
	}
	

	return false;
}


// double perception_oru::ndt_feature_finder::NDTCorner::getCellOrientation(const lslgeneric::NDTCell& cell) const
// {
// 	
// 	//Calculate the abgle the distribution
// 	
// 	assert(cell.hasGaussian_ == true);
// 	
// // 	cell.classify(); //Get the type first of all
// 	
// 	if(cell.getClass() != cell.ROUGH && cell.getClass() != cell.UNKNOWN){
// 		return cell.getAngleCovariance();
// 	}
// 	else if(cell.getClass() == cell.ROUGH){
// 		throw std::runtime_error("Cell is rough");
// 	}
// 	else{
// 		throw std::runtime_error("Cell is unknown");
// 	}
// }

std::vector< lslgeneric::NDTCell* > perception_oru::ndt_feature_finder::NDTCorner::getClosestCells(const lslgeneric::NDTMap& map, const lslgeneric::NDTCell& cell, int neig_size) const{
	double max = _x_cell_size;
	if(max < _y_cell_size){
		max = _y_cell_size;
	}
	if(max < _z_cell_size){
		max = _z_cell_size;
	}
	
	auto lazygrid = dynamic_cast<lslgeneric::LazyGrid*>(map.getMyIndex());
	pcl::PointXYZ point = cell.getCenter();
	double radius = max;
	std::vector<lslgeneric::NDTCell*> cells;
	lazygrid->getNeighbors(point, radius * neig_size, cells);
	
// 	std::cout << "OUT /*SIZE*/ " << cells.size() << " " << max <<std::endl;
	
	std::vector<lslgeneric::NDTCell*> cells_out;
	for(size_t j = 0 ; j < cells.size() ; ++j){
// 		std::cout << "Celly " << cells[j]->getMean() << std::endl;
		if(cells[j]->hasGaussian_ == true){
// 			std::cout << "PUSHED" << std::endl;
			cells_out.push_back(cells[j]);
		}
	}
// 	std::cout << "Center " << cell.getMean() << std::endl;
// 	std::cout << "Nei " << cells_out.size() << " " << max <<std::endl;
	
	return cells_out;
	
	
}


// std::vector< lslgeneric::NDTCell* > perception_oru::ndt_feature_finder::NDTCorner::getClosestCells(const std::vector< lslgeneric::NDTCell* >& allcells, const lslgeneric::NDTCell& cell) const
// {
// 	double max = _x_cell_size;
// 	if(max < _y_cell_size){
// 		max = _y_cell_size;
// 	}
// 	if(max < _z_cell_size){
// 		max = _z_cell_size;
// 	}
// 	
// 	double distance = -1;
// 	std::vector< lslgeneric::NDTCell* > out;
// 	//First find minimum
// 	for(size_t i = 0 ; i < allcells.size() ; ++i){
// // 		std::cout << "Checking out cell " << i << std::endl;
// 		//TODO : Make sure the distance is of one cell
// 		
// 		auto mean = allcells[i]->getMean();
// 		//Get the distance between two vector
// 		auto tmp_vec = mean - cell.getMean();
// 		double tmp = tmp_vec.norm();
// 		if(distance == -1){
// 			distance = tmp;
// 		}
// 		else if(distance == 0){
// 			//Same point is nothing to do
// 		}
// 		else{
// 			//If the new distance if better
// 			
// 			if(tmp < distance){
// 				distance = tmp;
// 			}
// 		}
// 		
// 	}
// 	
// 	if(distance <= max ){
// 		//Find other close enough
// 		for(size_t i = 0 ; i < allcells.size() ; ++i){
// 	// 		std::cout << "Checking out cell " << i << std::endl;
// 			//TODO : Make sure the distance is of one cell
// 			
// 			auto mean = allcells[i]->getMean();
// 			//Get the distance between two vector
// 			auto tmp_vec = mean - cell.getMean();
// 			double tmp = tmp_vec.norm();
// 			
// 			if(tmp < distance + max){
// 				out.push_back(allcells[i]);
// 			}
// 		}
// 	}
// 	
// 	return out;
// }


// double perception_oru::ndt_feature_finder::NDTCorner::getAngle(const Eigen::Vector3d& vec, const Eigen::Vector3d& vec2) const
// {
// 
// // 	std::cout << "Minor Axis " << std::endl << vec << std::endl;
// // 	std::cout << "Compared to  " << std::endl << vec2 << std::endl;
// 	//dot product with vertical axis gives us the angle
// 	double d = vec.dot(vec2);
// 	double l = vec.norm() * vec2.norm();
// 	
// // 	std::cout << " d : " << d << " norm " << l << std::endl;
// 	double ac = d/l;
// 	ac = std::acos(ac);
// 	return ac;
// 
// }

int perception_oru::ndt_feature_finder::NDTCorner::getBiggestEigenVector2D(const lslgeneric::NDTCell& cell, Eigen::Vector3d& eigenval, Eigen::Matrix3d& eigenvec) const
{
	
	eigenval = cell.getEvals();
	eigenvec = cell.getEvecs();
// 	std::cout << "Eigen Vec : "<< std::endl << eigenvec << std::endl;
// 	std::cout << "Eigen sort" << std::endl;
	EigenSort2D(eigenval, eigenvec);
// 	std::cout << "Eigen sorted" << std::endl;
		
	Eigen::Vector3d biggest_eigen = eigenvec.col(0);
	Eigen::Vector3d biggest_eigen1 = eigenvec.col(1);
	
	if(eigenval(1) > eigenval(0)){
		return  1;
	}
	return 0;
	
	
// 	double norm = biggest_eigen.norm();
// 	std::cout << "Comparing norms" << std::endl;
// 	if(biggest_eigen1.norm() > biggest_eigen.norm()){
// 		assert(eigenval(1) > eigenval(0));
// // 		norm = biggest_eigen1.norm();
// // 		std::cout << "Print EIGEN MAT " << std::endl << eigenvec << std::endl << " OUt we chose you " << std::endl << biggest_eigen1 << std::endl;
// 		return biggest_eigen1;
// 		
// 	}
// 	
// // 	std::cout << "Print EIGEN MAT " << std::endl << eigenvec << std::endl << " OUt we chose you " << std::endl << biggest_eigen << std::endl;
// 	assert(eigenval(0) > eigenval(1));
// 	return biggest_eigen;
}


std::vector< lslgeneric::NDTCell* > perception_oru::ndt_feature_finder::NDTCorner::getCellsPointingToward(std::vector< lslgeneric::NDTCell* >& neighbor, const lslgeneric::NDTCell& cell) const
{
//Check if the angle between the center and the neighbor is going toward the center == Angle of the vector mean|mean to reference is the same as neighbor variance to reference
	
	std::vector< lslgeneric::NDTCell* > possible_neighbor;
	auto mean_cell = cell.getMean();
	
	for(size_t j = 0 ; j < neighbor.size() ; ++j){
		
		//Get orientation of neighbor 
		Eigen::Vector3d eigenval; 
		Eigen::Matrix3d eigenvec;
		int index = getBiggestEigenVector2D(*neighbor[j], eigenval, eigenvec);
		Eigen::Vector3d orientation_neighbor = eigenvec.col(index);
		
		//Get Vector from neighbor to center
		auto mean_tmp = neighbor[j]->getMean();
		//Difference between both point with mean_tmp as start
		Eigen::Vector3d vec = mean_cell - mean_tmp;
		
		// angle
		double angle_tmp = getAngle(vec, orientation_neighbor);
		
		//Move angle into [0 ; PI]
		if(angle_tmp > M_PI){
			angle_tmp = (M_PI * 2) - angle_tmp;
		}
		
		// Corner if it is close to 0deg or 180deg
		if(angle_tmp < 0.1 || angle_tmp > M_PI - 0.1){
			possible_neighbor.push_back(neighbor[j]);
		}
	}
	
	return possible_neighbor;
}

bool perception_oru::ndt_feature_finder::NDTCorner::gotAngledNDT(const lslgeneric::NDTMap& map,std::vector< lslgeneric::NDTCell* >& neighbor, Eigen::Vector3d& corner) {
	
	for(size_t j = 0 ; j < neighbor.size() ; ++j){
		//Get orientation of neighbor 
// 		std::cout << " j " << j <<std::endl;
		Eigen::Vector3d eigenval; 
		Eigen::Matrix3d eigenvec;
		int index = getBiggestEigenVector2D(*neighbor[j], eigenval, eigenvec);
		
// 		std::cout << "Eigen Vec : "<<eigenvec << std::endl;
// 		std::cout << "Eigen VAL : "<<eigenval << std::endl;
// 		std::cout << "Index " << index << std::endl;
		Eigen::Vector3d orientation = eigenvec.col(index);
		auto mean_cell = neighbor[j]->getMean();
		
		for(size_t i = j + 1 ; i < neighbor.size() ; ++i){
			
			assert(neighbor[j] != neighbor[i]);
			
// 			std::cout << " i " << i <<std::endl;
			//Get Vector from neighbor to center
			//Difference between both point with mean_tmp as start
			Eigen::Vector3d eigenval_tmp; 
			Eigen::Matrix3d eigenvec_tmp;
			int index_tmp = getBiggestEigenVector2D(*neighbor[i], eigenval_tmp, eigenvec_tmp);
			
// 			std::cout << "Index tmp" << index_tmp << std::endl;
			Eigen::Vector3d orientation_tmp = eigenvec_tmp.col(index_tmp);
			
			int index_other = 0;
			if(index_other == index_tmp){
				index_other = 1;
			}
			
			//CHeck the eigen value are different enough
// 			if(eigenval_tmp(index_tmp) >= eigenval_tmp(index_other) * 5){
				
				
				// angle
				double angle_tmp_test = getAngle(orientation_tmp, orientation);
				if(angle_tmp_test > M_PI){
					angle_tmp_test = (M_PI * 2) - angle_tmp_test;
				}
				//NEW
				corner = collisionRay(orientation_tmp, neighbor[i]->getMean(), orientation, neighbor[j]->getMean());
					
				auto mean_cell_tmp = neighbor[i]->getMean();
				auto vector1 = mean_cell - corner;
				auto vector2 = mean_cell_tmp - corner;
				double angle_from = atan2(vector1(1), vector1(0)) - atan2(0, 1);
				if (angle_from < 0) angle_from += 2 * M_PI;
				double angle_to = atan2(vector2(1), vector2(0)) - atan2(0, 1);
				if (angle_to < 0) angle_to += 2 * M_PI;
				double direction = (angle_to + angle_from) / 2;
				
				double angle_between = atan2(vector1(1), vector1(0)) - atan2(vector2(1), vector2(0));
				if (angle_between < 0) angle_between += 2 * M_PI;
				
				
// 				std::cout << vector1 << "\n\n "<< orientation << "\n\n " << vector2 << " \n\n"<< orientation_tmp << std::endl;
// 				std::cout << "means " << neighbor[i]->getMean() << " " << mean_cell_tmp << " and " <<neighbor[j]->getMean() << " " << mean_cell << " corner " << corner << std::endl << std::endl;;
// 				int a; 
// 				std::cin >> a;
				
// 				double width = std::abs(angle_to - angle_from);
				
				if(angle_between > M_PI){
// 					std::cout << (2*M_PI) - width << " == " << angle_tmp << std::endl;
// 					assert((2*M_PI) - width <= angle_tmp + 0.1);
// 					assert((2*M_PI) - width >= angle_tmp - 0.1);
					angle_between = (M_PI * 2) - angle_between;
					direction - M_PI;
					if(direction < 0){
						direction = direction + (2 * M_PI);
					}
				}
				
				
				double angle_tmp = angle_between;
// 				std::cout << angle_tmp << " == " << angle_tmp_test << std::endl;
// 				assert(angle_tmp == angle_tmp_test);
				
				//Move angle into [0 ; PI]
				if(angle_tmp > M_PI){
					angle_tmp = (M_PI * 2) - angle_tmp;
				}
// 				std::cout << "nsize " << neighbor.size() << " Angle of neighbor " << angle_tmp << " deg angle " << angle_tmp * 180 / M_PI << std::endl;
	// 			std::cout << "between " << std::endl << orientation << std::endl << " and " << std::endl << orientation_tmp << std::endl;
				// Corner if it is close to 90deg
				if( (angle_tmp < ( ( M_PI / 2 ) + 0.1 ) ) && ( angle_tmp > ( ( M_PI / 2 ) - 0.1 ) ) ){
// 				if( (angle_tmp_test < ( ( M_PI / 2 ) + 0.1 ) ) && ( angle_tmp_test > ( ( M_PI / 2 ) - 0.1 ) ) ){
// 				if( (angle_tmp < 1.745 ) && ( angle_tmp > 1.222 ) ){
	// 				std::cout << "GOOD enter a: ";
	// 				int a;
	// 				std::cin >> a;
					
					//TODO calculate collision point !
// 					corner = collisionRay(orientation_tmp, neighbor[i]->getMean(), orientation, neighbor[j]->getMean());
// 					
// 					auto mean_cell_tmp = neighbor[i]->getMean();
// // 					auto vector1 = mean_cell - corner;
// 					auto vector1 = orientation;
// // 					auto vector2 = mean_cell_tmp - corner;
// 					auto vector2 = orientation_tmp;
// 					
// 					double angle_from = atan2(vector1(1), vector1(0)) - atan2(0, 1);
// 					if (angle_from < 0) angle_from += 2 * M_PI;
// 					double angle_to = atan2(vector2(1), vector2(0)) - atan2(0, 1);
// 					if (angle_to < 0) angle_to += 2 * M_PI;
// 					double direction = (angle_to + angle_from) / 2;
// 					double angle_between = atan2(vector1(1), vector1(0)) - atan2(vector2(1), vector2(0));
// 					if (angle_between < 0) angle_between += 2 * M_PI;
// 					
// 					double width = std::abs(angle_to - angle_from);
// 					
// 					std::cout <<"angles " << angle_tmp << " " << width << " " << angle_from << " " << angle_to << " between " << angle_between << " width " << width << std::endl;
// 					
// 					//Always point toward the smallest angle
// 					if(width > M_PI){
// 						std::cout << (2*M_PI) - width << " == " << angle_tmp << std::endl;
// 						assert((2*M_PI) - width <= angle_tmp + 0.1);
// 						assert((2*M_PI) - width >= angle_tmp - 0.1);
// 						direction + M_PI;
// 						while(direction >= 2 * M_PI){
// 							direction = direction - (2 * M_PI);
// 						}
// 					}
					
					
					//TODO Parameter to check that a certain number of neighbor of the crossing ndt or aligned.
					
					auto neig1 = getClosestCells(map, *neighbor[j], 2);
					auto neig2 = getClosestCells(map, *neighbor[i], 2);
					bool al = AlignedNDT(neig1, *neighbor[j]);
					bool al2 = AlignedNDT(neig2, *neighbor[i]);
// 					if(al == true && al2 == true){
					
						std::pair<double, double> pair(angle_tmp, direction);
						_angles.push_back(pair);

// 						std::cout << "COLLSION 
						return true;
// 					}
					
				}
// 			}
		}
	}
	return false;
}


void perception_oru::ndt_feature_finder::NDTCorner::clearAccurateCorner()
{
	std::vector < Eigen::Vector3d > tmp;
	std::vector<std::pair<double,double> > angles_tmp;
	for(size_t i = 0 ; i < _corners_position.size() ; ++i){
		bool seen = false;
		
		for(size_t j = 0 ; j < tmp.size() ; ++j){
// 			if(tmp[j] == _corners_position[i]){
			Eigen::Vector3d vec = tmp[j] - _corners_position[i];
			if( vec.norm() < _x_cell_size){
				seen = true;
			}
		}
		if(seen == false){	
			tmp.push_back(_corners_position[i]);
			angles_tmp.push_back(_angles[i]);
		}		
	}
	
	_corners_position.clear();
	_angles.clear();
	_corners_position = tmp;
	_angles = angles_tmp;

}

void perception_oru::ndt_feature_finder::NDTCorner::toOpenCV()
{
	_opencv_corners.clear();
	_opencv_corners_position.clear();
	
	//Get openCV point _corners
	auto it = _corners.begin();
	for(it; it != _corners.end() ; ++it){
		cv::Point2d p;
		p.x = (*it)->getMean()(0);
		p.y = (*it)->getMean()(1);
		_opencv_corners.push_back(p);	
	}
	auto it_pos = _corners_position.begin();
	for(it_pos; it_pos != _corners_position.end() ; ++it_pos){
		cv::Point2d p;
		p.x = (*it_pos)(0);
		p.y = (*it_pos)(1);
		_opencv_corners_position.push_back(p);	
	}
// 	std::cout << "All done" << std::endl;
}


void perception_oru::ndt_feature_finder::NDTCorner::calculateAngles(const lslgeneric::NDTMap& map)
{

}

/**
 * return true if at least one ndt in the neighbor is aligned with the given cell
 */
bool perception_oru::ndt_feature_finder::NDTCorner::AlignedNDT(const std::vector <lslgeneric::NDTCell* >& neighbor, const lslgeneric::NDTCell& cell)
{
	Eigen::Vector3d eigenval; 
	Eigen::Matrix3d eigenvec;
	int index = getBiggestEigenVector2D(cell, eigenval, eigenvec);
	Eigen::Vector3d orientation = eigenvec.col(index);
	
	for(size_t j = 0 ; j < neighbor.size() ; ++j){
		if(neighbor[j] != &cell){
			Eigen::Vector3d eigenval2; 
			Eigen::Matrix3d eigenvec2;
			int index2 = getBiggestEigenVector2D(*neighbor[j], eigenval2, eigenvec2);
			Eigen::Vector3d orientation2 = eigenvec2.col(index2);
			double angle_between = atan2(orientation(1), orientation(0)) - atan2(orientation2(1), orientation2(0));
			if (angle_between < 0) angle_between += 2 * M_PI;
			
			//Around 0
			if(angle_between <= 0.349 || angle_between >= 5.934){
				//Around 180
				if(angle_between >= 2.793 && angle_between <=3.491){
					return true;
				}
			}
			
		}
	}
	return false;

}

