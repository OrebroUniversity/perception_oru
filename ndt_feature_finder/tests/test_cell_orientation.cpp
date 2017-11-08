#include "ndt_feature_finder/ndt_corner.hpp"


double compareCell(const Eigen::Matrix3d& eigen){
	perception_oru::NDTCell cell;
	Eigen::Vector3d eval;
	eval(0) = 0.01 ;
	eval(1) = 0.01 ;
	eval(2) = 0.01 ;
	cell.setEvals(eval);
// 	cell.setEvecs(eigen);
	
	//angle should be 90deg or pi/2
	
// 	return cell.getAngleCovariance();
}

int main(){
	
	perception_oru::NDTMap map;
	
	std::string filename = "map.jff";
	map.loadFromJFF(filename.c_str());
	
	perception_oru::ndt_feature_finder::NDTCorner ndt_corners;

	
	//Identity
	Eigen::Matrix3d eigen;
	eigen(0,0) = 1;
	eigen(0,1) = 0;
	eigen(0,2) = 0;
	eigen(1,0) = 0;
	eigen(1,1) = 1; 
	eigen(1,2) = 0; 
	eigen(2,0) = 0;
	eigen(2,1) = 0; 
	eigen(2,2) = 1;
	
	double angle = compareCell(eigen);
	std::cout << "Angle should be 90 or pi/2 (1.57) : " << angle << std::endl;
	
	eigen(0,0) = -1;
	eigen(1,0) = 0;
	eigen(2,0) = 0;
	
	eigen(0,1) = 0;
	eigen(1,1) = -1;
	eigen(2,1) = 0; 
	
	eigen(0,2) = 0;
	eigen(1,2) = 0; 
	eigen(2,2) = -1;
	
	
	angle = compareCell(eigen);
	std::cout << "Angle should be 90 or pi/2 (1.57) : " << angle << std::endl;
	
	
	eigen(0,0) = 0;
	eigen(1,0) = 0;
	eigen(2,0) = 2;
	
	eigen(0,1) = 0;
	eigen(1,1) = -1;
	eigen(2,1) = 0; 
	
	eigen(0,2) = 0;
	eigen(1,2) = 0; 
	eigen(2,2) = -1;
	
	
	angle = compareCell(eigen);
	std::cout << "Angle should be 0 : " << angle << std::endl;
	
	eigen(0,0) = 0;
	eigen(1,0) = 0;
	eigen(2,0) = -2;
	
	eigen(0,1) = 0;
	eigen(1,1) = -1;
	eigen(2,1) = 0; 
	
	eigen(0,2) = 0;
	eigen(1,2) = 0; 
	eigen(2,2) = -1;
	
	
	angle = compareCell(eigen);
	std::cout << "Angle should be 180 or pi (3.14159) : " << angle << std::endl;
	
	eigen(0,0) = 0;
	eigen(1,0) = 36;
	eigen(2,0) = 0;
	
	eigen(0,1) = 0;
	eigen(1,1) = -1;
	eigen(2,1) = 0; 
	
	eigen(0,2) = 0;
	eigen(1,2) = 0; 
	eigen(2,2) = -1;
	
	
	angle = compareCell(eigen);
	std::cout << "Angle should be 90 or pi/2 (1.57) : " << angle << std::endl;
	
	
// 	lslgeneric::NDTCell cell = map.getCellAtPoint();
// 	auto cell_around = map.getInitializedCellsForPoint();
	
// 	getNeighbors in lazy grid reutnr all cell withn a radius pb, order ?
	
	
	
}