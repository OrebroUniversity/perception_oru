#include "ndt_feature_finder/ndt_corner.hpp"

double getAngle(const Eigen::Vector3d& orientation_tmp, const Eigen::Vector3d& orientation){
	perception_oru::ndt_feature_finder::NDTCorner coirners;
	double angle_tmp = getAngle(orientation_tmp, orientation);
	
	//Move angle into [0 ; PI]
	if(angle_tmp > M_PI){
		angle_tmp = (M_PI * 2) - angle_tmp;
	}
	return angle_tmp;
}

int main(){

	
	Eigen::Vector3d orientation_tmp;
	orientation_tmp(0) = 0;
	orientation_tmp(1) = 1;
	orientation_tmp(2) = 0;
	Eigen::Vector3d orientation;
	orientation(0) = 0;
	orientation(1) = 1;
	orientation(2) = 0;
	// angle
	
	double angle_tmp = getAngle(orientation_tmp, orientation);
	
	std::cout << " Angle of neighbor " << angle_tmp << " deg angle " << angle_tmp * 180 / M_PI << std::endl;
	std::cout << "between " << std::endl << orientation << std::endl << " and " << std::endl << orientation_tmp << std::endl;
	// Corner if it is close to 90deg
	if( (angle_tmp < ( ( M_PI / 2 ) + 0.1 ) ) && ( angle_tmp > ( ( M_PI / 2 ) - 0.1 ) ) ){
		std::cout << "BUG " << __LINE__ << std::endl;
	}
	else{
		std::cout << "good" << std::endl;
	}
	
	orientation(0) = 1;
	orientation(1) = 0;
	orientation(2) = 0;
	
	angle_tmp = getAngle(orientation_tmp, orientation);
	
	std::cout << " Angle of neighbor " << angle_tmp << " deg angle " << angle_tmp * 180 / M_PI << std::endl;
	std::cout << "between " << std::endl << orientation << std::endl << " and " << std::endl << orientation_tmp << std::endl;
	// Corner if it is close to 90deg
	if( (angle_tmp < ( ( M_PI / 2 ) + 0.1 ) ) && ( angle_tmp > ( ( M_PI / 2 ) - 0.1 ) ) ){
		std::cout << "good" << std::endl;
	}
	else{
		std::cout << "BUG " << __LINE__ << std::endl;
		
	}
	
	orientation(0) = 1;
	orientation(1) = 0;
	orientation(2) = 10;
	
	angle_tmp = getAngle(orientation_tmp, orientation);
	
	std::cout << " Angle of neighbor " << angle_tmp << " deg angle " << angle_tmp * 180 / M_PI << std::endl;
	std::cout << "between " << std::endl << orientation << std::endl << " and " << std::endl << orientation_tmp << std::endl;
	// Corner if it is close to 90deg
	if( (angle_tmp < ( ( M_PI / 2 ) + 0.1 ) ) && ( angle_tmp > ( ( M_PI / 2 ) - 0.1 ) ) ){
		std::cout << "good" << std::endl;
	}
	else{
		std::cout << "BUG " << __LINE__ << std::endl;
		
	}
	
	orientation(0) = 1;
	orientation(1) = 0.95;
	orientation(2) = 0.1;
	
	angle_tmp = getAngle(orientation_tmp, orientation);
	
	std::cout << " Angle of neighbor " << angle_tmp << " deg angle " << angle_tmp * 180 / M_PI << std::endl;
	std::cout << "between " << std::endl << orientation << std::endl << " and " << std::endl << orientation_tmp << std::endl;
	// Corner if it is close to 90deg
	if( (angle_tmp < ( ( M_PI / 2 ) + 0.1 ) ) && ( angle_tmp > ( ( M_PI / 2 ) - 0.1 ) ) ){
		
		std::cout << "BUG " << __LINE__ << std::endl;
	}
	else{
		std::cout << "good" << std::endl;
	}
	
	orientation(0) = 0;
	orientation(1) = -1;
	orientation(2) = 0.1;
	
	angle_tmp = getAngle(orientation_tmp, orientation);
	
	std::cout << " Angle of neighbor " << angle_tmp << " deg angle " << angle_tmp * 180 / M_PI << std::endl;
	std::cout << "between " << std::endl << orientation << std::endl << " and " << std::endl << orientation_tmp << std::endl;
	// Corner if it is close to 90deg
	if( (angle_tmp < ( ( M_PI / 2 ) + 0.1 ) ) && ( angle_tmp > ( ( M_PI / 2 ) - 0.1 ) ) ){
		
		std::cout << "BUG " << __LINE__ << std::endl;
	}
	else{
		std::cout << "good" << std::endl;
	}
	
	
}