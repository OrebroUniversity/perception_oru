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
	
	Eigen::Vector3d v1;
	v1 << 0, 0, 0;
	Eigen::Vector3d v2;
	v2 << 1, 1, 1;
	Eigen::Vector3d v3;
	v3 << 2, 2, 2;
	Eigen::Matrix3d _eigen_vector;
	_eigen_vector << v2, v3, v1 ;
	
	std::cout << "EIGEN " << _eigen_vector << std::endl;
	
	perception_oru::NDTCell* p_cell = new perception_oru::NDTCell;
	boost::shared_ptr<perception_oru::NDTCell> cell(p_cell);
	
	Eigen::Vector3d eval;
	eval << 2, 1, 1;
	
	std::cout << "Set evals" << std::endl;
	cell->setEvals(eval);
	std::cout << "DONE" << std::endl;
	
	Eigen::Matrix3d evels;
	Eigen::Vector3d evec1;
	Eigen::Vector3d evec2;
	Eigen::Vector3d evec3;
	evec1 << 1, 0, 0;
	evec2 << 0, 1, 0;
	evec3 << 0, 0, 1;
	evels << evec1, evec2, evec3;	
	
	std::cout << "Set evecs" << std::endl;
	cell->setEvecs(evels);
	
	Eigen::Vector3d mean;
	mean << 0, 0, 0;
	
	cell->setMean(mean);
	
	
	
	perception_oru::NDTCell* p_cell2 = new perception_oru::NDTCell;
	boost::shared_ptr<perception_oru::NDTCell> cell2(p_cell2);
	
	Eigen::Vector3d eval2;
	eval2 << 1, 2, 1;
	
	std::cout << "Set evals" << std::endl;
	cell2->setEvals(eval2);
	std::cout << "DONE" << std::endl;
	
	Eigen::Matrix3d evels2;
	Eigen::Vector3d evec12;
	Eigen::Vector3d evec22;
	Eigen::Vector3d evec32;
	evec12 << 1, 0, 0;
	evec22 << 0, -1, 0;
	evec32 << 0, 0, 1;
	evels2 << evec12, evec22, evec32;	
	
	std::cout << "Set evecs" << std::endl;
	cell2->setEvecs(evels2);
	
	Eigen::Vector3d mean2;
	mean2 << 1, 1, 0;
	
	cell2->setMean(mean2);
	
	mean << 1, 0, 0;
	perception_oru::ndt_feature_finder::NDTCornerBundle ncb(mean);
	
	ncb.push_back_cell1(cell);
	ncb.push_back_cell2(cell2);
	std::cout << "Gaussian" << std::endl;
	ncb.gaussian();
	
	auto eigen_vec = ncb.getEigenVectors();
	auto eigen_val = ncb.getEigenValues();
	
	std::cout << "Eigen " << eigen_vec << std::endl;
	Eigen::Matrix3d res;
	res << 1, -1, 0,
		   1, 1, 0,
		   0, 0, 0;
		   
	assert(res == eigen_vec);
	
	
	///TEST 2
	
	evec12 << 1, 1, 0;
	evec22 << 1, -1, 0;
	evec32 << 0, 0, 1;
	evels2 << evec12, evec22, evec32;	
	std::cout << "WELL " << evels2 << std::endl;
	
	mean2 << 3, 5, 0;
	
	cell2->setEvecs(evels2);
	cell2->setMean(mean2);
	
	mean << 8, 0 , 0 ;
	perception_oru::ndt_feature_finder::NDTCornerBundle ncb2(mean);
	
	ncb2.push_back_cell1(cell);
	ncb2.push_back_cell2(cell2);
	std::cout << "Gaussian" << std::endl;
	ncb2.gaussian();
	
	auto eigen_vec2 = ncb2.getEigenVectors();
	auto eigen_val2 = ncb2.getEigenValues();
	
	std::cout << "Eigen \n" << eigen_vec2 << std::endl;
	Eigen::Matrix3d res2;
	res2 << 1, -3, 0,
		   1, 1, 0,
		   0, 0, 0;
	std::cout << "Eigen should be same as before \n" << res2 << std::endl;
		   
// 	assert(res2 == eigen_vec2);
	
	
	
}
