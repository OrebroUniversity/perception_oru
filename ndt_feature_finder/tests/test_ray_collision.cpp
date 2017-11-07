#include "ndt_feature_finder/utils.hpp"

#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_CASE(trying)
{
	
	Eigen::Vector3d ray_direction; 
	Eigen::Vector3d ray_point;
	Eigen::Vector3d ray_direction_second; 
	Eigen::Vector3d ray_point_second;
	
	ray_direction << 1, 0, 0;
	ray_direction_second << 0, 1, 0;
	
	std::cout << "Test 1 " << std::endl;
	ray_point << 0, 0, 0;
	ray_point_second << 0, 0, 0;
	Eigen::Vector3d out = perception_oru::ndt_feature_finder::collisionRay(ray_direction, ray_point, ray_direction_second, ray_point_second);
	Eigen::Vector3d test;
	test << 0,0,0;
	BOOST_CHECK_EQUAL(out, test);
// 	ray_point << 0, 0, 0;
	ray_point_second << -1, 0, 0;
	out = perception_oru::ndt_feature_finder::collisionRay(ray_direction, ray_point, ray_direction_second, ray_point_second);
	test << -1,0,0;
	std::cout << "Test 2 " << std::endl;
	BOOST_CHECK_EQUAL(out, test);
	
	
	std::cout << "Test 3 " << std::endl;
	ray_point_second << -1, 1, 0;
	out = perception_oru::ndt_feature_finder::collisionRay(ray_direction, ray_point, ray_direction_second, ray_point_second);
	BOOST_CHECK_EQUAL(out, test);
	
	std::cout << "Test 4 " << std::endl;
	ray_direction_second << 1, 1, 0;
	ray_point_second << -1, -1, 0;
	out = perception_oru::ndt_feature_finder::collisionRay(ray_direction, ray_point, ray_direction_second, ray_point_second);
	test << 0,0,0;
	BOOST_CHECK_EQUAL(out, test);
	
	ray_direction << 1, 1, 0;
	ray_point << -1, 1, 0;
	ray_direction_second << -1, 1, 0;
	ray_point_second << -2, -1, 0;
	out = perception_oru::ndt_feature_finder::collisionRay(ray_direction, ray_point, ray_direction_second, ray_point_second);
	test << -2, 1, 0;
	std::cout << "Test 5 " << std::endl;
	BOOST_CHECK_EQUAL(out, test);
	
	ray_direction << 1, 1, 0;
	ray_point << -1, 1, 0;
	ray_direction_second << 1, -1, 0;
	ray_point_second << -2, -1, 0;
	out = perception_oru::ndt_feature_finder::collisionRay(ray_direction, ray_point, ray_direction_second, ray_point_second);
	test << -2, 1, 0;
	
	std::cout << std::endl << ray_point - out << std::endl;
	std::cout << std::endl << ray_point_second - out << std::endl;
	std::cout << "Test 6 " << std::endl;
	BOOST_CHECK_EQUAL(out, test);
	
	
	auto mean_cell_tmp = ray_point;
	auto vector2 = ray_point - out;
	double angle_to = atan2(vector2(1), vector2(0)) - atan2(0, 1);
	std::cout << "angle to " << angle_to << std::endl;
	if (angle_to < 0) angle_to += 2 * M_PI;
// 	if(angle_to > M_PI) angle_to = (2 * M_PI) - angle_to;
	
	Eigen::Vector3d horizon;
	horizon << 1, 0, 0 ;
	double rotation_degree = perception_oru::ndt_feature_finder::getAngle(horizon,vector2);
	if(rotation_degree > M_PI){
		rotation_degree = (M_PI * 2) - rotation_degree;
	}
	double rotation_degree2 = perception_oru::ndt_feature_finder::getAngle(horizon,ray_direction);
	if(rotation_degree2 > M_PI){
		rotation_degree2 = (M_PI * 2) - rotation_degree2;
	}
	
// 	BOOST_CHECK_EQUAL(angle_to, rotation_degree);
// 	BOOST_CHECK_EQUAL(rotation_degree2, rotation_degree);
	
	double rotation_degree4 = perception_oru::ndt_feature_finder::getAngle(ray_direction, ray_direction_second);
	if(rotation_degree4 > M_PI){
		rotation_degree4 = (M_PI * 2) - rotation_degree4;
	}
	
	auto vector3 = ray_point_second - out;
	std::cout << "VEC2 !" << vector2<<std::endl;
	std::cout << "VEC3 !" << vector3<<std::endl;
	double angle_to2 = atan2(vector3(1), vector3(0)) - atan2(0, 1);
	if (angle_to2 < 0) angle_to2 += 2 * M_PI;
// 	if(angle_to2 > M_PI) angle_to2 = (2 * M_PI) - angle_to2;
	std::cout << "angle to " << angle_to2 << std::endl;
	
	double a_b = perception_oru::ndt_feature_finder::getAngle(vector2, vector3);
	if(a_b > M_PI){
		a_b = (M_PI * 2) - a_b;
	}
	double width = std::abs(angle_to - angle_to2);
	if(width > M_PI){
		width = (M_PI * 2) - width;
	}
	BOOST_CHECK_EQUAL(width, a_b);
	BOOST_CHECK_EQUAL(width, rotation_degree4);
	
	
	
	std::cout << ray_point << " - " << out << " = "  << vector2 << " \n\n"<< ray_direction << std::endl;
	int a; 
	std::cin >> a;
	
	
	
	
	
	
}