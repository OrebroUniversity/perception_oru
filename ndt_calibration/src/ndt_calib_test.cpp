#include <ndt_calibration/ndt_calib.h>

#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/pointcloud_utils.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <algorithm>

#include <ros/ros.h>


#include <boost/program_options.hpp>


namespace po = boost::program_options;
using namespace std;


int main(int argc, char **argv){
      
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ;


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help"))
    {
	cout << desc << "\n";
	return 1;
    }

    {
      Eigen::Affine3d T1 = getAsAffine3d(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0));
      NDTCalibScan scan(T1, T1, 0.);
      std::cout << "Should give zeros";
      std::cout << "scan.getTs() : " << affine3dToString(scan.getTs()) << std::endl;
    }

    {
      Eigen::Affine3d T1 = getAsAffine3d(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0));
      Eigen::Affine3d T2 = getAsAffine3d(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(0, 0, M_PI/2.));
      NDTCalibScan scan(T1, T2, 0.);
      std::cout << "Should give 1 0 0 0 0 M_PI/2..." << std::endl;
      std::cout << "scan.getTs() : " << affine3dToString(scan.getTs()) << std::endl;
    }
    
    {
      Eigen::Affine3d T1 = getAsAffine3d(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0));
      Eigen::Affine3d T2 = getAsAffine3d(Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(0, 0, M_PI/2.));
      Eigen::Affine3d Ts = getAsAffine3d(Eigen::Vector3d(3.,0, 1.2), Eigen::Vector3d(0,0,0));
      
      NDTCalibScan scan1(T1, T1*Ts, 0.);
      NDTCalibScan scan2(T2, T2*Ts, 0.);
      NDTCalibScanPair pair;
      pair.first = scan1;
      pair.second = scan2;
      
      std::cout << "Should give 1 0 0 0 0 M_PI/2..." << std::endl;
      std::cout << "pair.getRelativePose() : " << affine3dToString(pair.getRelativePose()) << std::endl;
      std::cout << "Should give -2 3 0 0 0 M_PI/2..." << std::endl;
      std::cout << "pair.getRelativeEstSensorPose() : " << affine3dToString(pair.getRelativeEstSensorPose()) << std::endl;
      
      std::cout << "Should give 1 0 0 0 0 M_PI/2..." << std::endl;
      Eigen::Affine3d Ts2 = getAsAffine3d(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
      std::cout << "pair.getPredictedRelativeEstSensorPose(Ts2) : " << affine3dToString(pair.getPredictedRelativeEstSensorPose(Ts2)) << std::endl;

      std::cout << "Should give -2 3 0 0 0 M_PI/2..." << std::endl;
      std::cout << "pair.getPredictedRelativeEstSensorPose(Ts) : " << affine3dToString(pair.getPredictedRelativeEstSensorPose(Ts)) << std::endl;

      std::cout << "Shold give 0 0 0 0 0 0 0" << ::endl;
      std::cout << "pair.getDifference(Ts) : " << affine3dToString(pair.getDifference(Ts)) << std::endl;
    }

}
