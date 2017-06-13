#include <ndt_calibration/ndt_calib.h>
#include <ndt_calibration/ndt_calib_rviz.h>

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

#include <sensor_msgs/PointCloud2.h>

#include <boost/program_options.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace po = boost::program_options;
using namespace std;




int main(int argc, char **argv){
    std::string gt_file;
    std::string est_sensorpose_file;
    std::string base_name_pcd;
    std::string bag_file;
    std::string pose_frame;
    std::string world_frame;
    Eigen::Vector3d transl;
    Eigen::Vector3d euler;
    int score_type, objective_type;
    double max_translation;
    double min_rotation;
    double sensor_time_offset;
    double resolution;
    int index_offset;
    po::options_description desc("Allowed options");
    desc.add_options()
	("help", "produce help message")
	("visualize", "visualize the output")
	("gt_file", po::value<std::string>(&gt_file), "vehicle pose files in world frame")
	("est_sensorpose_file", po::value<std::string>(&est_sensorpose_file)->default_value(std::string("")), "estimated sensor poses in world frame")
	("bag_file", po::value<std::string>(&bag_file), "bag file containing the poses to use for performing sensor time offset calibration")
	("base_name_pcd", po::value<string>(&base_name_pcd)->default_value(std::string("")), "prefix for the .pcd files")
      ("pose_frame", po::value<string>(&pose_frame)->default_value("/state_base_link"), "frame containing the vehicle pose") // EKF
        ("world_frame", po::value<string>(&world_frame)->default_value("/world"), "default world frame used")  // World
	("x", po::value<double>(&transl[0])->default_value(0.), "translation vector x")
	("y", po::value<double>(&transl[1])->default_value(0.), "translation vector y")
	("z", po::value<double>(&transl[2])->default_value(0.), "translation vector z")
	("ex", po::value<double>(&euler[0])->default_value(0.), "euler angle vector x")
	("ey", po::value<double>(&euler[1])->default_value(0.), "euler angle vector y")
	("ez", po::value<double>(&euler[2])->default_value(0.), "euler angle vector z")
	("no_calib", "no calibration is performed")
	("score_type", po::value<int>(&score_type)->default_value(3), "score type used in the optimization, 0 - ICP, 1 - sensorpose difference, 2 - relative sensorpose difference, 3 - NDT")
	("objective_type", po::value<int>(&objective_type)->default_value(0), "objective type used in the optimization, 0 - full 6D, 1 - only time offset")

	("max_translation", po::value<double>(&max_translation)->default_value(3.), "max allowed translation between two scan pair")
	("min_rotation", po::value<double>(&min_rotation)->default_value(0.2), "min required rotation between two scan pair")
	("sensor_time_offset", po::value<double>(&sensor_time_offset)->default_value(0.), "initial time offset for the sensor")
        ("cx", "calibrate x axis")
        ("cy", "calibrate y axis")
        ("cz", "calibrate z axis")
        ("cex", "calibrate roll")
        ("cey", "calibrate pitch")
        ("cez", "calibrate yaw")
        ("ct", "calibrate time offset")
	("resolution", po::value<double>(&resolution)->default_value(2.), "NDT map resolution")
      ("index_offset", po::value<int>(&index_offset)->default_value(0), "if there is an index offset bettwen the cloudXXX.pcd and the rows in the pose files")
        ;


    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (!vm.count("gt_file"))
    {
	cout << "Missing gt_file.\n";
	cout << desc << "\n";
	return 1;
    }
    if (vm.count("help"))
    {
	cout << desc << "\n";
	return 1;
    }
    bool calib = !vm.count("no_calib");
    bool cx = vm.count("cx");
    bool cy = vm.count("cy");
    bool cz = vm.count("cz");
    bool cex = vm.count("cex");
    bool cey = vm.count("cey");
    bool cez = vm.count("cez");
    bool ct = vm.count("ct");
    Eigen::Affine3d Ts;
    //Ts.setIdentity();
    Ts = getAsAffine3d(transl, euler);
    
    
    NDTCalibScanPairs pairs;
    loadNDTCalibScanPairs(gt_file, est_sensorpose_file, base_name_pcd, pairs, max_translation, min_rotation, index_offset);
    cout << "Ts : " << affine3dToString(Ts) << std::endl;

    if (pairs.empty()) {
	std::cout << "failed in  loading / no scan pairs detected..." << std::endl;
    }
    std::cout << "got : " << pairs.size() << " scan pairs" << std::endl;

    std::cout << "Computing NDTMap... " << std::endl;
    pairs.computeNDTMap(resolution);
    std::cout << "done." << std::endl;

    std::cout << "Setting up pose interpolation" << std::endl;
    PoseInterpolationNavMsgsOdo pose_interp(bag_file, std::string("/tf"), world_frame, ros::Duration(3600));
    pose_interp.readBagFile();
    std::cout << "done." << std::endl;


    NDTCalibOptimize::ObjectiveType objective(cx, cy, cz, cex, cey, cez, ct);

    NDTCalibOptimize opt(pairs, score_type, objective, pose_interp, pose_frame);
    std::cout << "Optimize time : " << objective.optimizeTime() << std::endl;
    
    opt.interpPairPoses(sensor_time_offset);

    std::cout << " initial sensor pose est : " << affine3dToString(Ts) << " dt : " << sensor_time_offset << " error : " << opt.getScore6d(Ts) << std::endl;
    if (calib) {
	std::cout << "starting calibration, using score type : " << score_type << std::endl;
	opt.calibrate(Ts, sensor_time_offset);
        opt.interpPairPoses(sensor_time_offset);
	std::cout << "done. " << std::endl;
	std::cout << " new sensor pose est : " << affine3dToString(Ts) << " dt : " << sensor_time_offset << " error : " << opt.getScore6d(Ts) << std::endl;
    }

    // ROS based visualization below...
    
    std::cout << "Starting visualization..." << std::endl;

    ros::init(argc, argv, "ndt_calib");
    srand(time(NULL));


    ros::NodeHandle nh;
  
    visualization_msgs::MarkerArray m_array = ndt_visualisation::getMarkerArrayFromNDTCalibScanPairs(pairs);
    visualization_msgs::MarkerArray m_array2 = ndt_visualisation::getMarkerArrayRelFromNDTCalibScanPair(pairs[0], Ts);

    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    
    // Get a global point cloud for all poses with calibration and data in sensor coords.
    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    pairs.getGlobalPointCloud(Ts, global_cloud);
    global_cloud.header.frame_id = "/world";

    ros::Publisher pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("global_points", 1);

    ros::Rate r(1);

    int i = 0; 
    while (ros::ok()) {
	pointcloud_pub.publish(global_cloud);

	// Publish the marker...
	marker_array_pub.publish(m_array);
	marker_array_pub.publish(m_array2);

	ros::spinOnce();
	r.sleep();

	m_array2 = ndt_visualisation::getMarkerArrayRelFromNDTCalibScanPair(pairs[i++], Ts);
          
    }
    
    
    
}
