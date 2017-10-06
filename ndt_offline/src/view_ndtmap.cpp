#include <ndt_map/ndt_map.h>
#include <ndt_rviz/ndt_rviz.h>
#include <tf_conversions/tf_eigen.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <algorithm>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
using namespace std;

int main(int argc, char **argv){
    std::string filename;
    double resolution;
    
    po::options_description desc("Allowed options");
    desc.add_options()
	("help", "produce help message")
	("filename", po::value<string>(&filename), "map file to be visualized")
        ("resolution", po::value<double>(&resolution)->default_value(1.), "map resolution")
        ;
    

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (!vm.count("filename"))
    {
	cout << "Missing filename.\n";
	cout << desc << "\n";
	return 1;
    }
    if (vm.count("help"))
    {
	cout << desc << "\n";
	return 1;
    }

    perception_oru::NDTMap map(new perception_oru::LazyGrid(resolution));
    map.loadFromJFF(filename.c_str());
    ROS_INFO_STREAM("Loaded map: " << filename << " containing " << map.getAllCells().size() << " cells");

    ros::init(argc, argv, "view_ndtmap");

    ros::NodeHandle nh;

    visualization_msgs::Marker m = ndt_visualisation::markerNDTCells(map, 1, "nd_map");
    
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 3);
    ros::Rate r(1);
    
    while (ros::ok()) {

        marker_pub.publish(m);
        ros::spinOnce();
        r.sleep();
    }
    ROS_INFO_STREAM("Quitting...");
}
