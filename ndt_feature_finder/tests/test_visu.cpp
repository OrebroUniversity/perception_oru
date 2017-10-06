#include <ros/ros.h>

#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/ndt_conversions.h>

#include "ndt_feature/NDTCornerVisu.hpp"

int main(int argc, char **argv)
{
	//*****************************First load the map and calculate points
	std::string file = "/home/malcolm/Documents/mapping0.jff";
// 	map.loadFromJFF(file.c_str());
	double resolution = 0.2;
	auto mapGrid = new perception_oru::LazyGrid(resolution);
	perception_oru::NDTMap map(mapGrid);
	if(map.loadFromJFF(file.c_str()) < 0)
		std::cout << "File didn't load" << std::endl;
	std::cout << "File loaded" << std::endl;
	
	auto cells = map.getAllInitializedCells();
	
	double x, y, z;
	map.getCellSizeInMeters(x, y, z);
	
	std::cout << "Size : " << x <<" " << y<< " " << z << std::endl;
	
// 	exit(0);
	
	for(size_t i = 0 ; i < cells.size() ; ++i){
// 		std::cout << "Classifying" << std::endl;
		cells[i]->classify();
		perception_oru::NDTCell cell;
		assert(cells[i]->getClass() != cells[i]->UNKNOWN);
	}
	
/************ EXPORT NDT CORNER TO FILE ******************/

// 	AASS::das::NDTCorner corners_export;
// 	std::cout << "Searching for corners" << std::endl;
// 	auto ret_export = corners_export.getAllCorners(map);
// 	auto ret_opencv_export = corners_export.getCvCorners();
// 	std::cout << "Found " << ret_export.size() << " Corners. yes same as " << ret_opencv_export.size() << std::endl;
// 
// 	std::ofstream out_stream("/home/malcolm/Documents/corners_ndt_accurate_mapping0.txt");
// 	corners_export.exportAccurateCorners(out_stream);
// 	
// 	exit(0);
	
	
	/**************************/
	
	AASS::das::NDTCorner corners;
	
	std::ifstream in_stream("/home/malcolm/Documents/corners_ndt_accurate.txt");
	corners.readAccurateCorners(in_stream);
	
	std::cout << "Searching for corners" << std::endl;
// 	auto ret = corners.getAllCorners(map);
	auto ret = corners.getAccurateCvCorners();
	std::cout << "Found " << ret.size() << " Corners " << std::endl;
	
// 	exit(0);
	//********************' NOW THE ROS STUFF TO SEE IT
	
    ros::init(argc, argv, "corner_visu");
    ros::NodeHandle nh("~");
	
	
// 	ros::Publisher marker_pub_graph = nh.advertise<visualization_msgs::Marker>("visualization_marker_graph", 10);
// 	ros::Publisher marker_pub_graph2 = nh.advertise<visualization_msgs::Marker>("visualization_marker_graph2", 10);
	ros::Publisher marker_pub_graph_accurate = nh.advertise<visualization_msgs::Marker>("accurate_markers", 10);
	ros::Publisher ndt_map_pub= nh.advertise<ndt_map::NDTMapMsg>("lastgraphmap", 10);
	
	AASS::das::NDTCornerVisu visu;
	visualization_msgs::Marker origins;
	visualization_msgs::Marker origins_accurate;
	
	visu.rvizPrint(corners, origins, "/world");
	visu.rvizPrintAccurateCorners(corners, origins_accurate, "/world");
	
	ndt_map::NDTMapMsg mapmsg;
	bool good = perception_oru::toMessage(&map, mapmsg, "/world");
	
	while (ros::ok()){
		visualization_msgs::Marker origins_sec;
// 		visu.printNei(map, corners, origins_sec, "/world");
// 		marker_pub_graph.publish(origins);
// 		marker_pub_graph2.publish(origins_sec);
		marker_pub_graph_accurate.publish(origins_accurate);
		ndt_map_pub.publish(mapmsg);
		ros::spinOnce();
	}

    return 0;
}