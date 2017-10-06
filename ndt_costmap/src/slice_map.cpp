#include <vector>
#include <string>
#include <Eigen/Core>
#include <cmath>
#include "ndt_map/ndt_map.h"
#include <fstream>
#include "ndt_map/ndt_cell.h"
#include "ros/ros.h"
class map_converter {
  std::string ndt_map_name;
  std::string occ_map_name;
  double ndt_resolution;
  double occ_resolution;
  double lik_tr;
  double sample_height;
  double floor_deviation;
  double robot_height;
  int width, height, breadth;
  perception_oru::NDTMap *ndtMap;
  perception_oru::LazyGrid *mapGrid;
  bool floor_only;
public:
  map_converter(ros::NodeHandle private_nh){
    private_nh.param<std::string>("ndt_map_name", ndt_map_name, "map.jff");
    private_nh.param<double>("ndt_resolution", ndt_resolution, 0.4);
    private_nh.param<std::string>("occ_map_name", occ_map_name, "occ");
    private_nh.param<double>("occ_resolution", occ_resolution, 0.1);
    private_nh.param<double>("sample_height", sample_height, -0.3);
    private_nh.param<double>("floor_deviation", floor_deviation, 0.15);
    private_nh.param<double>("lik_tr", lik_tr, 0.1);
    private_nh.param<double>("robot_height", robot_height, 1.9);
    private_nh.param<bool>("floor_only", floor_only, false);
    LoadMap();
    BuildMap();
  }

private:
  int LoadMap(){
    FILE * jffin;
    jffin = fopen(ndt_map_name.c_str(), "r+b");
    mapGrid = new perception_oru::LazyGrid(ndt_resolution);
    ndtMap = new perception_oru::NDTMap(mapGrid);
    if(ndtMap->loadFromJFF(jffin) < 0)
      return -1;
    return 0;
  }

  double getAverageLevel(std::vector<perception_oru::NDTCell*> floors){
    double aver = 0;

    for(size_t i = 0; i < floors.size(); i++)
      aver += floors[i]->getMean()[2];
    return aver / double(floors.size());
  }

  void BuildMap(){
    double xs, ys, zs;
    double xc, yc, zc;
    double minx, miny, minz, maxx, maxy, maxz;
    ndtMap->getCentroid(xc, yc, zc);
    ndtMap->getGridSizeInMeters(xs, ys, zs);
    minx = xc - xs / 2.0;
    miny = yc - ys / 2.0;
    minz = zc - zs / 2.0;
    maxx = xc + xs / 2.0;
    maxy = yc + ys / 2.0;
    maxz = zc + zs / 2.0;
    height = abs(ceil(xs / occ_resolution));
    width = abs(ceil(ys / occ_resolution));
    std::vector<std::vector<double> > maps;
    maps.resize(width, std::vector<double>(height));

    ROS_INFO_STREAM(xc<<" "<<yc<<" "<<zc);
    ROS_INFO_STREAM(xs<<" "<<ys<<" "<<zs);
    ROS_INFO_STREAM(minx<<" "<<miny<<" "<<minz);
    ROS_INFO_STREAM(height<<" "<<width);

    perception_oru::NDTCell* ptCell = NULL;
    for (int h=0;h<height;h++){
      for (int w=0;w<width;w++){
		pcl::PointXYZ pt(minx+occ_resolution/2+h*occ_resolution,miny+occ_resolution/2+w*occ_resolution,sample_height);
	if(!ndtMap->getCellAtPoint(pt,ptCell)){
//	  ROS_INFO_STREAM("no cell");
	  //Cell does not exist, it's unexplored (gray)
	  maps[w][h]=1;
	}
	else{
//	  ROS_INFO_STREAM(ptCell->getOccupancy());
	  if(ptCell->getLikelihood(pt)>=lik_tr){
	    //cell exists and the point is likely -> white
	    maps[w][h]=2;
	  }
	  else{
	    if(ptCell->getOccupancy()<0){
	      //the point is not likely, but the cell is not occupied -> white
	      maps[w][h]=2;
	    }
	    else
	      if(ptCell->getLikelihood(pt)<lik_tr)
	      {
		 //occupied
	         maps[w][h]=0;
	     }
	  }    
	}
      }
    }
			std::cout << "saving map" << std::endl;
			std::string png_map_file = occ_map_name;
			png_map_file += std::to_string(sample_height);
			png_map_file += ".pgm";
			std::ofstream map_file(png_map_file.c_str());
			if(map_file.is_open()){
				map_file << "P2\n";
				map_file << width << " " << height << std::endl;
				map_file << "3\n";
				for(int w = width - 1; w >= 0; w--){
					for(int h = 0; h < height; h++)
						map_file << maps[w][h] << " ";
					map_file << std::endl;
				}
				map_file.close();
			}else std::cout << "Unable to open file";

    
  }
};

int main(int argc, char **argv){
  ROS_INFO("Starting node");
  ros::init(argc, argv, "off_line_velodyne_test");
  ros::NodeHandle nh;
  ros::NodeHandle parameters("~");
  map_converter mc(parameters);
}
