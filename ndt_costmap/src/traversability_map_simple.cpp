
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
	double floor_height;
	double floor_deviation;
	double robot_height;
	int width, height;
	lslgeneric::NDTMap *ndtMap;
	lslgeneric::LazyGrid *mapGrid;
	bool floor_only;
public:
	map_converter(ros::NodeHandle private_nh){
		private_nh.param<std::string>("ndt_map_name", ndt_map_name, "map.jff");
		private_nh.param<double>("ndt_resolution", ndt_resolution, 0.4);
		private_nh.param<std::string>("occ_map_name", occ_map_name, "occ");
		private_nh.param<double>("occ_resolution", occ_resolution, 0.1);
		private_nh.param<double>("floor_height", floor_height, -0.3);
		private_nh.param<double>("floor_deviation", floor_deviation, 0.15);
		private_nh.param<double>("lik_tr", lik_tr, 0.1);
		private_nh.param<double>("robot_height", robot_height, 1.9);
		private_nh.param<bool>("floor_only", floor_only, false);
		LoadMap();
		BuildMap();
		// SaveMap();
		// saveYAMLFile();
	}

private:
	int LoadMap(){
		FILE * jffin;

		jffin = fopen(ndt_map_name.c_str(), "r+b");
		mapGrid = new lslgeneric::LazyGrid(ndt_resolution);
		ndtMap = new lslgeneric::NDTMap(mapGrid);
		if(ndtMap->loadFromJFF(jffin) < 0)
			return -1;
		return 0;
	}

	double getAverageLevel(std::vector<lslgeneric::NDTCell*> floors){
		double aver = 0;

		for(size_t i = 0; i < floors.size(); i++)
			aver += floors[i]->getMean()[2];
		return aver / double(floors.size());
	}

	void BuildMap(){

		std::vector<lslgeneric::NDTCell*> ndts = ndtMap->getAllCells();
		std::vector<lslgeneric::NDTCell*> floors;
		std::vector<float> av_level;
		double sx, sy, sz;
		ndtMap->getCellSizeInMeters(sx, sy, sz);
		for(size_t i = 0; i < ndts.size(); i++){
			//if(ndts[i]->getClass() == lslgeneric::NDTCell::HORIZONTAL){ // commented 14.03
				//bool found = false;
				//for(size_t j = 0; j < floors.size(); j++){
					//if(av_level[j] - floor_deviation<ndts[i]->getMean()[2] && av_level[j] + floor_deviation>ndts[i]->getMean()[2]){
					//if(floor_height - floor_deviation<ndts[i]->getMean()[2] && floor_height + floor_deviation>ndts[i]->getMean()[2]){
					double ccx, ccy, ccz;
					ndts[i]->getCenter(ccx, ccy, ccz);
					if(floor_height - floor_deviation<ccz && floor_height + floor_deviation>ccz){
						//found = true;
						floors.push_back(ndts[i]);
						//av_level[j] = getAverageLevel(floors[j]);
						//break;
					}
				//}
				// if(!found && ndts[i]->getMean()[2]<floor_deviation+floor_height && ndts[i]->getMean()[2]>floor_height-floor_deviation ){
				// 	av_level.push_back(ndts[i]->getMean()[2]);
				// 	std::vector<lslgeneric::NDTCell*> temp;
				// 	temp.push_back(ndts[i]);
				// 	floors.push_back(temp);
				// }

			//}// commented 14.03
		}
		double xs, ys, zs;
		double xc, yc, zc;
		double minx, miny, minz, maxx, maxy, maxz;
		ndtMap->getCentroid(xc, yc, zc);
		ndtMap->getGridSizeInMeters(xs, ys, zs);
		minx = xc - xs / 2.0;
		miny = yc - ys / 2.0;
		maxx = xc + xs / 2.0;
		maxy = yc + ys / 2.0;
		height = abs(ceil(xs / occ_resolution));
		width = abs(ceil(ys / occ_resolution));
		std::vector<std::vector<int> > maps;
		maps.resize(width, std::vector<int>(height));

		//for(size_t j = 0; j < floors.size(); j++){
			for(size_t k = 0; k < floors.size(); k++){
				double ccx, ccy, ccz;
				floors[k]->getCenter(ccx, ccy, ccz);
				for(double lx = ccx - sx/2.0 + occ_resolution / 2.0; lx <= ccx + sx/2.0 - occ_resolution / 2.0; lx += occ_resolution){
					for(double ly = ccy - sy/2.0 + occ_resolution / 2.0; ly <= ccy + sy/2.0 - occ_resolution / 2.0; ly += occ_resolution){
						double lccz = ccz;
						int X = floor((lx+sx/2.0)/occ_resolution+0.5) + height/2.0;// round((lx - (occ_resolution - xs) / 2.0) / occ_resolution);
						int Y = floor((ly+sy/2.0)/occ_resolution+0.5) + width/2.0;//round((ly - (occ_resolution - ys) / 2.0) / occ_resolution);
						pcl::PointXYZ p1(lx, ly, ccz - sz / 2.0);
						pcl::PointXYZ p2(lx, ly, ccz + sz / 2.0);
						Eigen::Vector3d out;
						// ROS_INFO_STREAM(X<<" "<<Y);
						// ROS_INFO_STREAM(p1<<" "<<p2<<" "<<floors[j][k]->computeMaximumLikelihoodAlongLine(p1, p2, out)<<" "<<X<<" "<<Y);

						//if(floors[k]->hasGaussian_){//computeMaximumLikelihoodAlongLine(p1, p2, out) > lik_tr){
							bool traversable = true;
							if(!floor_only){
								while(lccz + 0.5 * sz - floor_height < robot_height){
									lccz += sz;

									lslgeneric::NDTCell *cell;
									pcl::PointXYZ refPoint(lx, ly, lccz);

									if(lccz + 0.5 * sz - floor_height < robot_height){
										p1.z = lccz - sz / 2.0;
										p2.z = lccz + sz / 2.0;
									}
									if(lccz + 0.5 * sz - floor_height >= robot_height){
										p1.z = lccz - sz / 2.0;
										p2.z = floor_height + robot_height;
									}
									//std::cout << av_level[j] << " " << p1 << " " << p2 << std::endl;
									if(ndtMap->getCellAtPoint(refPoint, cell)){
										if(cell->hasGaussian_&&cell->getOccupancyRescaled()>0.9){

											if(cell->computeMaximumLikelihoodAlongLine(p1, p2, out) > lik_tr){
												traversable = false;
												//std::cout << cell->getCenter() << " break occ" << std::endl;
												break;
											}
										}
									}else{
										traversable = false;
										//std::cout << "break null" << std::endl;
										break;
									}

								}
							}
							if(traversable){
								maps[Y][X] = 1;
								//std::cout << "done" << std::endl;
							}
						}
					//}

				}
			}
			std::cout << "saving map" << std::endl;
			std::string png_map_file = occ_map_name;
			png_map_file += std::to_string(floor_height);
			png_map_file += ".pgm";
			std::ofstream map_file(png_map_file.c_str());
			if(map_file.is_open()){
				map_file << "P2\n";
				map_file << width << " " << height << std::endl;
				map_file << "1\n";
				// 12.10.15 to fit the inverted map
				//for(int w=0;w<width;w++){
				//  for(int h=height-1;h>=0;h--){
				////////////////////////////////////////////
				for(int w = width - 1; w >= 0; w--){
					for(int h = 0; h < height; h++)
						map_file << maps[w][h] << " ";
					map_file << std::endl;
				}
				map_file.close();
			}else std::cout << "Unable to open file";
		//}
	}
// void BuildMap(){
//  double xs, ys, zs;
//  double xc, yc, zc;
//  double minx, miny, minz, maxx, maxy, maxz;
//
//  ndtMap->getCentroid(xc, yc, zc);
//  ndtMap->getGridSizeInMeters(xs, ys, zs);
//  minx = xc - xs / 2.0;
//  miny = yc - ys / 2.0;
//  maxx = xc + xs / 2.0;
//  maxy = yc + ys / 2.0;
//  height = round(xs / occ_resolution);
//  width = round(ys / occ_resolution);
//  std::cout << height << " " << width << std::endl;
//  int h = 0;
//  map.resize(width, std::vector<int>(height));
//  for(double current_x = -xs / 2 + occ_resolution; current_x < xs - occ_resolution; current_x += occ_resolution){
//   int w = 0;
//   for(double current_y = -ys / 2 + occ_resolution; current_y < ys - occ_resolution; current_y += occ_resolution){
//
//
//    lslgeneric::NDTCell *check_cell;
//    pcl::PointXYZ p;
//    p.x = current_x;
//    p.y = current_y;
//    p.z = 0.0;
//    if(ndtMap->getCellAtPoint(p, check_cell)){
//     //std::cout<<"init"<<std::endl;
//     if(check_cell->getOccupancy() < 0.0)
//      map[w][h] = 1;
//     else if(check_cell->getOccupancy() > 0.0){
//      //std::cout<<"lik "<<check_cell->getLikelihood(p)<<std::endl;
//      if(check_cell->getLikelihood(p) < lik_tr)
//       // Eigen::Vector3d m=check_cell->getMean();
//       // Eigen::Vector3d e=check_cell->getEvals();
//       // double d=sqrt((m[0]-current_x)*(m[0]-current_x)+(m[1]-current_y)*(m[1]-current_y));
//       // double de=sqrt(e[0]*e[0]+e[1]*e[1]);
//       // if(d<=de)
//       map[w][h] = 1;
//     }
//    }
//
//
//
//    w++;
//   }
//   h++;
//  }
// }
//
// void SaveMap(){
//  std::cout << "saving map" << std::endl;
//  std::string png_map_file = occ_map_name;
//  png_map_file += ".png";
//  std::ofstream map_file(png_map_file.c_str());
//  if(map_file.is_open()){
//   map_file << "P2\n";
//   map_file << width << " " << height << std::endl;
//   map_file << "1\n";
//   // 12.10.15 to fit the inverted map
//   //for(int w=0;w<width;w++){
//   //  for(int h=height-1;h>=0;h--){
//   ////////////////////////////////////////////
//   for(int w = width - 1; w >= 0; w--){
//    for(int h = 0; h < height; h++)
//     map_file << map[w][h] << " ";
//    map_file << std::endl;
//   }
//   map_file.close();
//  }else std::cout << "Unable to open file";
// }
//
// void saveYAMLFile(){
//  double xs, ys, zs;
//  double xc, yc, zc;
//  double cenx, ceny;
//
//  ndtMap->getCentroid(xc, yc, zc);
//  ndtMap->getGridSizeInMeters(xs, ys, zs);
//  cenx = xc - xs / 2.0;
//  ceny = yc - ys / 2.0;
//  std::cout << "saving yaml file" << std::endl;
//  std::string yaml_map_file = occ_map_name;
//  yaml_map_file += ".yaml";
//  std::ofstream map_file(yaml_map_file.c_str());
//  if(map_file.is_open()){
//   map_file << "image: " << occ_map_name << ".png\n";
//   map_file << "resolution: " << occ_resolution << std::endl;
//   map_file << "origin: [" << cenx << ", " << ceny << ", 0.0]\n";
//   map_file << "occupied_thresh: 0.5\n";
//   map_file << "free_thresh: 0.5\n";
//   map_file << "negate: 0\n";
//   std::cout << "YAML FILE SAVED, PLEAS EDIT PATH TO MAP!" << std::endl;
//   map_file.close();
//  }else std::cout << "Unable to open file";
// }
};

int main(int argc, char **argv){
	ROS_INFO("Starting node");
	ros::init(argc, argv, "off_line_velodyne_test");
	ros::NodeHandle nh;
	ros::NodeHandle parameters("~");
	map_converter mc(parameters);
}
