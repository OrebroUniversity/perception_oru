#include <limits>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <cmath>
#include "ndt_map/ndt_map_hmt.h"
#include <fstream>
#include "ndt_map/ndt_cell.h"
#include "ros/ros.h"
class map_converter {
	std::string ndt_map_directory;
	std::string occ_map_name;
	double ndt_resolution;
	double occ_resolution;
	double lik_tr;
	double floor_height;
	double floor_deviation;
	double robot_height;
	double xs, ys;
	double v_size_x;
	double v_size_y;
	double v_size_z;
	double max_range;
	int width, height;
	lslgeneric::NDTMapHMT *ndtMap;
	lslgeneric::LazyGrid *mapGrid;
	bool floor_only;
	std::vector<double> floor_levels;
	std::vector<int> floor_count;
	std::vector<std::string> map_names;
	std::vector<std::vector<std::vector<int> > >maps;
	std::vector<std::vector<int> > map;
public:
	map_converter(ros::NodeHandle private_nh){
		private_nh.param<std::string>("ndt_map_directory", ndt_map_directory, "map");
		private_nh.param<double>("ndt_resolution", ndt_resolution, 0.4);
		private_nh.param<std::string>("occ_map_name", occ_map_name, "occ");
		private_nh.param<double>("occ_resolution", occ_resolution, 0.1);
		private_nh.param<double>("floor_height", floor_height, -0.3);
		private_nh.param<double>("floor_deviation", floor_deviation, 0.15);
		private_nh.param<double>("lik_tr", lik_tr, 0.1);
		private_nh.param<double>("robot_height", robot_height, 1.9);
		private_nh.param<bool>("floor_only", floor_only, false);
		private_nh.param<double>("v_size_x", v_size_x, 80.0);
		private_nh.param<double>("v_size_y", v_size_y, 80.0);
		private_nh.param<double>("v_size_z", v_size_z, 10.0);
		private_nh.param("max_range", max_range, 80.0);
		//InitializeOCCMap();
		LoadMap();
		BuildMapN();
		// SaveMap();
		// saveYAMLFile();
	}

private:
	int LoadMap(){

		ndtMap = new lslgeneric::NDTMapHMT(ndt_resolution, 0, 0, 0, v_size_x, v_size_y, v_size_z, max_range, ndt_map_directory);
		Eigen::Vector3d newPos;
		newPos << 0, 0, 0;
		ndtMap->setInsertPosition(newPos);
		return 0;
	}

	double getAverageLevel(std::vector<lslgeneric::NDTCell*> floors){
		double aver = 0;

		for(size_t i = 0; i < floors.size(); i++)
			aver += floors[i]->getMean()[2];
		return aver / double(floors.size());
	}


	void BuildMapN(){
		double X_min = 0.125;//-359.125;
		double Y_min = 0.125;//-299.125;
		double X_max = 199.125;
		double Y_max = 519.125;
		double step = 0.25;
		double Z_max = 1.9;
		double Z_min = -0.3;

		std::cout << "saving map" << std::endl;
		std::string png_map_file = occ_map_name;

		png_map_file += ".pgm";
		std::ofstream map_file(png_map_file.c_str(), std::ofstream::out);
		if(map_file.is_open()){
			map_file << "P2" <<std::endl;
			map_file << int(fabs(X_max - X_min) / step) << " " << int(fabs(Y_max - Y_min) / step) <<std::endl;
			map_file << "1"<<std::endl;
			map.resize(int(fabs(Y_max - Y_min) / step), std::vector<int> (int(fabs(X_max - X_min) / step), 0));
			for(size_t i_x = 0; i_x < fabs(X_max - X_min) / step; i_x++){
				for(size_t i_y = 0; i_y < fabs(Y_max - Y_min) / step; i_y++){
					bool traversable = true;
					Eigen::Vector3d newPos;
					newPos << X_min + i_x * step, Y_min + i_y * step, 0;
					ndtMap->setInsertPosition(newPos);
					for(size_t i_z = 0; i_z < int(fabs(Z_max-Z_min)/step); i_z++){
						Eigen::Vector3d newPos;
						pcl::PointXYZ pt(X_min + i_x * step, Y_min + i_y * step, Z_min +i_z*step);
						ROS_INFO_STREAM(pt);
						lslgeneric::NDTCell *cell;
						if(ndtMap->getCellForPoint(pt, cell)){
							if (cell==NULL) {
								traversable = false;
								break;
							}
							double cz;
							cell->getCenter(cz, cz, cz);
							pcl::PointXYZ p1(X_min + i_x * step, Y_min + i_y * step, cz - ndt_resolution / 2.0);
							pcl::PointXYZ p2(X_min + i_x * step, Y_min + i_y * step, cz + ndt_resolution / 2.0);
							Eigen::Vector3d out;
							if(cell->computeMaximumLikelihoodAlongLine(p1, p2, out) > lik_tr){
								// ROS_INFO_STREAM(__LINE__);
								traversable = false;
								break;
							}
						}else {
							traversable = false;
							break;
						}
					}
					if(traversable){
						map[i_y][i_x] = 1;

					}
					else{
						map[i_y][i_x] = 0;
					}
					map_file << map[i_y][i_x] << " ";
					map_file.flush();
				}
				map_file << std::endl;
			}
		}
	}

};





	// void BuildMap(){
	//
	// 	std::vector<lslgeneric::NDTCell*> ndts = ndtMap->getAllCells();
	// 	std::vector<std::vector<lslgeneric::NDTCell*> > floors;
	// 	std::vector<float> av_level;
	// 	double sx, sy, sz;
	// 	ndtMap->getCellSizeInMeters(sx, sy, sz);
	// 	for(size_t i = 0; i < ndts.size(); i++){
	// 		if(ndts[i]->getClass() == lslgeneric::NDTCell::HORIZONTAL){
	// 			bool found = false;
	// 			for(size_t j = 0; j < floors.size(); j++){
	// 				if(av_level[j] - floor_deviation<ndts[i]->getMean()[2] && av_level[j] + floor_deviation>ndts[i]->getMean()[2]){
	// 					found = true;
	// 					floors[j].push_back(ndts[i]);
	// 					av_level[j] = getAverageLevel(floors[j]);
	// 					break;
	// 				}
	// 			}
	// 			if(!found){
	// 				av_level.push_back(ndts[i]->getMean()[2]);
	// 				std::vector<lslgeneric::NDTCell*> temp;
	// 				temp.push_back(ndts[i]);
	// 				floors.push_back(temp);
	// 			}
	//
	// 		}
	// 	}
	// 	double xs, ys, zs;
	// 	double xc, yc, zc;
	// 	double minx, miny, minz, maxx, maxy, maxz;
	// 	ndtMap->getCentroid(xc, yc, zc);
	// 	ndtMap->getGridSizeInMeters(xs, ys, zs);
	// 	minx = xc - xs / 2.0;
	// 	miny = yc - ys / 2.0;
	// 	maxx = xc + xs / 2.0;
	// 	maxy = yc + ys / 2.0;
	// 	height = abs(ceil(xs / occ_resolution));
	// 	width = abs(ceil(ys / occ_resolution));
	// 	std::vector<std::vector<std::vector<int> > >maps;
	// 	maps.resize(av_level.size(), std::vector<std::vector<int> >(width, std::vector<int>(height)));
	//
	// 	for(size_t j = 0; j < floors.size(); j++){
	// 		for(size_t k = 0; k < floors[j].size(); k++){
	// 			double ccx, ccy, ccz;
	// 			floors[j][k]->getCenter(ccx, ccy, ccz);
	// 			for(double lx = ccx - sx / 2.0 + occ_resolution / 2.0; lx <= ccx + sx / 2.0 - occ_resolution / 2.0; lx += occ_resolution){
	// 				for(double ly = ccy - sy / 2.0 + occ_resolution / 2.0; ly <= ccy + sy / 2.0 - occ_resolution / 2.0; ly += occ_resolution){
	// 					double lccz = ccz;
	// 					int X = floor((lx + sx / 2.0) / occ_resolution + 0.5) + height / 2.0;   // round((lx - (occ_resolution - xs) / 2.0) / occ_resolution);
	// 					int Y = floor((ly + sy / 2.0) / occ_resolution + 0.5) + width / 2.0;    //round((ly - (occ_resolution - ys) / 2.0) / occ_resolution);
	// 					pcl::PointXYZ p1(lx, ly, ccz - sz / 2.0);
	// 					pcl::PointXYZ p2(lx, ly, ccz + sz / 2.0);
	// 					Eigen::Vector3d out;
	// 					// ROS_INFO_STREAM(X<<" "<<Y);
	// 					// ROS_INFO_STREAM(p1<<" "<<p2<<" "<<floors[j][k]->computeMaximumLikelihoodAlongLine(p1, p2, out)<<" "<<X<<" "<<Y);
	//
	// 					if(floors[j][k]->computeMaximumLikelihoodAlongLine(p1, p2, out) > lik_tr){
	// 						bool traversable = true;
	// 						if(!floor_only){
	// 							while(lccz + 0.5 * sz - av_level[j] < robot_height){
	// 								lccz += sz;
	//
	// 								lslgeneric::NDTCell *cell;
	// 								pcl::PointXYZ refPoint(lx, ly, lccz);
	//
	// 								if(lccz + 0.5 * sz - av_level[j] < robot_height){
	// 									p1.z = lccz - sz / 2.0;
	// 									p2.z = lccz + sz / 2.0;
	// 								}
	// 								if(lccz + 0.5 * sz - av_level[j] >= robot_height){
	// 									p1.z = lccz - sz / 2.0;
	// 									p2.z = av_level[j] + robot_height;
	// 								}
	// 								//std::cout << av_level[j] << " " << p1 << " " << p2 << std::endl;
	// 								if(ndtMap->getCellAtPoint(refPoint, cell)){
	// 									if(cell->hasGaussian_ && cell->getOccupancyRescaled() > 0.9){
	//
	// 										if(cell->computeMaximumLikelihoodAlongLine(p1, p2, out) > lik_tr){
	// 											traversable = false;
	// 											//std::cout << cell->getCenter() << " break occ" << std::endl;
	// 											break;
	// 										}
	// 									}
	// 								}else{
	// 									traversable = false;
	// 									//std::cout << "break null" << std::endl;
	// 									break;
	// 								}
	//
	// 							}
	// 						}
	// 						if(traversable)
	// 							maps[j][Y][X] = 1;
	// 						//std::cout << "done" << std::endl;
	// 					}
	// 				}
	//
	// 			}
	// 		}
	// 		std::cout << "saving map" << std::endl;
	// 		std::string png_map_file = occ_map_name;
	// 		png_map_file += std::to_string(av_level[j]);
	// 		png_map_file += ".pgm";
	// 		std::ofstream map_file(png_map_file.c_str());
	// 		if(map_file.is_open()){
	// 			map_file << "P2\n";
	// 			map_file << width << " " << height << std::endl;
	// 			map_file << "1\n";
	// 			// 12.10.15 to fit the inverted map
	// 			//for(int w=0;w<width;w++){
	// 			//  for(int h=height-1;h>=0;h--){
	// 			////////////////////////////////////////////
	// 			for(int w = width - 1; w >= 0; w--){
	// 				for(int h = 0; h < height; h++)
	// 					map_file << maps[j][w][h] << " ";
	// 				map_file << std::endl;
	// 			}
	// 			map_file.close();
	// 		}else std::cout << "Unable to open file";
	// 	}
	// }

	// int InitializeOCCMap(){
	//  // get the size of the map in meters
	//  double sizeMeta;
	//
	//  std::string meta = ndt_map_directory;
	//  meta += "/metadata.txt";
	//  FILE* meta_f = fopen(meta.c_str(), "a+");
	//  if(meta_f == 0) return -1;
	//  char *line = NULL;
	//  size_t len;
	//  bool found = false;
	//  if(getline(&line, &len, meta_f) > 0 ){
	//   char *tk = strtok(line, " ");
	//   if(tk == NULL) return false;
	//   if(strncmp(tk, "VERSION", 7) == 0){
	//    tk = strtok(NULL, " ");
	//    if(tk == NULL) return false;
	//    if(strncmp(tk, "2.0", 3) == 0){
	//     if(!getline(&line, &len, meta_f) > 0 )
	//      return false;
	//     tk = strtok(line, " ");
	//     if(tk == NULL) return false;
	//     if(strncmp(tk, "SIZE", 4) != 0) return false;
	//     tk = strtok(NULL, " ");
	//     sizeMeta = atof(tk);
	//    }
	//   } else {
	//    //this is a version 1.0 file, close and re-open and go on
	//    std::cerr << "metafile version 1.0, no protection against different grid size\n";
	//    fclose(meta_f);
	//    meta_f = fopen(meta.c_str(), "a+");
	//   }
	//  }
	//  float min_x = std::numeric_limits<float>::max(),
	//        min_y = std::numeric_limits<float>::max(),
	//        max_x = std::numeric_limits<float>::min(),
	//        max_y = std::numeric_limits<float>::min();
	//  while(getline(&line, &len, meta_f) > 0 ){
	//   pcl::PointXYZ cen;
	//   char *token = strtok(line, " ");
	//   if(token == NULL) return -1;
	//   cen.x = atof(token);
	//   token = strtok(NULL, " ");
	//   if(token == NULL) return -1;
	//   cen.y = atof(token);
	//   token = strtok(NULL, " ");
	//   if(token == NULL) return -1;
	//   cen.z = atof(token);
	//   token = strtok(NULL, " ");
	//   if(token == NULL) return -1;
	//   std::string str(token);
	//   map_names.push_back(str);
	//   if(cen.x < min_x)
	//    min_x = cen.x;
	//   if(cen.y < min_y)
	//    min_y = cen.y;
	//   if(cen.x > max_x)
	//    max_x = cen.x;
	//   if(cen.y > max_y)
	//    max_y = cen.y;
	//  }
	//  min_x = min_x - sizeMeta / 2.0;
	//  min_y = min_y - sizeMeta / 2.0;
	//  max_x = max_x + sizeMeta / 2.0;
	//  max_y = max_y + sizeMeta / 2.0;
	//  xs = fabs(min_x - max_x);
	//  ys = fabs(min_y - max_y);
	//  height = abs(ceil(xs / occ_resolution));
	//  width = abs(ceil(ys / occ_resolution));
	//  for(size_t map_it = 0; map_it < map_names.size(); map_it++){
	//
	//   FILE * jffin;
	//   std::string sep = "/";
	//   std::string path = ndt_map_directory + sep + map_names[map_it];
	//   //ROS_INFO_STREAM(path);
	//   jffin = fopen(path.c_str(), "r+b");
	//   if(jffin==NULL)
	//    continue;
	//   mapGrid = new lslgeneric::LazyGrid(ndt_resolution);
	//   lslgeneric::NDTCell *ptCell = new lslgeneric::NDTCell();
	//      mapGrid->setCellType(ptCell);
	//      delete ptCell;
	//   mapGrid->loadFromJFF(jffin);
	//   ndtMap = new lslgeneric::NDTMap(mapGrid);
	//
	//   std::vector<lslgeneric::NDTCell*> ndts = ndtMap->getAllCells();
	//   double sx, sy, sz;
	//   ndtMap->getCellSizeInMeters(sx, sy, sz);
	//   for(size_t i = 0; i < ndts.size(); i++){
	//    if(ndts[i]->getClass() == lslgeneric::NDTCell::HORIZONTAL){
	//     bool found = false;
	//     for(size_t j = 0; j < floor_levels.size(); j++){
	//      if(floor_levels[j] - floor_deviation<ndts[i]->getMean()[2] && floor_levels[j] + floor_deviation>ndts[i]->getMean()[2]){
	//       found = true;
	//       floor_levels[j] = (floor_levels[j] * floor_count[j] + ndts[i]->getMean()[2]) / (floor_count[j] + 1);
	//       floor_count[j]++;
	//       break;
	//      }
	//     }
	//     if(!found){
	//      floor_levels.push_back(ndts[i]->getMean()[2]);
	//      floor_count.push_back(1);
	//     }
	//
	//    }
	//   }
	//   fclose(jffin);
	//  }
	//  maps.resize(floor_levels.size(), std::vector<std::vector<int> >(width, std::vector<int>(height)));
	// }
	//
	// void BuildMapTr(){
	//
	//  for(size_t map_it = 0; map_it < map_names.size(); map_it++){
	//
	//   FILE * jffin;
	//   std::string sep = "/";
	//   std::string path = ndt_map_directory + sep + map_names[map_it];
	//   ROS_INFO_STREAM(path);
	//   jffin = fopen(path.c_str(), "r+b");
	//
	//   if(jffin==NULL)
	//    continue;
	//   mapGrid = new lslgeneric::LazyGrid(ndt_resolution);
	//   lslgeneric::NDTCell *ptCell = new lslgeneric::NDTCell();
	//   mapGrid->setCellType(ptCell);
	//   delete ptCell;
	//   mapGrid->loadFromJFF(jffin);
	//   ndtMap = new lslgeneric::NDTMap(mapGrid);
	//   std::vector<lslgeneric::NDTCell*> ndts = ndtMap->getAllCells();
	//   ROS_INFO_STREAM( ndts .size());
	//   double sx, sy, sz;
	//   ndtMap->getCellSizeInMeters(sx, sy, sz);
	//   for(size_t i = 0; i < ndts.size(); i++){
	//    if(ndts[i]->getClass() == lslgeneric::NDTCell::HORIZONTAL){
	//     double min_dist = std::numeric_limits<double>::max();
	//     int level = -1;
	//     double z=ndts[i]->getMean()[2];
	//     for(size_t it_flor_levels = 0; it_flor_levels < floor_levels.size(); it_flor_levels++){
	//      if(fabs(floor_levels[it_flor_levels] - z) < min_dist){
	//       min_dist = fabs(floor_levels[it_flor_levels] - z);
	//       level = it_flor_levels;
	//      }
	//     }
	//     double ccx, ccy, ccz;
	//     ndts[i]->getCenter(ccx, ccy, ccz);
	//     for(double lx = ccx - sx / 2.0 + occ_resolution / 2.0; lx <= ccx + sx / 2.0 - occ_resolution / 2.0; lx += occ_resolution){
	//      for(double ly = ccy - sy / 2.0 + occ_resolution / 2.0; ly <= ccy + sy / 2.0 - occ_resolution / 2.0; ly += occ_resolution){
	//       double lccz = ccz;
	//       int X = floor((lx + sx / 2.0) / occ_resolution + 0.5) + height / 2.0;            // round((lx - (occ_resolution - xs) / 2.0) / occ_resolution);
	//       int Y = floor((ly + sy / 2.0) / occ_resolution + 0.5) + width / 2.0;             //round((ly - (occ_resolution - ys) / 2.0) / occ_resolution);
	//       pcl::PointXYZ p1(lx, ly, ccz - sz / 2.0);
	//       pcl::PointXYZ p2(lx, ly, ccz + sz / 2.0);
	//       Eigen::Vector3d out;
	//       if(ndts[i]->computeMaximumLikelihoodAlongLine(p1, p2, out) > lik_tr){
	//        bool traversable = true;
	//        if(!floor_only){
	//         while(lccz + 0.5 * sz - floor_levels[level] < robot_height){
	//          lccz += sz;
	//          lslgeneric::NDTCell *cell;
	//          pcl::PointXYZ refPoint(lx, ly, lccz);
	//          if(lccz + 0.5 * sz - floor_levels[level] < robot_height){
	//           p1.z = lccz - sz / 2.0;
	//           p2.z = lccz + sz / 2.0;
	//          }
	//          if(lccz + 0.5 * sz - floor_levels[level] >= robot_height){
	//           p1.z = lccz - sz / 2.0;
	//           p2.z = floor_levels[level] + robot_height;
	//          }
	//          if(ndtMap->getCellAtPoint(refPoint, cell)){
	//           if(cell->hasGaussian_ && cell->getOccupancyRescaled() > 0.9){
	//            if(cell->computeMaximumLikelihoodAlongLine(p1, p2, out) > lik_tr){
	//             traversable = false;
	//             break;
	//            }
	//           }
	//          }else{
	//           traversable = false;
	//           //std::cout << "break null" << std::endl;
	//           break;
	//          }
	//         }
	//        }
	//        if(traversable)
	//         maps[level][Y][X] = 1;
	//       }
	//      }
	//     }
	//    }
	//   }
	//  }
	//  for (size_t j = 0;  j<floor_levels.size() ; j++) {
	//
	//  std::cout << "saving map" << std::endl;
	//  std::string png_map_file = occ_map_name;
	//  png_map_file += std::to_string(floor_levels[j]);
	//  png_map_file += ".pgm";
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
	//     map_file << maps[j][w][h] << " ";
	//    map_file << std::endl;
	//   }
	//   map_file.close();
	//  }else std::cout << "Unable to open file";
	//  }
	// }

// void BuildMapLik(){
//
//  std::vector<lslgeneric::NDTCell*> ndts = ndtMap->getAllCells();
//  std::vector<std::vector<lslgeneric::NDTCell*> > floors;
//  std::vector<float> av_level;
//  double sx, sy, sz;
//  ndtMap->getCellSizeInMeters(sx, sy, sz);
//  for(size_t i = 0; i < ndts.size(); i++){
//   if(ndts[i]->getClass() == lslgeneric::NDTCell::HORIZONTAL){
//    bool found = false;
//    for(size_t j = 0; j < floors.size(); j++){
//     if(av_level[j] - floor_deviation<ndts[i]->getMean()[2] && av_level[j] + floor_deviation>ndts[i]->getMean()[2]){
//      found = true;
//      floors[j].push_back(ndts[i]);
//      av_level[j] = getAverageLevel(floors[j]);
//      break;
//     }
//    }
//    if(!found){
//     av_level.push_back(ndts[i]->getMean()[2]);
//     std::vector<lslgeneric::NDTCell*> temp;
//     temp.push_back(ndts[i]);
//     floors.push_back(temp);
//    }
//
//   }
//  }
//  double xs, ys, zs;
//  double xc, yc, zc;
//  double minx, miny, minz, maxx, maxy, maxz;
//  ndtMap->getCentroid(xc, yc, zc);
//  ndtMap->getGridSizeInMeters(xs, ys, zs);
//  minx = xc - xs / 2.0;
//  miny = yc - ys / 2.0;
//  maxx = xc + xs / 2.0;
//  maxy = yc + ys / 2.0;
//  height = abs(ceil(xs / occ_resolution));
//  width = abs(ceil(ys / occ_resolution));
//  std::vector<std::vector<std::vector<int> > >maps;
//  maps.resize(av_level.size(), std::vector<std::vector<int> >(width, std::vector<int>(height)));
//
//  for(size_t j = 0; j < floors.size(); j++){
//   for(size_t k = 0; k < floors[j].size(); k++){
//    double ccx, ccy, ccz;
//    floors[j][k]->getCenter(ccx, ccy, ccz);
//    for(double lx = ccx - sx / 2.0 + occ_resolution / 2.0; lx <= ccx + sx / 2.0 - occ_resolution / 2.0; lx += occ_resolution){
//     for(double ly = ccy - sy / 2.0 + occ_resolution / 2.0; ly <= ccy + sy / 2.0 - occ_resolution / 2.0; ly += occ_resolution){
//      double lccz = ccz;
//      int X = floor((lx + sx / 2.0) / occ_resolution + 0.5) + height / 2.0;        // round((lx - (occ_resolution - xs) / 2.0) / occ_resolution);
//      int Y = floor((ly + sy / 2.0) / occ_resolution + 0.5) + width / 2.0;         //round((ly - (occ_resolution - ys) / 2.0) / occ_resolution);
//      pcl::PointXYZ p1(lx, ly, ccz - sz / 2.0);
//      pcl::PointXYZ p2(lx, ly, ccz + sz / 2.0);
//      Eigen::Vector3d out;
//      // ROS_INFO_STREAM(X<<" "<<Y);
//      // ROS_INFO_STREAM(p1<<" "<<p2<<" "<<floors[j][k]->computeMaximumLikelihoodAlongLine(p1, p2, out)<<" "<<X<<" "<<Y);
//
//      if(floors[j][k]->computeMaximumLikelihoodAlongLine(p1, p2, out) > lik_tr){
//       bool traversable = true;
//       if(!floor_only){
//        while(lccz + 0.5 * sz - av_level[j] < robot_height){
//         lccz += sz;
//
//         lslgeneric::NDTCell *cell;
//         pcl::PointXYZ refPoint(lx, ly, lccz);
//
//         if(lccz + 0.5 * sz - av_level[j] < robot_height){
//          p1.z = lccz - sz / 2.0;
//          p2.z = lccz + sz / 2.0;
//         }
//         if(lccz + 0.5 * sz - av_level[j] >= robot_height){
//          p1.z = lccz - sz / 2.0;
//          p2.z = av_level[j] + robot_height;
//         }
//         //std::cout << av_level[j] << " " << p1 << " " << p2 << std::endl;
//         if(ndtMap->getCellAtPoint(refPoint, cell)){
//          if(cell->hasGaussian_ && cell->getOccupancyRescaled() > 0.9){
//
//           if(cell->computeMaximumLikelihoodAlongLine(p1, p2, out) > lik_tr){
//            traversable = false;
//            //std::cout << cell->getCenter() << " break occ" << std::endl;
//            break;
//           }
//          }
//         }else{
//          traversable = false;
//          //std::cout << "break null" << std::endl;
//          break;
//         }
//
//        }
//       }
//       if(traversable)
//        maps[j][Y][X] = 1;
//       //std::cout << "done" << std::endl;
//      }
//     }
//
//    }
//   }
//   std::cout << "saving map" << std::endl;
//   std::string png_map_file = occ_map_name;
//   png_map_file += std::to_string(av_level[j]);
//   png_map_file += ".pgm";
//   std::ofstream map_file(png_map_file.c_str());
//   if(map_file.is_open()){
//    map_file << "P2\n";
//    map_file << width << " " << height << std::endl;
//    map_file << "1\n";
//    // 12.10.15 to fit the inverted map
//    //for(int w=0;w<width;w++){
//    //  for(int h=height-1;h>=0;h--){
//    ////////////////////////////////////////////
//    for(int w = width - 1; w >= 0; w--){
//     for(int h = 0; h < height; h++)
//      map_file << maps[j][w][h] << " ";
//     map_file << std::endl;
//    }
//    map_file.close();
//   }else std::cout << "Unable to open file";
//  }
// }

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
//};

int main(int argc, char **argv){
	ROS_INFO("Starting node");
	ros::init(argc, argv, "off_line_velodyne_test");
	ros::NodeHandle nh;
	ros::NodeHandle parameters("~");
	map_converter mc(parameters);
}
