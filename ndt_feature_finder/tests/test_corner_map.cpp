
#include "ndt_feature_finder/ndt_corner.hpp"

int main(){
	
	
	std::string file = "/home/malcolm/Documents/basement2d_map.jff";
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
	
// 	exit(0);
	
	perception_oru::ndt_feature_finder::NDTCorner corners;
	std::cout << "Searching for corners" << std::endl;
	auto ret = corners.getAllCorners(map);
	std::cout << "Found " << ret.size() << " Corners " << std::endl;
	
}