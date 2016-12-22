#include <vector>
#include <string>
#include <ndt_map/ndt_map.h>
#include <boost/program_options.hpp>
namespace lslgeneric {
	class LevelSplit {
		std::string mapFile;
		lslgeneric::NDTMap* ndtMap;
		lslgeneric::LazyGrid* mapGrid;
		std::string map_prefix;
		std::string map_file_name;
		int x_or;                                                            //*< number of cells along x axis
		int y_or;                                                            //*< number of cells along y axis
		int z_or;                                                            //*< number of cells along z axis
		double gsx, gsy, gsz;
		double cx, cy, cz;
		double resolution;
	public:
		int LoadMap(){
			FILE * jffin;

			jffin = fopen(map_file_name.c_str(), "r+b");
			mapGrid = new lslgeneric::LazyGrid(resolution);
			ndtMap = new lslgeneric::NDTMap(mapGrid);
			if(ndtMap->loadFromJFF(jffin) < 0)
				return -1;
			return 0;
		}
		LevelSplit(std::string map_file_name_, std::string map_prefix_, double resolution_){
			this->map_prefix = map_prefix_;
			this->resolution = resolution_;
			this->map_file_name = map_file_name_;
			if(LoadMap() < 0)
				std::cerr << "opening map failed" << "\n";
			ndtMap->getGridSize(x_or, y_or, z_or);
			ndtMap->getGridSizeInMeters(gsx, gsy, gsz);
			ndtMap->getCentroid(cx, cy, cz);
			for(int z = 0; z < z_or; ++z){

				NDTCell *prototype = new lslgeneric::NDTCell();
				lslgeneric::LazyGrid* localMapGrid = new lslgeneric::LazyGrid(gsx, gsy, 3 * resolution,                        //gsz,
				                                                              resolution, resolution, resolution,
				                                                              cx, cy, 0.0,
				                                                              prototype);
				//localMapGrid->initializeAll();
				lslgeneric::NDTMap* localNdtMap = new lslgeneric::NDTMap(localMapGrid);
				for(int y = 0; y < y_or; ++y){
					for(int x = 0; x < x_or; ++x){
						NDTCell* cell = ndtMap->getCellAtID(x, y, z);
						if(cell != NULL){
							pcl::PointXYZ centerCell;
							centerCell = cell->getCenter();
							Eigen::Vector3d meanCell = cell->getMean();
							meanCell(2)=meanCell(2) - centerCell.z;
							centerCell.z=0.0;
							cell->setCenter(centerCell);
							cell->setMean(meanCell);
							localNdtMap->insertCell(*cell);
						}
					}
				}
				std::string level_map_name;
				level_map_name = map_prefix + "_" + std::to_string(z) + ".jff";
				std::cout << "saving map " << level_map_name << "\n";
				localNdtMap->writeToJFF(level_map_name.c_str());
				delete localNdtMap;
			}
		}

	};
}

int main(int argc, char **argv){
	std::string map_file_name_;
	std::string map_prefix_;
	double resolution_;
	boost::program_options::options_description desc("Allowed options");

	desc.add_options()
	 ("help", "produce help message")
	 ("file-name", boost::program_options::value<std::string>(&map_file_name_), "ndt-map to split")
	 ("prefix", boost::program_options::value<std::string>(&map_prefix_)->default_value("map"), "prefix_for_sequence")
	 ("resolution", boost::program_options::value<double>(&resolution_)->default_value(1.), "resolution of the map (necessary due to bug in loading, should be fixed soon)")
	;
	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);
	if( vm.count("help") || argc == 1){
		std::cout << desc << "\n";
		return 1;
	}
	lslgeneric::LevelSplit ls(map_file_name_, map_prefix_, resolution_);

}
