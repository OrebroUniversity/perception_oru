#include <boost/program_options.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <cmath>
#include "ndt_map/ndt_map.h"
#include <fstream>
class map_converter{
  std::string ndt_map_name;
  std::string occ_map_name;
  double ndt_resolution;
  double occ_resolution;
  double lik_tr;
  int width, height;
  lslgeneric::NDTMap *ndtMap;
  lslgeneric::LazyGrid *mapGrid;
  std::vector<std::vector<int> > map;
public:
  map_converter(int argc, char **argv){
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("ndt-file", boost::program_options::value<std::string>(&ndt_map_name), "location of the ndt map you want to conver")
      ("ndt-resolution", boost::program_options::value<double>(&ndt_resolution)->default_value(1.), "resolution of the map")
      ("occ-file", boost::program_options::value<std::string>(&occ_map_name), "name of new occupancy map (do not put extension)")
      ("occ-resolution", boost::program_options::value<double>(&occ_resolution)->default_value(1.), "desired resolution")
      ("lik_tr", boost::program_options::value<double>(&lik_tr)->default_value(0.01), "likelihood treshold the lower value the more conservative map")
      ;
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (!vm.count("ndt-file") || !vm.count("ndt-resolution") || !vm.count("occ-file") || !vm.count("occ-resolution"))
      {
        std::cout << "Missing arguments.\n";
        std::cout << desc << "\n";
        exit(0);
      }
    if (vm.count("help"))
      {
        std::cout << desc << "\n";
        exit(0);
      }
    LoadMap();
    //std::cout << "Map loaded.\n";
    BuildMap();
    //std::cout << "Map built.\n";
    SaveMap();
    //std::cout << "Map saved.\n";
    saveYAMLFile();
    //std::cout << "Yaml created.\n";
  }

private:
  int LoadMap(){
    FILE * jffin;
    jffin = fopen(ndt_map_name.c_str(),"r+b");
    mapGrid=new lslgeneric::LazyGrid(ndt_resolution);
    ndtMap=new lslgeneric::NDTMap(mapGrid);
    if (ndtMap->loadFromJFF(jffin)<0){
      return -1;
    }
    return 0;
  }

  void BuildMap(){
    double xs, ys, zs;
    double xc, yc, zc;
    double minx, miny, maxx,maxy;
    ndtMap->getCentroid(xc,yc,zc);
    ndtMap->getGridSizeInMeters(xs,ys,zs);
    minx=xc-xs/2.0;
    miny=yc-ys/2.0;
    maxx=xc+xs/2.0;
    maxy=yc+ys/2.0;
    height = round(xs/occ_resolution);
    width = round(ys/occ_resolution);

    //std::cout << "minx: " << minx << "\n";
    //std::cout << "miny: " << miny << "\n";
    //std::cout << "maxx: " << maxx << "\n";
    //std::cout << "maxy: " << maxy << "\n";
    //std::cout << "H:"<<height<< " W:"<<width<<"\n";
    //std::cout << "occ_resolution: " << occ_resolution << "\n";
    //std::cout << "xc: " << xc <<"yc: " << yc << "\n";
    //std::cout << "xs: " << xs <<"ys: " << ys << "\n";

    int h=0;
    map.resize(width, std::vector<int>(height));

    for(double current_x=-xs/2+occ_resolution;current_x<xs/2-occ_resolution;current_x+=occ_resolution){

      //std::cout << "first loop \n";
      int w=0;
      for(double current_y=-ys/2+occ_resolution;current_y<ys/2-occ_resolution;current_y+=occ_resolution){
        //std::cout << "H:"<<h<< " W:"<<w<<"\n";
        lslgeneric::NDTCell *check_cell;
        pcl::PointXYZ p;
        p.x=current_x;
        p.y=current_y;
        p.z=0.0;
        if(ndtMap->getCellAtPoint(p,check_cell)){
          //std::cout<<"init"<<std::endl;
          if(check_cell->getOccupancy()<0.0){
            map[w][h]=1;
          }
          else if(check_cell->getOccupancy()>0.0){
            //std::cout<<"lik "<<check_cell->getLikelihood(p)<<std::endl;
            if(check_cell->getLikelihood(p)>lik_tr){
            // Eigen::Vector3d m=check_cell->getMean();
            // Eigen::Vector3d e=check_cell->getEvals();
            // double d=sqrt((m[0]-current_x)*(m[0]-current_x)+(m[1]-current_y)*(m[1]-current_y));
            // double de=sqrt(e[0]*e[0]+e[1]*e[1]);
            // if(d<=de)
              //std::cout << "W:"<<w<< " H:"<<h<<"\n";
              map[w][h]=1;
            }
            else{
              //std::cout << "else\n";
              map[w][h] = 0;
            }
          }
        }
        w++;
      }
      h++;

    }
  }

  void SaveMap(){
    std::cout<<"saving map"<<std::endl;
    std::string png_map_file=occ_map_name;
    png_map_file+=".pgm";
    std::ofstream map_file (png_map_file.c_str());
    if (map_file.is_open())
      {
        map_file<<"P2\n";
        map_file<<width<<" "<<height<<std::endl;
        map_file<<"1\n";
        // 12.10.15 to fit the inverted map
        //for(int w=0;w<width;w++){
        //  for(int h=height-1;h>=0;h--){
        ////////////////////////////////////////////
        for(int w=width-1;w>=0;w--){
            for(int h=0;h<height;h++){
            map_file<<map[w][h]<<" ";
          }
          map_file<<std::endl;
        }
        map_file.close();
      }
    else std::cout << "Unable to open file";
  }

  void saveYAMLFile(){
    double xs, ys, zs;
    double xc, yc, zc;
    double cenx, ceny;
    ndtMap->getCentroid(xc,yc,zc);
    ndtMap->getGridSizeInMeters(xs,ys,zs);
    cenx=xc-xs/2.0;
    ceny=yc-ys/2.0;
    std::cout<<"saving yaml file"<<std::endl;
    std::string yaml_map_file=occ_map_name;
    yaml_map_file+=".yaml";
    std::ofstream map_file (yaml_map_file.c_str());
    if (map_file.is_open()){
      map_file<<"image: "<<occ_map_name<<".pgm\n";
      map_file<<"resolution: "<<occ_resolution<<std::endl;
      map_file<<"origin: ["<<cenx<<", "<<ceny<<", 0.0]\n";
      map_file<<"occupied_thresh: 0.5\n";
      map_file<<"free_thresh: 0.5\n";
      map_file<<"negate: 0\n";
      std::cout<<"YAML FILE SAVED, PLEAS EDIT PATH TO MAP!"<<std::endl;
      map_file.close();
    }
    else std::cout << "Unable to open file";
  }
};

int main(int argc, char **argv){
  map_converter mc(argc,argv);
}
