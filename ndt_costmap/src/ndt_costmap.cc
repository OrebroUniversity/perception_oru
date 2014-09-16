#include <ndt_visualisation/ndt_viz.h>
#include <ndt_costmap/costmap.h>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;
using namespace lslgeneric;
  
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;


int main(int argc, char **argv){

    std::string base_name;
    double resolution;

    po::options_description desc("Allowed options");
    desc.add_options()
	("help", "produce help message")
	("file-name", po::value<string>(&base_name), "location of the ndt map you want to view")
	("resolution", po::value<double>(&resolution)->default_value(1.), "resolution of the map (necessary due to bug in loading, should be fixed soon)")
	;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (!vm.count("file-name") || !vm.count("resolution"))
    {
	cout << "Missing arguments.\n";
	cout << desc << "\n";
	return 1;
    }
    if (vm.count("help"))
    {
	cout << desc << "\n";
	return 1;
    }

    std::cout<<"loading "<<base_name<<" at resolution "<<resolution<<std::endl;

    //load NDT map. Only Lazy grid supported!
    lslgeneric::NDTMap<pcl::PointXYZI> ndmap(new lslgeneric::LazyGrid<pcl::PointXYZI>(resolution));
    ndmap.loadFromJFF(base_name.c_str());
    
    NDTCostmap<pcl::PointXYZI> cmap(&ndmap);
    cmap.processMap(1.5, -1);
    cmap.saveCostMapIncr("cmap",0.25);
    
    //create visualizer
    NDTViz<pcl::PointXYZI> *viewer = new NDTViz<pcl::PointXYZI>(true);
    //display map
    viewer->plotNDTSAccordingToClass(-1,&ndmap);
    while(viewer->win3D->isOpen()){
	if (viewer->win3D->keyHit())
	{
	    mrptKeyModifier kmods;
	    int key = viewer->win3D->getPushedKey(&kmods);
	    printf("Key pushed: %c (%i) - modifiers: 0x%04X\n",char(key),key,kmods);

	    if (key==MRPTK_RIGHT) viewer->win3D->setCameraAzimuthDeg( viewer->win3D->getCameraAzimuthDeg() + 5 );
	    if (key==MRPTK_LEFT)  viewer->win3D->setCameraAzimuthDeg( viewer->win3D->getCameraAzimuthDeg() - 5 );

	    if(key =='q'){
		break;
	    }
	}
	usleep(100*1000);
    }
    delete viewer;

}
