#include <ndt_viz.h>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;

int main(int argc, char **argv){

    std::string base_name, point_type;
    double resolution;

    po::options_description desc("Allowed options");
    desc.add_options()
	("help", "produce help message")
	("file-name", po::value<string>(&base_name), "location of the ndt map you want to view")
	("point-type", po::value<string>(&point_type), "type of the underlying index. supported types are XYZ (default), XYZI and XYZRGB")
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

    std::cout<<"loading "<<base_name<<" at resolution "<<resolution<<" with point type "<<point_type<<std::endl;
    
    if(point_type == "XYZI") {
	//load NDT map. Only Lazy grid supported!
	lslgeneric::NDTMap<pcl::PointXYZI> ndmap(new lslgeneric::LazyGrid<pcl::PointXYZI>(resolution));
	ndmap.loadFromJFF(base_name.c_str());
	//create visualizer
	NDTViz<pcl::PointXYZI> *viewer = new NDTViz<pcl::PointXYZI>(true);
	//display map
	viewer->plotNDTSAccordingToOccupancy(-1,&ndmap);
	while(viewer->win3D->isOpen()){
	    if (viewer->win3D->keyHit())
	    {
		mrptKeyModifier kmods;
		int key = viewer->win3D->getPushedKey(&kmods);
		printf("Key pushed: %c (%i) - modifiers: 0x%04X\n",char(key),key,kmods);

		if (key==MRPTK_RIGHT) viewer->win3D->setCameraAzimuthDeg( viewer->win3D->getCameraAzimuthDeg() + 5 );
		if (key==MRPTK_LEFT)  viewer->win3D->setCameraAzimuthDeg( viewer->win3D->getCameraAzimuthDeg() - 5 );
		
		if(key =='o'){
		    viewer->plotNDTSAccordingToOccupancy(-1,&ndmap);
		}
		if(key =='c'){
		    viewer->plotNDTSAccordingToClass(-1,&ndmap);
		}
		if(key =='p'){
		    viewer->plotNDTSAccordingToCost(-1,10000,&ndmap);
		}

		if(key =='h'){
		    fprintf(stderr,"[h] help\n");
		    fprintf(stderr,"[o] Just plot acording to occupancy \n");
		    fprintf(stderr,"[c] plot acording to class and occupancy \n");
		    fprintf(stderr,"[p] plot acording to cost-to-go from the path planner \n");
		}

		if(key =='q'){
		    break;
		}
	    }
	    usleep(100*1000);
	}
	delete viewer;
    } else {
	//load NDT map. Only Lazy grid supported!
	lslgeneric::NDTMap<pcl::PointXYZ> ndmap(new lslgeneric::LazyGrid<pcl::PointXYZ>(resolution));
	ndmap.loadFromJFF(base_name.c_str());
	//create visualizer
	NDTViz<pcl::PointXYZ> *viewer = new NDTViz<pcl::PointXYZ>(true);
	//display map
	viewer->plotNDTSAccordingToOccupancy(-1,&ndmap);
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

}

