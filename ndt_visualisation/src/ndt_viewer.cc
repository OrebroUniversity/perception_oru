#include <ndt_visualisation/ndt_viz.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;


void printUsage() {
    fprintf(stderr,"========== USAGE =========\n");
    fprintf(stderr,"[h] help\n");
    fprintf(stderr,"[o] Just plot acording to occupancy \n");
    fprintf(stderr,"[c] plot acording to class and occupancy \n");
    fprintf(stderr,"[p] plot acording to cost-to-go from the path planner \n");
    fprintf(stderr,"[t] plot traversability map \n");
    fprintf(stderr,"[q] quit \n");
    fprintf(stderr,"==========================\n");
}

int main(int argc, char **argv){

    std::string base_name;
    double resolution;

    po::options_description desc("Allowed options");
    desc.add_options()
	("help", "produce help message")
	("file-name", po::value<std::string>(&base_name), "location of the ndt map you want to view")
	("resolution", po::value<double>(&resolution)->default_value(1.), "resolution of the map (necessary due to bug in loading, should be fixed soon)")
	;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (!vm.count("file-name") || !vm.count("resolution"))
    {
        std::cout << "Missing arguments.\n";
        std::cout << desc << "\n";
	return 1;
    }
    if (vm.count("help"))
    {
        std::cout << desc << "\n";
	return 1;
    }

    std::cout<<"loading "<<base_name<<" at resolution "<<resolution<<std::endl;
    printUsage();
    
    //load NDT map. Only Lazy grid supported!
    lslgeneric::NDTMap ndmap(new lslgeneric::LazyGrid(resolution));
    ndmap.loadFromJFF(base_name.c_str());
    //create visualizer
    NDTViz *viewer = new NDTViz(true);
    //display map
    viewer->win3D->start_main_loop_own_thread();
    viewer->plotNDTSAccordingToOccupancy(-1,&ndmap);
    while(viewer->win3D->isOpen()){
	//viewer->win3D->process_events();
	if (viewer->win3D->keyHit())
	{
	    int key = viewer->win3D->getPushedKey();
	    printf("Key pushed: %c (%i)\n",char(key),key);
            
            if(key =='o'){
                viewer->plotNDTSAccordingToOccupancy(-1,&ndmap);
            }
            if(key =='c'){
                viewer->plotNDTSAccordingToClass(-1,&ndmap);
            }
            if(key =='p'){
                viewer->plotNDTSAccordingToCost(-1,10000,&ndmap);
            }
            if(key =='t'){
                viewer->plotNDTTraversabilityMap(&ndmap);
            }
            if(key =='h'){
                printUsage();
            }

            if(key =='q'){
		break;
	    }
	}
	usleep(1000);
    }
    delete viewer;


}

