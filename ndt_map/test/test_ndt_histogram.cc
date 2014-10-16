#include <ndt_map/ndt_histogram.h>
//#include <ndt_map/oc_tree.h>
#include <ndt_map/pointcloud_utils.h>
#include <ndt_map/lazy_grid.h>

#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>

#include <fstream>

//#define FULLTESTER

using namespace std;

int main (int argc, char** argv) {

    if(argc!=3){
        cout<<"usage: histTest point_cloud1 point_cloud2\n";
        return(-1);
    }
    pcl::PointCloud<pcl::PointXYZ> cloud, cloud2, cloud3;
    pcl::PointCloud<pcl::PointXYZI> outCloud;

    // lslgeneric::OctTree<pcl::PointXYZ> tr;
    // tr.BIG_CELL_SIZE = 2;
    // tr.SMALL_CELL_SIZE = 0.2;
    lslgeneric::LazyGrid tr(0.5);

#ifdef FULLTESTER
    // std::string fname_str = argv[1];
    // int nclouds = atoi(argv[2]);
    // //lslgeneric::AdaptiveOctTree::MIN_CELL_SIZE = 0.01;

    // ofstream logger ("similarity.m");
    // logger<< "S = [";
    // struct timeval tv_start, tv_end;
    // gettimeofday(&tv_start,NULL);
    // lslgeneric::NDTHistogram *array  = new lslgeneric::NDTHistogram[nclouds];
    // for(int i=0; i<nclouds; i++){
    //     char cloudname [500];
        
    //     lslgeneric::NDTMap nd(&tr);
    //     nd.loadPointCloud(cloud);
    //     nd.computeNDTCells();
    //     array[i] = lslgeneric::NDTHistogram(nd);
    // }
    // gettimeofday(&tv_end,NULL);
    // double avg_build = (tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.;
    // avg_build = avg_build/nclouds;
    // cout<<"building histograms took "<<avg_build<<" msec per scan\n";

    // gettimeofday(&tv_start,NULL);
    // for(int i=0; i<nclouds; i++)
    // {
    //     for(int j=0; j<nclouds; j++)
    //     {
    //         logger<<array[j].getSimilarity(array[i])<<" ";
    //     }
    //     logger<<";\n";
    //     cout<<" I "<<i<<endl;
    // }

    // gettimeofday(&tv_end,NULL);
    // double avg_match = (tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.;
    // avg_match = avg_match/(nclouds*nclouds);
    // cout<<"matching histograms took "<<avg_match<<" msec per scan\n";

    // logger<<"];\n";
#else

    pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], cloud2);
    //lslgeneric::NDTMap nd(new lslgeneric::LazzyGrid(5));
    lslgeneric::NDTMap nd(&tr);
    nd.loadPointCloud(cloud);
    //lslgeneric::NDTMap nd2(new lslgeneric::LazzyGrid(5));
    lslgeneric::NDTMap nd2(&tr);
    nd2.loadPointCloud(cloud2);

    nd.computeNDTCells();
    nd2.computeNDTCells();

    lslgeneric::NDTHistogram nh(nd);
    lslgeneric::NDTHistogram nh2(nd2);
    cout<<"1 =========== \n";
    nh.printHistogram(true);
    cout<<"2 =========== \n";
    nh2.printHistogram(true);

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;

    nh2.bestFitToHistogram(nh,T,true);
    cout<<" ==================== \n Transform R "<<T.rotation()<<"\nt "<<T.translation().transpose()<<endl;

    cout<<"scan similarity is "<<nh2.getSimilarity(nh)<<endl;
    cloud3 = lslgeneric::transformPointCloud(T,cloud2);
    //lslgeneric::NDTMap nd3(new lslgeneric::LazzyGrid(5));
    lslgeneric::NDTMap nd3(&tr);
    nd3.loadPointCloud(cloud3);
    nd3.computeNDTCells();

    lslgeneric::NDTHistogram nh3(nd3);
    cout<<"3 =========== \n";
    nh3.printHistogram(true);


    char fname[50];
  
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > directions = nh.directions;
    cout<<"direction = [";
    for(int i=0; i<directions.size(); i++)
    {
        cout<<directions[i].transpose()<<";";
    }
    cout<<"];\n";
#endif



    return (0);
}



