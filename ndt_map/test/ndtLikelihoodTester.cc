#include <NDTMap.hh>
#include <OctTree.hh>
#include <AdaptiveOctTree.hh>
#include <PointCloudUtils.hh>

#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>

#include <LazzyGrid.hh>

#include <fstream>
#include <sstream>

using namespace std;

static int ctr = 0;
int
main (int argc, char** argv)
{

    if(argc!=4)
    {
        //cout<<"usage: likelihood_test full_point_cloud laser_point_cloud_name tof_pointcloud \n";
        cout<<"usage: ./likelihood_test [full_point_cloud] [path_to_save_data] [path_to_save_CSV]\n";
        return(-1);
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> laserCloud;
    pcl::PointCloud<pcl::PointXYZ> tofCloud;
    pcl::PointCloud<pcl::PointXYZI> outCloudLaser;
    pcl::PointCloud<pcl::PointXYZI> outCloudTOF;

    cloud = lslgeneric::readVRML(argv[1]);
    //laserCloud = lslgeneric::readVRML(argv[2]);
    //tofCloud = lslgeneric::readVRML(argv[3]);
    string path= argv[2];
    string ValidationData = argv[3];
    ofstream out;
    out.open (ValidationData.c_str());
    if (!out.is_open())
    {
        cout<<"file not open";
    }

    for (int k=0; k<10; k++)
    {
        char fname[100];
        snprintf(fname,99,"%s/SR4000_%d.wrl",path.c_str(),k);
        tofCloud = lslgeneric::readVRML(fname);

        snprintf(fname,99,"%s/LaserScanner_%d.wrl",path.c_str(),k);
        laserCloud = lslgeneric::readVRML(fname);


        //   lslgeneric::NDTMap nd(new lslgeneric::AdaptiveOctTree());
        lslgeneric::NDTMap nd(new lslgeneric::LazzyGrid(0.2));
        nd.loadPointCloud(cloud);

        nd.computeNDTCells();
        snprintf(fname,99,"%s/ndt_map.wrl",path.c_str());
        nd.writeToVRML(fname);


        double maxLikelihood = INT_MIN;
        double sumLikelihoods = 0;

        outCloudLaser.points.resize(laserCloud.points.size());
        outCloudTOF.points.resize(tofCloud.points.size());

        //loop through points and compute likelihoods LASER
        for(int i=0; i<laserCloud.points.size(); i++)
        {
            pcl::PointXYZ thisPt = laserCloud.points[i];
            double likelihood = nd.getLikelihoodForPointWithInterpolation(thisPt);
            //double likelihood = nd.getLikelihoodForPoint(thisPt);
            pcl::PointXYZI outPt;
            outPt.x = thisPt.x;
            outPt.y = thisPt.y;
            outPt.z = thisPt.z;
            outPt.intensity = likelihood;
            sumLikelihoods += likelihood;
            maxLikelihood = (likelihood > maxLikelihood) ?
                            likelihood : maxLikelihood;
            outCloudLaser.points[i] = outPt;
        }
        cout<<"LASER max likelihood "<<maxLikelihood<<endl;
        cout<<"LASER sum likelihoods "<<sumLikelihoods<<endl;
        cout<<"LASER average likelihood "<<sumLikelihoods/laserCloud.points.size()<<endl;
        out<<sumLikelihoods<<","<<sumLikelihoods/laserCloud.points.size()<<","<<maxLikelihood<<endl;
        //normalize for display
        //compute standart deviation
        for(int i=0; i<laserCloud.points.size(); i++)
        {
            outCloudLaser.points[i].intensity /= maxLikelihood;

        }
        snprintf(fname,99,"%s/likelihood_LASER_%d.wrl",path.c_str(),k);
        lslgeneric::writeToVRML(fname,outCloudLaser);


        maxLikelihood = INT_MIN;
        sumLikelihoods = 0;
        //loop through points and compute likelihoods TOF
        for(int i=0; i<tofCloud.points.size(); i++)
        {
            pcl::PointXYZ thisPt = tofCloud.points[i];
            double likelihood = nd.getLikelihoodForPointWithInterpolation(thisPt);
            //double likelihood = nd.getLikelihoodForPoint(thisPt);
            pcl::PointXYZI outPt;
            outPt.x = thisPt.x;
            outPt.y = thisPt.y;
            outPt.z = thisPt.z;
            outPt.intensity = likelihood;
            sumLikelihoods += likelihood;
            maxLikelihood = (likelihood > maxLikelihood) ?
                            likelihood : maxLikelihood;
            outCloudTOF.points[i] = outPt;
        }
        cout<<"TOF max likelihood "<<maxLikelihood<<endl;
        cout<<"TOF sum likelihoods "<<sumLikelihoods<<endl;
        cout<<"TOF average likelihood "<<sumLikelihoods/tofCloud.points.size()<<endl;
        out<<sumLikelihoods<<","<<sumLikelihoods/tofCloud.points.size()<<","<<maxLikelihood<<endl;
        //normalize for display
        //compute standart deviation
        for(int i=0; i<tofCloud.points.size(); i++)
        {
            outCloudTOF.points[i].intensity /= maxLikelihood;

        }
        snprintf(fname,99,"%s/likelihood_TOF_%d.wrl",path.c_str(),k);
        lslgeneric::writeToVRML(fname,outCloudTOF);
    }

    return (0);
}



