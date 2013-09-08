#include <pointcloud_utils.h>

using namespace std;
using namespace lslgeneric;

int main(int argc, char **argv)
{

    if(argc!=2)
    {
        cout<<argv[0]<<" nameOfScan.wrl\n";
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ> pc = readVRML<pcl::PointXYZ>(argv[1]), pcs;
    char fname[500];
    snprintf(fname,499,"%s_subsampled.wrl",argv[1]);

    double retain_percentage = 0.05;
    for(int i=0; i<pc.points.size(); i++)
    {
        double r = (double)rand()/(double)RAND_MAX;
        if(r < retain_percentage)
        {
            pcl::PointXYZ pt = pc.points[i];
            pt.x = pt.x*100;
            pt.y = pt.y*100;
            pt.z = pt.z*100;
            pcs.points.push_back(pt);
        }
    }
    writeToVRML<pcl::PointXYZ>(fname,pcs);

    return 0;
}
