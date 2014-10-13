#include <ndt_map/ndt_map.h>
#include <ndt_map/lazy_grid.h>
#include <pointcloud_vrml/pointcloud_utils.h>

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>
#include <cstring>

// #include <opencv/cv.h>
// #include <opencv/highgui.h>
// #include <boost/shared_ptr.hpp>
// #include <boost/thread/mutex.hpp>
// #include <cv_bridge/CvBridge.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/Image.h>

#include <Eigen/Eigen>

using namespace std;

int main (int argc, char** argv)
{
    if(argc < 1)
    {
        cout << "[ USAGE ] jffLoadTest \n";
        exit(1);
    }

    cout << "Started loadTest\n";

    char fname[] = "test_jff.wrl";

    lslgeneric::NDTMap<pcl::PointXYZ> nd(new lslgeneric::LazyGrid<pcl::PointXYZ>(0.2));
    if (nd.loadFromJFF("LazyGrid.jff") < 0)
        cout << "loading from jff failed\n";
    //nd.writeToVRML(fname);

    cout << "Finished loadTest\n";

    return 0;
}

