#include <ndt_map/ndt_map_hmt.h>
#include <ndt_map/lazy_grid.h>
#include <ndt_map/cell_vector.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"

#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <Eigen/Eigen>

static int ctr = 0, ctr2 = 0;
static boost::mutex mutex;
static bool fresh = false;

void ndtCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg,cloud);

    ROS_INFO ("Received %d data points with the following fields: %s", (int)(msg->width * msg->height),
              pcl::getFieldsList (*msg).c_str ());

    lslgeneric::NDTMap nd(new lslgeneric::LazyGrid(2));

    Eigen::Affine3d T;
    T.setIdentity();
    nd.addPointCloud(T.translation(),cloud);
    nd.computeNDTCells();

    ROS_INFO("Loaded point cloud");
    if(ctr%10 == 0) {
	nd.writeToJFF("maps02.jff");
    }
    ctr++;

}

int
main (int argc, char** argv)
{

    ros::init(argc, argv, "ndt_builder");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("points2_in", 10, ndtCallback);
    ros::spin();

    return (0);
}



