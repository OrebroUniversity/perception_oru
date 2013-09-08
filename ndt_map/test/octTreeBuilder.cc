#include <oc_tree.h>

#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include "Eigen/Core"
#include <cstdio>

static int ctr = 0;

void octCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    //build the oct tree and save to disk, update counter
//    pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg,cloud);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud,centroid);
    pcl::PointXYZ c;
    c.x=centroid(0);
    c.y=centroid(1);
    c.z=centroid(2);

    lslgeneric::OctTree<pcl::PointXYZ> tr(c,10,10,10,new lslgeneric::NDTCell<pcl::PointXYZ>());

    for(unsigned int i=0; i<cloud.points.size(); ++i)
    {
        tr.addPoint(cloud.points[i]);
    }

    char fname[50];
    snprintf(fname,49,"oct_tree%05d.wrl",ctr);
    FILE* fout = fopen (fname,"w");
    if(fout == NULL)
    {
        return;
    }
    fprintf(fout,"#VRML V2.0 utf8\n");
    tr.writeToVRML(fout);
    lslgeneric::writeToVRML<pcl::PointXYZ>(fout,cloud);
    ctr++;
    fclose(fout);


}

int
main (int argc, char** argv)
{

    ros::init(argc, argv, "oct_builder");
    ros::NodeHandle n;
    ros::Subscriber chatter_sub = n.subscribe("points2_in", 10, octCallback);
    ros::spin();

    return (0);
}



