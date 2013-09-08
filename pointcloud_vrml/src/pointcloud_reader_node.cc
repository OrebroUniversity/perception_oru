#include <pcl/point_cloud.h>
#include "pcl/point_types.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "pcl/io/pcd_io.h"
#include <pointcloud_utils.h>

int
main (int argc, char** argv)
{
    if(argc<2)
    {
        ROS_INFO("wrong number of arguments!");
        return(-1);
    }

    sensor_msgs::PointCloud2 cloudMSG;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    //load vrml file into cloud
    cloud = lslgeneric::readVRML<pcl::PointXYZ>(argv[1]);
    pcl::toROSMsg(cloud,cloudMSG);
    ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloudMSG.width * cloudMSG.height),
              argv[1], pcl::getFieldsList (cloudMSG).c_str ());

    //create a broadcaster node
    ros::init(argc, argv, "pointcloud_vrml");
    ros::NodeHandle n;
    ros::Publisher vrmlPublisher =
        n.advertise<sensor_msgs::PointCloud2>("points2_in", 10);
    ros::Rate loop_rate(0.1);
    int count = 0;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, 0));
    cloudMSG.header.frame_id = "/pc_frame";

    while (ros::ok())
    {
        cloudMSG.header.stamp = ros::Time::now();
        vrmlPublisher.publish(cloudMSG);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/pc_frame"));
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return (0);
}



