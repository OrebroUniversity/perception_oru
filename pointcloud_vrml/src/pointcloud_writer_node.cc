#include<pcl/ros/conversions.h>
#include<iostream>
#include<fstream>
#include"sensor_msgs/PointCloud2.h"
#include"sensor_msgs/point_cloud_conversion.h"
#include<cstring>
#include<Eigen/Eigen>
#include<pointcloud_utils.h>
#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "pcl/io/pcd_io.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>

using namespace std;

class WriterNode
{
protected:
    // Our NodeHandle
    ros::NodeHandle nh_;
    ros::Subscriber pc_;
    ros::Subscriber pose_;
    // Use the vector as a cyclic buffer (increment with std::rotate).
    pcl::PointCloud<pcl::PointXYZ> sensor_pc;
    int dumpNumber;
    const char* dumpPath;

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tinit, T;
    bool inited;

    boost::mutex data_mutex;

    FILE *fout;
public:
    // Constructor
    WriterNode(const char* _dumpPath) : dumpNumber(0)
    {
        dumpPath = _dumpPath;
        pc_ = nh_.subscribe("/camera/rgb/points", 10, &WriterNode::log, this);
        pose_ = nh_.subscribe("/pose", 10, &WriterNode::updatePose, this);
        char fn[500];

        inited = false;
        snprintf (fn,499,"%s/pose.dat",dumpPath);
        fout = fopen(fn,"w");
    }

    ~WriterNode()
    {
        fclose(fout);
    }

    // Callback
    void log(const sensor_msgs::PointCloud2::ConstPtr& msg_in)
    {
        char fname [500];
        snprintf (fname,499,"%s/cloud%03d.wrl",dumpPath,dumpNumber);
        dumpNumber ++;

        pcl::fromROSMsg (*msg_in, sensor_pc);
        lslgeneric::writeToVRML<pcl::PointXYZ>(fname,sensor_pc);
        printf ("Dumped %s\n",fname);

        Eigen::Quaterniond qlocal;
        Eigen::Vector3d vlocal;
        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tlocal;

        data_mutex.lock();
        if(!inited)
        {
            inited = true;
            Tinit = T;
            cout<<"Tinit:: "<<Tinit.translation().transpose()<<" r "<<Tinit.rotation().eulerAngles(0,1,2).transpose()<<endl;
        }
        Tlocal = Tinit.inverse()*T;
        data_mutex.unlock();

        vlocal = Tlocal.translation();
        qlocal = Tlocal.rotation();
        fprintf(fout,"0 %lf %lf %lf %lf %lf %lf %lf\n",
                vlocal(0),vlocal(1),vlocal(2),qlocal.x(),qlocal.y(),qlocal.z(),qlocal.w());

    }

    void updatePose(const nav_msgs::Odometry::ConstPtr& msg_in)
    {
        data_mutex.lock();
        Eigen::Vector3d v;
        v<<msg_in->pose.pose.position.x,msg_in->pose.pose.position.y,msg_in->pose.pose.position.z;
        Eigen::Quaterniond q = Eigen::Quaterniond(msg_in->pose.pose.orientation.w,msg_in->pose.pose.orientation.x,
                               msg_in->pose.pose.orientation.y,msg_in->pose.pose.orientation.z);
        T = Eigen::Translation<double,3>(v)*q;

        data_mutex.unlock();

    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};



int main(int argc, char**argv)
{

    if(argc!=2)
    {
        cout<<"Usage: "<<argv[0]<<" PATH_TO_POINTCLOUDS \n";
        return -1;
    }
    ros::init(argc, argv, "pointcloud_writer");

    WriterNode nd(argv[1]);
    ros::spin();

}
