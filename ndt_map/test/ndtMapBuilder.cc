#include <ndt_map_hmt.h>
#include <oc_tree.h>
#include <lazy_grid.h>
#include <cell_vector.h>

#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
//#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <Eigen/Eigen>

static int ctr = 0, ctr2 = 0;
static cv::Mat color_img;
static boost::mutex mutex;
static bool fresh = false;

void ndtCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg,cloud);

    ROS_INFO ("Received %d data points with the following fields: %s", (int)(msg->width * msg->height),
              pcl::getFieldsList (*msg).c_str ());

    //lslgeneric::NDTMap<pcl::PointXYZ> nd(new lslgeneric::OctTree<pcl::PointXYZ>());
    //lslgeneric::NDTMap<pcl::PointXYZ> nd(new lslgeneric::LazyGrid<pcl::PointXYZ>(0.2));
    lslgeneric::NDTMapHMT<pcl::PointXYZ> nd(new lslgeneric::LazyGrid<pcl::PointXYZ>(0.2),0,0,0,10,10,2,3.0);

    Eigen::Affine3d T;
    T.setIdentity();
    nd.addPointCloud(T.translation(),cloud);
    nd.computeNDTCells();

    ROS_INFO("Loaded point cloud");
    nd.writeTo("maps/");
    ctr++;

}

void ndtCallbackDepthImg(const sensor_msgs::ImageConstPtr &msg)
{

    cv::Mat depth_img(cv_bridge::toCvCopy(msg, "")->image);
    cv::Mat color_img_loc;

    mutex.lock();
    if(!fresh)
    {
        mutex.unlock();
        return;
    }
    color_img.copyTo(color_img_loc);
    mutex.unlock();


    ROS_INFO ("Received image data");

    //lslgeneric::NDTMap<pcl::PointXYZ> nd(new lslgeneric::OctTree<pcl::PointXYZ>());
    //lslgeneric::NDTMap<pcl::PointXYZ> nd(new lslgeneric::LazyGrid<pcl::PointXYZ>(0.2));
    lslgeneric::NDTMap<pcl::PointXYZ> nd(new lslgeneric::CellVector<pcl::PointXYZ>());

    // TODO - this are the values from the Freiburg 1 camera.
    double fx = 517.3;
    double fy = 516.5;
    double cx = 318.6;
    double cy = 255.3;
    std::vector<double> dist(5);
    dist[0] = 0.2624;
    dist[1] = -0.9531;
    dist[2] = -0.0054;
    dist[3] = 0.0026;
    dist[4] = 1.1633;
    double ds = 1.035; // Depth scaling factor.
    bool isFloat = (depth_img.depth() == CV_32F);
    double scale = 1; //0.0002;
    if(!isFloat)
    {
        scale = 0.002; //raw scale conversion
    }

    lslgeneric::DepthCamera<pcl::PointXYZ> cameraParams (fx,fy,cx,cy,dist,ds*scale,isFloat);
//   pcl::PointCloud<pcl::PointXYZ> pc;
//   cameraParams.convertDepthImageToPointCloud(depth_img, pc);

    std::vector<cv::KeyPoint> kpts;
    cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("SURF");
    detector->detect(color_img_loc, kpts);

    ROS_INFO("found %d keypoints",kpts.size());
    size_t support = 7;
    double maxVar = 0.3;
//    nd.loadDepthImage(depth_img, cameraParams);
    //nd.loadDepthImageFeatures(depth_img, kpts, support, maxVar, cameraParams);
    nd.loadDepthImageFeatures(depth_img, kpts, support, maxVar, cameraParams,true);
    nd.computeNDTCells();

    lslgeneric::CellVector<pcl::PointXYZ> *cv = dynamic_cast<lslgeneric::CellVector<pcl::PointXYZ>* > (nd.getMyIndex());
    if(cv!=NULL)
    {
        cv->cleanCellsAboveSize(maxVar);
    }

    ROS_INFO("Loaded point cloud");
    char fname[50];
    snprintf(fname,49,"ndt_map%05d.wrl",ctr2);
    nd.writeToVRML(fname);
    /*    snprintf(fname,49,"cloud%05d.wrl",ctr2);
        lslgeneric::writeToVRML(fname,pc);
        snprintf(fname,49,"depth%05d.png",ctr2);
        cv::imwrite(fname,depth_img);
        snprintf(fname,49,"depth_raw%05d.png",ctr2);
        cv::imwrite(fname,depth_img_raw);
    */
    ctr2++;
    //std::exit(0);
}

void callColorImg(const sensor_msgs::ImageConstPtr &msg)
{

    //sensor_msgs::CvBridge _imBridge;
    mutex.lock();
    fresh = true;
    //cv::Mat tmp_mat = cv::Mat(_imBridge.imgMsgToCv(msg, "mono8"));
    cv::Mat tmp_mat(cv_bridge::toCvCopy(msg, "mono8")->image);
    tmp_mat.copyTo(color_img);
    mutex.unlock();

}

int
main (int argc, char** argv)
{

    ros::init(argc, argv, "ndt_builder");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("points2_in", 10, ndtCallback);
    ros::Subscriber sub2 = n.subscribe("depth_image", 10, ndtCallbackDepthImg);
    ros::Subscriber sub3 = n.subscribe("color_image", 10, callColorImg);
    ros::spin();

    return (0);
}



