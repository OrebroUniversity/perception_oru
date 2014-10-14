// Visualizer of feature f2f registration.
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/program_options.hpp>

#define DO_DEBUG_PROC

#include <ndt_feature_reg/ndt_frame.h>
#include <ndt_feature_reg/ndt_frame_proc.h>
#include <ndt_feature_reg/ndt_frame_viewer.h>

#include <ndt_map/depth_camera.h>
#include <ndt_registration/ndt_matcher_d2d_feature.h>
#include <ndt_map/pointcloud_utils.h>

#include <cv.h>
#include <highgui.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

using namespace ndt_feature_reg;
using namespace pcl;
using namespace cv;
using namespace std;
using namespace lslgeneric;
namespace po = boost::program_options;

int main(int argc, char** argv)
{
    cout << "--------------------------------------------------" << endl;
    cout << "Small test program of the frame matching between 2" << endl;
    cout << "pair of depth + std (RGB) images. Each image pair " << endl;
    cout << "is assumed to have 1-1 correspondance, meaning    " << endl;
    cout << "that for each pixel in the std image we have the  " << endl;
    cout << "corresponding depth / disparity pixel at the same " << endl;
    cout << "location in the depth image.                      " << endl;
    cout << "--------------------------------------------------" << endl;

    string std1_name, depth1_name, std2_name, depth2_name, pc1_name, pc2_name;
    double scale, max_inldist_xy, max_inldist_z;
    int nb_ransac;
    int support_size;
    double maxVar;

    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("debug", "print debug output")
    ("visualize", "visualize the output")
    ("depth1", po::value<string>(&depth1_name), "first depth image")
    ("depth2", po::value<string>(&depth2_name), "second depth image")
    ("std1", po::value<string>(&std1_name), "first standard image")
    ("std2", po::value<string>(&std2_name), "second standard image")
//	  ("pc1", po::value<string>(&pc1_name), "first pointcloud")
//	  ("pc2", po::value<string>(&pc2_name), "second pointcloud")
    ("scale", po::value<double>(&scale)->default_value(0.0002), "depth scale (depth = scale * pixel value)")
    ("max_inldist_xy", po::value<double>(&max_inldist_xy)->default_value(0.1), "max inlier distance in xy related to the camera plane (in meters)")
    ("max_inldist_z", po::value<double>(&max_inldist_z)->default_value(0.1), "max inlier distance in z - depth (in meters)")
    ("nb_ransac", po::value<int>(&nb_ransac)->default_value(100), "max number of RANSAC iterations")
    ("ident", "don't use the pe matching as input to the registration")
//	  ("nb_points", po::value<int>(&nb_points)->default_value(21), "number of surrounding points")
    ("support_size", po::value<int>(&support_size)->default_value(5), "width of patch around each feature in pixels")
    ("maxVar", po::value<double>(&maxVar)->default_value(0.05), "max variance of ndt cells")
    ("skip_matching", "skip the matching step");
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        cout << desc << "\n";
        return 1;
    }
    bool debug = vm.count("debug");
    bool visualize = vm.count("visualize");
    bool ident = vm.count("ident");
    bool skip_matching = vm.count("skip_matching");
    if (debug)
        cout << "scale : " << scale << endl;
    if (!(vm.count("depth1") && vm.count("depth2")))
    {
        cout << "Check depth data input - quitting\n";
        return 1;
    }

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
    lslgeneric::DepthCamera<pcl::PointXYZ> cameraParams;

    // Load the files.
    //pcl::PCDReader reader;
    double t0,t1,t2,t3;

    if (debug)
        cout << "loading imgs : " << std1_name << " and " << std2_name << endl;

    size_t support = support_size;

    NDTFrame *frame1 = new NDTFrame();
    NDTFrame *frame2 = new NDTFrame();
    frame1->img = cv::imread(std1_name, 0);
    frame2->img = cv::imread(std2_name, 0);
    frame1->supportSize = support;
    frame2->supportSize = support;
    frame1->maxVar = maxVar;
    frame2->maxVar = maxVar;

    /*     cout<<"img channels "<<frame1.img.channels()<<endl;
         cout<<"img depth "<<frame1.img.depth()<<endl;
         cout<<"img channels "<<frame2.img.channels()<<endl;
    */
    NDTFrameProc proc(nb_ransac, max_inldist_xy, max_inldist_z);

    if (debug)
        cout << "loading depth img : " << depth1_name << " and " << depth2_name << endl;

    t0 = getDoubleTime();
    frame1->depth_img = cv::imread(depth1_name, CV_LOAD_IMAGE_ANYDEPTH); // CV_LOAD_IMAGE_ANYDEPTH is important to load the 16bits image
    frame2->depth_img = cv::imread(depth2_name, CV_LOAD_IMAGE_ANYDEPTH);
    t1 = getDoubleTime();

    bool isFloat = (frame1->depth_img.depth() == CV_32F);
    cameraParams = DepthCamera<pcl::PointXYZ>(fx,fy,cx,cy,dist,ds*scale,isFloat);
    cameraParams.setupDepthPointCloudLookUpTable(frame1->depth_img.size());

    frame1->cameraParams = cameraParams;
    frame2->cameraParams = cameraParams;
    t2 = getDoubleTime();
    cout << "load: " << t1-t0 << endl;
    cout << "lookup: " << t2-t1 << endl;

    t1 = getDoubleTime();
    proc.addFrameIncremental(frame1,skip_matching);
    proc.addFrameIncremental(frame2,skip_matching);
    t2 = getDoubleTime();
    cout<<"add frames: "<<t2-t1 <<endl;

    /*
     proc.addFrame(frame1);
     proc.addFrame(frame2);
     t1 = getDoubleTime();
     proc.processFrames(skip_matching);
     t2 = getDoubleTime();
     cout<<"process per frame: "<<(t2-t1)/2<<endl;

     */
    NDTFrameViewer viewer(&proc);

    viewer.showFeaturePC();
    viewer.showMatches(proc.pe.inliers);

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;

    /*
    proc.detectKeypoints(frame1);
    proc.detectKeypoints(frame2);

    t1 = getDoubleTime();
    proc.calcDescriptors(frame1);
    proc.calcDescriptors(frame2);
    t2 = getDoubleTime();
    cout <<" calc descriptors: "<<t2-t1<<endl;

    if (debug)
    	  cout << "Computing NDT ..." << endl;
    t1 = getDoubleTime();
    frame1.computeNDT();
    frame2.computeNDT();
    t2 = getDoubleTime();
    cout << "compute NDT : " << t2 - t1 << endl;
    if (debug)
    	  cout << " - done." << endl;

    NDTFrameViewer<pcl::PointXYZ> viewer(&frame1, &frame2, proc);

    // Do the feature matching
    t1 = getDoubleTime();
    PoseEstimator<pcl::PointXYZ,pcl::PointXYZ> pe(nb_ransac, max_inldist_xy, max_inldist_z);
    frame1.assignPts();
    frame2.assignPts();

    pe.estimate(frame1, frame2);
    t2 = getDoubleTime();
    cout<<" pose estimation: "<<t2-t1<<endl;

    if (visualize)
    {
     cv::Mat display;
     drawMatches(frame1.img, frame1.kpts, frame2.img, frame2.kpts, pe.inliers, display);
     const std::string window_name = "matches";
     cv::namedWindow(window_name,0);
     cv::imshow(window_name, display);
     cv::waitKey(0);
    }

    #if 0
    // -------------------------- only the pe --------------------------
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::transformPointCloud<pcl::PointXYZ> (frame2.pc, tmp, pe.getTransform());
    frame2.pc.swap(tmp);
    // -----------------------------------------------------------------
    #endif

    // View the pe estimate...
    Eigen::Matrix3d rot = pe.rot;
    Eigen::Vector3d trans = pe.trans;
    Eigen::Affine3d transl_transform = (Eigen::Affine3d)Eigen::Translation3d(trans);
    Eigen::Affine3d rot_transform = (Eigen::Affine3d)Eigen::Matrix3d(rot);
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> transform = transl_transform*rot_transform;

    std::vector<std::pair<int,int> > corr = convertMatches(pe.inliers);
    NDTMatcherFeatureD2D<pcl::PointXYZ,pcl::PointXYZ> matcher_feat(corr);
    cout << "Matching ..." << endl;
    t1 = getDoubleTime();
    if (ident)
     transform.setIdentity();
    if (!skip_matching)
     matcher_feat.match(frame1.ndt_map, frame2.ndt_map, transform, true); //true = use initial guess
    t2 = getDoubleTime();
    cout << "Matching ... - done" << endl;
    std::cout << "ndt2ndt matching : " << t2 - t1 << std::endl;

    transformPointCloudInPlace(transform, frame2.pc);

    viewer.showFeaturePC();
    viewer.showMatches(pe.inliers);

    while (!viewer.wasStopped ())
    {
     viewer.spinOnce();
     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;
    */
}
