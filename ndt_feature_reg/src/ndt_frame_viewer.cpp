#include <ndt_map/pointcloud_utils.h>
#include <ndt_feature_reg/ndt_frame_viewer.h>

namespace ndt_feature_reg
{

NDTFrameViewer::NDTFrameViewer(NDTFrameProc *proc)
{
    //_frames.push_back(f);
    _proc = proc;
    initViewer();
}
/*
template <typename PointT>
NDTFrameViewer<PointT>::NDTFrameViewer(NDTFrame<PointT> *f0, NDTFrame<PointT> *f1, NDTFrameProc<PointT> &proc) : _proc(proc)
{
     _frames.push_back(f0);
     _frames.push_back(f1);
     initViewer();
} */

void
NDTFrameViewer::initViewer()
{
    _viewer.reset (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    _viewer->setBackgroundColor( 0, 0, 0.01 );
    _viewer->addCoordinateSystem(1.0);
}

/*
template <typename PointT>
void
NDTFrameViewer<PointT>::showPC()
{
     for (size_t i = 0; i < _proc.frames.size(); i++)
     {
	  std::string name = "cloud" + toString(i);
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp;
	  *tmp = _proc.frames[i].get
	  = _proc.createColoredFeaturePC(*(_frames[i]), getPCLColor(i));
	  _viewer->addPointCloud<PointT> (pc.makeShared(), name);
     }
}*/

void
NDTFrameViewer::showFeaturePC()
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr big_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    std::string name = "featcloud"; // + toString(i);
    for (size_t i = 0; i < _proc->frames.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = _proc->createColoredFeaturePC(*_proc->frames[i], this->getPCLColor(i));
        lslgeneric::transformPointCloudInPlace(_proc->transformVector[i],*tmp);

        //lslgeneric::writeToVRML(name.c_str(),*tmp);
        // TODO -> check why this is needed here!?!
        //pcl::io::savePCDFile ("tmp.pcd", *tmp);
        //pcl::io::loadPCDFile ("tmp.pcd", *tmp);

        *big_cloud += *tmp;
    }
    big_cloud->is_dense = false;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(big_cloud);
    _viewer->addPointCloud<pcl::PointXYZRGB> (big_cloud, rgb, name);
    //FIXME
    //lslgeneric::writeToVRML(name.c_str(),*big_cloud);
}

void
NDTFrameViewer::showNDT()
{

}

void
NDTFrameViewer::showMatches(const std::vector<std::pair<int,int> > &matches)
{
    assert(_proc->frames.size() == 2);

    for (size_t i = 0; i < matches.size(); i++)
    {
        const pcl::PointXYZ& pt1 = _proc->frames[0]->getKeyPointCenter(matches[i].first);
        const pcl::PointXYZ& pt2 = _proc->frames[1]->getKeyPointCenter(matches[i].second);

        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T = _proc->transformVector[1];
        Eigen::Vector3d pt;
        pt<<pt2.x,pt2.y,pt2.z;
        pt = T*pt;
        pcl::PointXYZ pt3;
        pt3.x = pt(0);
        pt3.y = pt(1);
        pt3.z = pt(2);

        std::string name = "line" + toString(i);
        _viewer->addLine(pt1, pt3, 0, 1, 1, name);
    }
}

void
NDTFrameViewer::showMatches(const std::vector<cv::DMatch> &matches)
{
    std::vector<std::pair<int,int> > tmp;
    _proc->convertMatches(matches, tmp);
    this->showMatches(tmp);
}

bool
NDTFrameViewer::wasStopped()
{
    return _viewer->wasStopped();
}

void
NDTFrameViewer::spinOnce()
{
    _viewer->spinOnce(100);
}

}
