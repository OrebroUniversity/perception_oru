#include <highgui.h>
#include <Eigen/Eigen>

namespace ndt_feature_reg
{

template <typename PointT>
void NDTFrameProc<PointT>::addFrameIncremental (NDTFrame<PointT> *f, bool skipMatching, bool ndtEstimateDI,
        bool match_full, bool match_no_association)
{

    if(!match_full)
    {
        detectKeypoints(f);
    }

    f->computeNDT(ndtEstimateDI, this->non_mean);

    if(!(match_no_association||match_full))
    {
        f->assignPts();
        calcDescriptors(f);
    }
    frames.push_back(f);

    NDTFrameProc<PointT>::EigenTransform tr;
    tr.setIdentity();
    if(frames.size() == 1)
    {

        transformVector.clear();
        transformVector.push_back(tr);
        return;
    }


    Eigen::Affine3d transl_transform;
    Eigen::Affine3d rot_transform;
    std::vector<std::pair<int,int> > corr;

    int i = frames.size()-1;

    if(!(match_full || match_no_association))
    {
        pe.estimate(*frames[i-1],*frames[i]);
        transl_transform = (Eigen::Affine3d)Eigen::Translation3d(pe.trans);
        rot_transform = (Eigen::Affine3d)Eigen::Matrix3d(pe.rot);
        tr = transl_transform*rot_transform;
        if (!skipMatching)
        {
            corr.clear();
            corr = convertMatches(pe.inliers);
            lslgeneric::NDTMatcherFeatureD2D<PointT,PointT> matcher_feat(corr, trim_factor);
            matcher_feat.match(frames[i-1]->ndt_map, frames[i]->ndt_map, tr, true); //true = use initial guess
            //std::cout<<tr.rotation()<<"\n"<<tr.translation().transpose()<<std::endl;
        }
    }
    else
    {
        lslgeneric::NDTMatcherD2D<PointT,PointT> matcher;
        matcher.current_resolution = frames[i-1]->current_res/2;
        matcher.match(frames[i-1]->ndt_map, frames[i]->ndt_map, tr);
        //std::cout<<tr.rotation()<<"\n"<<tr.translation().transpose()<<std::endl;
    }


    transformVector.push_back(tr);

}

template <typename PointT>
void NDTFrameProc<PointT>::trimNbFrames (size_t maxNbFrames)
{
    if (frames.size() > maxNbFrames)
    {
        //frames.erase(frames.begin(), frames.begin() + frames.size() - maxNbFrames);
        for(size_t i =0; i<frames.size()-1; i++)
        {
            //std::cout<<"delete f : "<<i<<std::endl;
            delete frames[i];
        }
        frames.erase(frames.begin(), frames.end() - 1);
        transformVector.erase(transformVector.begin(), transformVector.end() - 1); // Just to keep a 1-1 coorespondance with frames.
    }
}

template <typename PointT>
void NDTFrameProc<PointT>::processFrames (bool skipMatching, bool ndtEstimateDI,
        bool match_full, bool match_no_association)
{
    if(frames.size() < 1) return;

    transformVector.clear();
    NDTFrameProc<PointT>::EigenTransform tr;
    tr.setIdentity();
    transformVector.push_back(tr);

    if(!match_full)
    {
        detectKeypoints(frames[0]);
    }
    frames[0]->computeNDT(ndtEstimateDI, this->non_mean);

    if(!(match_no_association||match_full))
    {
        frames[0]->assignPts();
        calcDescriptors(frames[0]);
    }

    Eigen::Affine3d transl_transform;
    Eigen::Affine3d rot_transform;
    std::vector<std::pair<int,int> > corr;

    for(size_t i=1; i<frames.size(); i++)
    {

        if(!match_full)
        {
            detectKeypoints(frames[i]);
        }
        frames[i]->computeNDT(ndtEstimateDI, this->non_mean);
        if(!(match_no_association||match_full))
        {
            frames[i]->assignPts();
            calcDescriptors(frames[i]);

            pe.estimate(*frames[i-1],*frames[i]);
            transl_transform = (Eigen::Affine3d)Eigen::Translation3d(pe.trans);
            rot_transform = (Eigen::Affine3d)Eigen::Matrix3d(pe.rot);
            tr = transl_transform*rot_transform;

            if (!skipMatching)
            {
                corr.clear();
                corr = convertMatches(pe.inliers);
                lslgeneric::NDTMatcherFeatureD2D<PointT,PointT> matcher_feat(corr, trim_factor);
                matcher_feat.match(frames[i-1]->ndt_map, frames[i]->ndt_map, tr, true); //true = use initial guess
                //std::cout<<tr.rotation()<<"\n"<<tr.translation().transpose()<<std::endl;
            }
        }
        else
        {
            lslgeneric::NDTMatcherD2D<PointT,PointT> matcher;
            matcher.current_resolution = frames[i-1]->current_res/2;
            matcher.match(frames[i-1]->ndt_map, frames[i]->ndt_map, tr);
            //std::cout<<tr.rotation()<<"\n"<<tr.translation().transpose()<<std::endl;
        }

        /*
        	transformVector.push_back(tr);
        	cv::Mat display;
        	drawMatches(frames[i-1]->img, frames[i-1]->kpts, frames[i]->img, frames[i]->kpts, pe.inliers, display);
        	const std::string window_name = "matches";
        	cv::namedWindow(window_name,0);
        	cv::imshow(window_name, display);
        	cv::waitKey(0);
        */

    }

}

template <typename PointT>
void
NDTFrameProc<PointT>::detectKeypoints(NDTFrame<PointT> *f) const
{
    if (img_scale == 1.)
    {
        detector->detect(f->img, f->kpts);
    }
    else
    {
        cv::Mat tmp;
        cv::resize(f->img, tmp, cv::Size(), img_scale, img_scale, cv::INTER_LINEAR);
        detector->detect(tmp, f->kpts);
        float factor = 1./img_scale;
        scaleKeyPointPosition(f->kpts, factor);
    }
}

template <typename PointT>
void
NDTFrameProc<PointT>::calcDescriptors(NDTFrame<PointT> *f) const
{
    if (img_scale == 1.)
    {
        extractor->compute(f->img, f->kpts, f->dtors);
    }
    else
    {
        cv::Mat tmp;
        cv::resize(f->img, tmp, cv::Size(), img_scale, img_scale, cv::INTER_LINEAR);
        float factor = img_scale;
        scaleKeyPointPosition(f->kpts, factor);
        extractor->compute(tmp, f->kpts, f->dtors);
        factor = 1./img_scale;
        scaleKeyPointPosition(f->kpts, factor);
    }
}

template <typename PointT>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
NDTFrameProc<PointT>::createColoredFeaturePC(NDTFrame<PointT> &f, pcl::PointXYZRGB color)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ret(new pcl::PointCloud<pcl::PointXYZRGB>());
    *ret = f.getColoredPointCloud();
    size_t w = f.img.size().width;
    //size_t h = f.img.size().height;
    size_t idx = 0;

    for (size_t i = 0; i < f.kpts.size(); i++)
    {
        int uKey = static_cast<int>(f.kpts[i].pt.x+0.5);
        int vKey = static_cast<int>(f.kpts[i].pt.y+0.5);

        idx = vKey * w + uKey;
        pcl::PointXYZRGB& pt = ret->points[idx];
        pt.rgb = color.rgb;
    }
    return ret;
}

/*
template <typename PointT>
bool
NDTFrameProc<PointT>::loadImg(NDTFrame<PointT> &f, const std::string &fileName) const
{
     f.img = cv::imread(fileName, 0);
     if (f.img.empty())
     {
	 std::cerr << "Failed to load : " << fileName << std::endl;
	  return false;
     }
     return true;
}

template <typename PointT>
void
NDTFrameProc<PointT>::loadPC(NDTFrame<PointT> &f, const std::string &fileName) const
{

}

template <typename PointT>
bool
NDTFrameProc<PointT>::loadPCfromDepth(NDTFrame<PointT> &f, const std::string &fileName) const
{
     // Load the file.
     f.depth_img = cv::imread(fileName, CV_LOAD_IMAGE_ANYDEPTH); // CV_LOAD_IMAGE_ANYDEPTH is important to load the 16bits image
     if (f.depth_img.empty())
     {
	  std::cerr << "Failed to load : " << fileName << std::endl;
     }
     if (!this->_lookupTable.empty())
     {
	  //convertDepthToPC(f.depth_img, f.pc);
	  return true;
     }
     return false;
}

template <typename PointT>
bool
NDTFrameProc<PointT>::setupDepthToPC(const std::string &fileName, double fx, double fy, double cx, double cy, const std::vector<double> &dist, double ds)
{
     cv::Mat depth_img = cv::imread(fileName, CV_LOAD_IMAGE_ANYDEPTH);
     if (depth_img.empty())
     {
	  std::cerr << "Failed to load : " << fileName << std::endl;
	  return false;
     }
     setupDepthToPC(depth_img, fx, fy, cx, cy, dist, ds);
     return true;
}

template <typename PointT>
void
NDTFrameProc<PointT>::setupDepthToPC(const cv::Mat &depthImg, double fx, double fy, double cx, double cy, const std::vector<double> &dist, double ds)
{
     _camMat = getCameraMatrix(fx, fy, cx, cy);
     _dist = getDistVector(dist[0], dist[1], dist[2], dist[3], dist[4]);

     _lookupTable = getDepthPointCloudLookUpTable(depthImg.size(), _camMat, _dist, ds);
}

template <typename PointT>
void
NDTFrameProc<PointT>::convertDepthToPC(const cv::Mat &depthImg, pcl::PointCloud<PointT> &pc) const
{
     convertDepthImageToPointCloud(depthImg, _lookupTable, pc);
}
*/
}
