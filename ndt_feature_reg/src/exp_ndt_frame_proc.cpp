#include <highgui.h>
#include <Eigen/Eigen>

namespace ndt_feature_reg
{

template <typename PointT>
void ExpNDTFrameProc<PointT>::addFrameIncremental (NDTFrame<PointT> *f, bool skipMatching, bool ndtEstimateDI,
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

    ExpNDTFrameProc<PointT>::EigenTransform tr, tr2;
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
        pe.estimate(*frames[0],*frames[i]);
        transl_transform = (Eigen::Affine3d)Eigen::Translation3d(pe.trans);
        rot_transform = (Eigen::Affine3d)Eigen::Matrix3d(pe.rot);
        tr = transl_transform*rot_transform;
	std::cout<<pe.inliers.size()<<std::endl;
        if (!skipMatching)
        {
            corr.clear();
            corr = convertMatches(pe.inliers);
            lslgeneric::NDTMatcherFeatureD2D<PointT,PointT> matcher_feat(corr, trim_factor);
            matcher_feat.match(frames[0]->ndt_map, frames[i]->ndt_map, tr, true); //true = use initial guess
            //std::cout<<tr.rotation()<<"\n"<<tr.translation().transpose()<<std::endl;
        }
    }
    else
    {
        lslgeneric::NDTMatcherD2D<PointT,PointT> matcher;
        matcher.current_resolution = frames[0]->current_res/2;
        matcher.match(frames[0]->ndt_map, frames[i]->ndt_map, tr);
        //std::cout<<tr.rotation()<<"\n"<<tr.translation().transpose()<<std::endl;
    }

    //tr2 = tr*transformVector.back().inverse();

    transformVector.push_back(tr);

}

template <typename PointT>
void ExpNDTFrameProc<PointT>::trimNbFrames (size_t maxNbFrames)
{
    if (frames.size() > maxNbFrames)
    {
        //frames.erase(frames.begin(), frames.begin() + frames.size() - maxNbFrames);
        for(size_t i =1; i<frames.size(); i++)
        {
            //std::cout<<"delete f : "<<i<<std::endl;
            delete frames[i];
        }
        frames.erase(frames.begin()+1, frames.end());
        transformVector.erase(transformVector.begin()+1, transformVector.end()); // Just to keep a 1-1 coorespondance with frames.
    }
}

template <typename PointT>
void
ExpNDTFrameProc<PointT>::detectKeypoints(NDTFrame<PointT> *f) const
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
ExpNDTFrameProc<PointT>::calcDescriptors(NDTFrame<PointT> *f) const
{
    if (img_scale == 1.)
    {
	//create image with copied out pixels
	double dsize = f->kpts.size();
	int N_DIMS = ceil(sqrt(dsize));
	//std::cout<<dsize<<" "<<N_DIMS<<std::endl;
	int support_size_max = 0;
	for (unsigned int q=0; q<f->kpts.size(); ++q) {
	    support_size_max = (f->kpts[q].size > support_size_max) ? f->kpts[q].size : support_size_max;
	}
	std::cout<<"support_size_max = "<<support_size_max<<std::endl;

	int support_size;
	cv::Mat projected (support_size_max*N_DIMS,support_size_max*N_DIMS,f->img.type());
	cv::Mat projected_drawn (support_size_max*N_DIMS,support_size_max*N_DIMS,f->img.type());
	cv::Mat orig_drawn (f->img.size(),f->img.type());
	int i, j, p, k;
	std::vector<cv::KeyPoint> new_kpts;
	cv::KeyPoint kp;
	cv::Size sz;


	for (unsigned int q=0; q<f->kpts.size(); ++q) {
	    support_size = (f->kpts[q].size > support_size_max) ? support_size_max : f->kpts[q].size;

	    i = support_size_max * (q / N_DIMS);
	    j = support_size_max * (q % N_DIMS);
	    cv::Mat roiMat (projected,cv::Rect(i,j,support_size,support_size));
	    p = f->kpts[q].pt.x - support_size/2;
	    k = f->kpts[q].pt.y - support_size/2;
	    cv::Mat roi2Mat(f->img,cv::Rect(p,k,support_size,support_size));
	    roi2Mat.copyTo(roiMat);
	    kp = f->kpts[q];
	    kp.pt.x = i+support_size/2;
	    kp.pt.y = j+support_size/2;
	    kp.size = support_size;
	    new_kpts.push_back(kp);
	}

	cv::drawKeypoints(projected,new_kpts,projected_drawn,cv::Scalar::all(-1),4);
	cv::namedWindow("projected",0);
	cv::imshow("projected", projected_drawn);
	
	cv::drawKeypoints(f->img,f->kpts,orig_drawn,cv::Scalar::all(-1),4);
	cv::namedWindow("original",0);
	cv::imshow("original", orig_drawn);

	cv::waitKey(0);

	cv::Mat temptors;
	std::cout<<"valid kpts: "<<f->kpts.size()<<" "<<new_kpts.size()<<std::endl;
        extractor->compute(f->img, f->kpts, f->dtors);
        extractor->compute(projected, new_kpts, temptors);
	std::cout<<"after kpts: "<<f->kpts.size()<<" "<<new_kpts.size()<<std::endl;
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

}
