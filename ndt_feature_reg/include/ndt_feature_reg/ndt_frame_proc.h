/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AASS Research Center nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef NDTFRAMEPROC_HH
#define NDTFRAMEPROC_HH

#include <ndt_feature_reg/ndt_frame.h>
#include <ndt_registration/ndt_matcher_d2d_feature.h>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

namespace ndt_feature_reg
{
class NDTFrameProc
{
public:
//	  bool loadImg(NDTFrame<PointT> &f, const std::string &fileName) const;
//	  void loadPC(NDTFrame<PointT> &f, const std::string &fileName) const;

//	  bool loadPCfromDepth(NDTFrame<PointT> &f, const std::string &fileName) const;

//	  bool setupDepthToPC(const std::string &fileName, double fx, double fy, double cx, double cy, const std::vector<double> &dist, double ds);
//	  void setupDepthToPC(const cv::Mat &depthImg, double fx, double fy, double cx, double cy, const std::vector<double> &dist, double ds);

//	  void convertDepthToPC(const cv::Mat &depthImg, pcl::PointCloud<PointT> &pc) const;

    void addFrame (NDTFrame *f)
    {
        frames.push_back(f);
    }

    void addFrameIncremental (NDTFrame *f, bool skipMatching, bool ndtEstimateDI = false,
                              bool match_full = false, bool match_no_association = false);
    void trimNbFrames (size_t maxNbFrames);

    void processFrames (bool skipMatching, bool ndtEstimateDI = false,
                        bool match_full = false, bool match_no_association = false);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr createColoredFeaturePC(NDTFrame &f, pcl::PointXYZRGB color);

    NDTFrameProc(int nb_ransac, double max_inldist_xy, double max_inldist_z): pe(nb_ransac, max_inldist_xy, max_inldist_z)
    {
        detector = cv::FeatureDetector::create("SURF");
        extractor = cv::DescriptorExtractor::create("SURF");
        img_scale = 1.;
        trim_factor = 1.;
        non_mean = false;
	cv::initModule_nonfree();
        cv::initModule_features2d();

    }

    PoseEstimator pe;
    typedef Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> EigenTransform;
    typename std::vector< NDTFrame*,Eigen::aligned_allocator<NDTFrame > > frames;
    typename std::vector<EigenTransform, Eigen::aligned_allocator<EigenTransform> > transformVector;

    inline void convertMatches(const std::vector<cv::DMatch> &in, std::vector<std::pair<int,int> > &out)
    {
        out.resize(in.size());
        for (size_t i = 0; i < in.size(); i++)
        {
            out[i].first = in[i].queryIdx;
            out[i].second = in[i].trainIdx;
        }
    }

    inline std::vector<std::pair<int,int> > convertMatches(const std::vector<cv::DMatch> &in)
    {
        std::vector<std::pair<int,int> > out;
        convertMatches(in,out);
        return out;
    }

    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> extractor;
    double img_scale;
    double trim_factor;
    bool non_mean;
    virtual ~NDTFrameProc()
    {
        for(size_t i =0; i<frames.size(); i++)
        {
            delete frames[i];
        }
        frames.clear();
    }
private:

    void detectKeypoints(NDTFrame *f) const;
    void calcDescriptors(NDTFrame *f) const;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//	  lslgeneric::NDTMatcherFeatureD2D<PointT,PointT> *matcher_feat; //(corr);


};



} // namespace

//#include <ndt_feature_reg/impl/ndt_frame_proc.hpp>
#endif
