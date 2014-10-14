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
#ifndef EXPNDTFRAMEPROC_HH
#define EXPNDTFRAMEPROC_HH

#include <ndt_feature_reg/ndt_frame.h>
#include <ndt_matcher_d2d_feature.h>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <ndt_feature_reg/ndt_frame_proc.h>

namespace ndt_feature_reg
{
class ExpNDTFrameProc : public ndt_feature_reg::NDTFrameProc
{
public:

    using NDTFrameProc::pe;
    using NDTFrameProc::detector;
    using NDTFrameProc::extractor;
    using NDTFrameProc::img_scale;
    using NDTFrameProc::trim_factor;
    using NDTFrameProc::non_mean;
    using NDTFrameProc::frames;
    using NDTFrameProc::transformVector;
    
    typedef Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> EigenTransform;
   

    void trimNbFrames (size_t maxNbFrames);
    void addFrameIncremental (NDTFrame *f, bool skipMatching, bool ndtEstimateDI = false,
                              bool match_full = false, bool match_no_association = false);
    
    ExpNDTFrameProc(int nb_ransac, double max_inldist_xy, double max_inldist_z):NDTFrameProc(nb_ransac,max_inldist_xy,max_inldist_z) 
    {
    }
    
    virtual ~ExpNDTFrameProc()
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

};



} // namespace

#include <ndt_feature_reg/impl/exp_ndt_frame_proc.hpp>
#endif
