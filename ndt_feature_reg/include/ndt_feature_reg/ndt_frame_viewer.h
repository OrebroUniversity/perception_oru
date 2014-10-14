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
#ifndef NDTFRAMEVIEWER_HH
#define NDTFRAMEVIEWER_HH

#include <ndt_feature_reg/ndt_frame.h>
#include <ndt_feature_reg/ndt_frame_proc.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace ndt_feature_reg
{
inline void viewKeypointMatches(NDTFrameProc *proc, int delay)
{
    if (proc->frames.size() < 2)
        return;
    cv::Mat display;
    int i = proc->frames.size()-1;
    cv::drawMatches(proc->frames[i-1]->img, proc->frames[i-1]->kpts, proc->frames[i]->img, proc->frames[i]->kpts, proc->pe.inliers, display);
    const std::string window_name = "matches";
    cv::namedWindow(window_name,0);
    cv::imshow(window_name, display);
    cv::waitKey(delay);
}
inline void viewKeypointMatchesFirst(NDTFrameProc *proc, int delay)
{
    if (proc->frames.size() < 2)
        return;
    cv::Mat display;
    int i = proc->frames.size()-1;
    cv::drawMatches(proc->frames[0]->img, proc->frames[0]->kpts, proc->frames[i]->img, proc->frames[i]->kpts, proc->pe.inliers, display);
    const std::string window_name = "matches";
    cv::namedWindow(window_name,0);
    cv::imshow(window_name, display);
    cv::waitKey(delay);
}

class NDTFrameViewer
{
public:
    NDTFrameViewer(NDTFrameProc *proc);
    void showPC();
    void showFeaturePC();
    void showNDT();
    void showMatches(const std::vector<cv::DMatch> &matches);
    void showMatches(const std::vector<std::pair<int,int> > &matches);
    boost::shared_ptr<pcl::visualization::PCLVisualizer>& getViewerPtr()
    {
        return _viewer;
    }
    bool wasStopped();
    void spinOnce();
private:
    void initViewer();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;
    NDTFrameProc *_proc;

    inline pcl::PointXYZRGB getPCLColor(int r, int g, int b)
    {
        pcl::PointXYZRGB ret;
        ret.r = r;
        ret.g = g;
        ret.b = b;
        return ret;
    }

    inline pcl::PointXYZRGB getPCLColor(size_t i)
    {
        pcl::PointXYZRGB ret;

        switch (i)
        {
        case 0:
            ret.r = 255;
            ret.g = 0;
            ret.b = 0;
            return ret;
        case 1:
            ret.r = 0;
            ret.g = 255;
            ret.b = 0;
            return ret;
        case 2:
            ret.r = 0;
            ret.g = 0;
            ret.b = 255;
            return ret;
        default:
            ret.r = 0;
            ret.g = 0;
            ret.b = 0;
        }
        return ret;
    }

};
}
//#include <ndt_feature_reg/impl/ndt_frame_viewer.hpp>

#endif
