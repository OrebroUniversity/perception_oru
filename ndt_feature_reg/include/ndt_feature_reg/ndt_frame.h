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
#ifndef NDTFRAME_HH
#define NDTFRAME_HH

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include "pcl/registration/icp.h"
#include "pcl/filters/voxel_grid.h"
#include "opencv2/core/core.hpp"

#include <ndt_map/cell_vector.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/depth_camera.h>

#include <ndt_feature_reg/ndt_frame_tools.h>
#include <ndt_map/pointcloud_utils.h>

namespace ndt_feature_reg
{
inline
double getDoubleTime(struct timeval& time)
{
    return time.tv_sec + time.tv_usec * 1e-6;
}

inline
double getDoubleTime()
{
    struct timeval time;
    gettimeofday(&time,NULL);
    return getDoubleTime(time);
}

class NDTFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    NDTFrame() : supportSize(3),maxVar(0.3),current_res(0.2)
    {
        ndt_map = lslgeneric::NDTMap(&idx_prototype);
    }
    virtual ~NDTFrame()
    {
    }
    NDTFrame(const NDTFrame &other)
    {
        other.img.copyTo(img);
        other.depth_img.copyTo(depth_img);
        kpts = other.kpts;
        pc_kpts = other.pc_kpts;
        pts = other.pts;
        idx_prototype = other.idx_prototype;
        ndt_map = other.ndt_map;
        supportSize = other.supportSize;
        cameraParams = other.cameraParams;
        maxVar = other.maxVar;
        current_res=other.current_res;
        other.dtors.copyTo(dtors);
    }

    void clear()
    {
        maxVar = 0;
        kpts.resize(0);
        pc_kpts.resize(0);
        pts.resize(0);
        idx_prototype = lslgeneric::CellVector();
        ndt_map = lslgeneric::NDTMap(&idx_prototype);

        //pc.resize(0);
        //kpts_pc_indices.resize(0);
    }

    cv::Mat img;
    cv::Mat depth_img;
    size_t supportSize;
    double maxVar;
    double current_res;
    lslgeneric::DepthCamera<pcl::PointXYZ> cameraParams;
    std::vector<cv::KeyPoint> kpts;
    pcl::PointCloud<pcl::PointXYZ> pc_kpts; //cloud containing only keypoints
    /// \brief 3d points, linked to keypoints and SBA points for point-to-point matches. /pc_kpts in 4d vector
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > pts;

    lslgeneric::CellVector idx_prototype;
    lslgeneric::NDTMap ndt_map;

    const pcl::PointXYZ& getKeyPointCenter(int keyPointIdx)
    {
        //const std::vector<size_t> &indices = kpts_pc_indices[keyPointIdx];
        const pcl::PointXYZ &pt = pc_kpts[keyPointIdx]; // TODO - The first points is the centre only if the centre point is invalid...
        return pt;
    }

    void assignPts()
    {
        if(pc_kpts.size() == 0)
            computeNDT();
        pts.resize(pc_kpts.size());
        for (size_t i = 0; i < pc_kpts.size(); i++)
        {
            const pcl::PointXYZ& pt = pc_kpts[i];
            pts[i].head(3) = Eigen::Vector3d(pt.x,pt.y,pt.z);
            pts[i](3) = 1.0;
            //std::cout<<"keypoint "<<i<<" at "<<pts[i].transpose()<<std::endl;
        }
    }

    void computeNDT(bool estimateDI = false, bool nonMean = false)
    {
        if(kpts.size() > 0)
        {
            //double t1 = getDoubleTime();
            pc_kpts = ndt_map.loadDepthImageFeatures(depth_img,kpts,supportSize,maxVar,cameraParams,estimateDI,nonMean);
            //double t2 = getDoubleTime();
            //std::cout<<"computing ndt took (features)"<<t2-t1<<std::endl;
        }
        else
        {
            lslgeneric::LazyGrid idx_prototype_grid(current_res);
            ndt_map = lslgeneric::NDTMap(&idx_prototype_grid);
            //double t1 = getDoubleTime();
            ndt_map.loadDepthImage(depth_img,cameraParams);
            ndt_map.computeNDTCells();
            //double t2 = getDoubleTime();
            //std::cout<<"computing ndt took (grid)"<<t2-t1<<std::endl;

        }
        //ndt_map.computeNDTCells();
        //double t3 = getDoubleTime();

        /*
         double t1 = getDoubleTime();
         selectKeyPointIndices();
         double t2 = getDoubleTime();
         //ndt_map.loadPointCloud(pc, kpts_pc_indices);
         double t3 = getDoubleTime();
         ndt_map.computeNDTCells();
         double t4 = getDoubleTime();
         std::cout << "selectKeyPointIndices : " << t2 - t1 << std::endl;
         std::cout << "loadPointCloud : " << t3 - t2 << std::endl;
         std::cout << "computeNDTCells : " << t4 - t3 << std::endl;
         std::cout << "kpts_pc_indices.size() : " << kpts_pc_indices.size() << std::endl;
        */
    }
    /*
    void createKeyPointCloud()
    {
        pc_kpts.clear();
        size_t sz = 1;
        pcl::PointCloud<PointT> pc_kpts_loc;
        std::vector<cv::KeyPoint> good_kpts;
        //lslgeneric::CellVector<PointT> *cv = dynamic_cast<lslgeneric::CellVector<PointT> *>(ndt_map->getMyIndex());

        for (size_t i = 0; i < kpts.size(); i++)
        {
      cameraParams.computePointsAtIndex(depth_img,kpts[i],sz,pc_kpts_loc);
      PointT pt = pc_kpts_loc[0];
      if(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
        continue;

      good_kpts.push_back(kpts[i]);
      pc_kpts.push_back(pt);
        }
        kpts.clear();
        kpts.resize(0);
        kpts = good_kpts;
    } */

    pcl::PointCloud<pcl::PointXYZRGB> getColoredPointCloud()
    {
        lslgeneric::DepthCamera<pcl::PointXYZRGB> cameraParamsLocal (cameraParams.fx,cameraParams.fy,cameraParams.cx,
                cameraParams.cy,cameraParams.dist,cameraParams.ds, cameraParams.isFloatImg);
        cameraParamsLocal.setupDepthPointCloudLookUpTable(depth_img.size());

        pcl::PointCloud<pcl::PointXYZRGB> cloud; //(new pcl::PointCloud<pcl::PointXYZRGB>);
        cameraParamsLocal.convertDepthImageToPointCloud(depth_img,cloud);

        size_t w = img.size().width;
        size_t h = img.size().height;
        size_t idx = 0;
        cloud.width = w;
        cloud.height = h;
        cloud.is_dense = true;
        const uchar* pimg = img.ptr<uchar>(0);
        std::cout<<"img channels "<<img.channels()<<std::endl;
        pcl::PointXYZRGB color;
        //int r=0, g=0, b=0;
        uint8_t r = 0, g = 0, b = 0;

        for( size_t i =0; i<h; i++)
        {
            for( size_t j =0; j<w; j++)
            {
                if(img.channels() == 3)
                {
                    const cv::Vec3b& bgr = img.at<cv::Vec3b>((int)i,(int)j);
                    r = bgr[0];
                    g = bgr[1];
                    b = bgr[2];

                    idx = 3*(i * w + j);
                    /*			r = pimg[idx];
                    			g = pimg[idx+1];
                    			b = pimg[idx+2];*/
                }
                else if (img.channels() == 1)
                {
                    idx = (i * w + j);
                    r = pimg[idx];
                    g = pimg[idx];
                    b = pimg[idx];
                }

//		    std::cout<<r<<" "<<g<<" "<<b<<std::endl;
//		    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                pcl::PointXYZRGB &color = cloud.points[idx];
                //color.rgb = *reinterpret_cast<float*>(&rgb);
                color.r = r;
                color.g = g;
                color.b = b;
//		    std::cout<<color.r<<" "<<color.g<<" "<<color.b<<" "<<color.rgb<<std::endl;

//		    cloud.at(j,i) = color;
                /*
                		    color = getPCLColor(r,g,b);
                		    color.r = r;
                		    color.g = g;
                		    color.b = b;
                		    std::cout<<color.r<<" "<<color.g<<" "<<color.b<<" "<<color.rgb<<std::endl;
                		    cloud->at(j,i).rgb = color.rgb;
                		    cloud->at(j,i).r = r;
                		    cloud->at(j,i).g = g;
                		    cloud->at(j,i).b = b;
                */
            }
        }
        return cloud;
    }

    cv::Mat dtors;

    //int nbPoints;
    //pcl::PointCloud<PointT> pc;
    //std::vector<std::vector<size_t> > kpts_pc_indices; //indeces of the keypoints inside the point cloud - not needed
    // Only to use with an initial alignment (simply a sparser NDTMap with non-grid Gaussians based on the keypoints).
    // Only used to define the spatial structure of the ndt_map.

    /*
    	  void removeFiniteIndices(std::vector<size_t> &indices)
    	       {
    		    std::vector<size_t> tmp;
    		    for (size_t i = 0; i < indices.size(); i++)
    		    {
    			 const PointT& pt = pc[indices[i]];
    			 if (!std::isfinite(pt.x) ||
    			     !std::isfinite(pt.y) ||
    			     !std::isfinite(pt.z))
    			 {

    			 }
    			 else
    			 {
    			      tmp.push_back(indices[i]);
    			 }
    		    }
    		    indices = tmp;
    	       }

    	  void selectKeyPointIndices()
    	       {
    		    size_t nkpts = kpts.size();
    		    kpts_pc_indices.reserve(nkpts);
    		    std::vector<cv::KeyPoint> good_kpts;
    		    for (size_t i=0; i<nkpts; i++)
    		    {

    			 std::vector<size_t> indices;
    			 selectIndicesAroundKeyPoint(kpts[i], pc, nbPoints, indices);
    			 removeFiniteIndices(indices);

    			 // Compute the NDTCell here...
    			 lslgeneric::NDTCell<PointT> tmp;
    			 for (size_t j = 0; j < indices.size(); j++)
    			 {
    			      tmp.points_.push_back(pc[indices[j]]);
    			 }
    			 tmp.computeGaussian();
    			 if (tmp.hasGaussian_)
    			 {
    			      kpts_pc_indices.push_back(indices);
    			      good_kpts.push_back(kpts[i]);
    			      idx_prototype.addCell(tmp.copy());
    			 }
    			 else
    			 {
    //			      assert(false);
    			 }
    		    }
    		    kpts = good_kpts; // Probably a bit inefficient- but now we only have good ones in the vector and can have a one to one mapping without having to check for NaN's / having a separate boolean vector indicating if it is a good point etc.
    		    std::cout << "kpts.size() : " << kpts.size() << std::endl;
    		    std::cout << "idx_prototype.size() : " << idx_prototype.size() << std::endl;
    		    assert((int)kpts.size() == idx_prototype.size());
    	       }
    */
};


class PoseEstimator
{
public:
    PoseEstimator(int NRansac,
                  double maxidx, double maxidd);
    size_t estimate(const NDTFrame &f0, const NDTFrame &f1);
    size_t estimate(const NDTFrame &f0, const NDTFrame &f1, const std::vector<cv::DMatch>& matches);

    const std::vector<cv::DMatch>& getInliers() const
    {
        return inliers;
    }

    void matchFrames(const NDTFrame& f0, const NDTFrame& f2, std::vector<cv::DMatch>& fwd_matches);

    inline Eigen::Affine3f getTransform()
    {
        Eigen::Affine3f transl_transform = (Eigen::Affine3f)Eigen::Translation3f(trans[0], trans[1], trans[2]);
        Eigen::Affine3f rot_transform = (Eigen::Affine3f)Eigen::Matrix3f(rot.cast<float>());
        return transl_transform*rot_transform;
    }

    // all matches and inliers
    std::vector<cv::DMatch> matches; ///< Matches between features in frames.
    std::vector<cv::DMatch> inliers; ///< RANSAC inliers of matches.
    /// number of RANSAC iterations
    int numRansac;

    /// Whether to do windowed or whole-image matching.
    bool windowed;

    /// Maximum dist^2 for inliers in pixels.
    double maxInlierXDist2, maxInlierDDist2;

    double maxDist;
    double minDist;

    /// Descriptor matcher to use (features_2d::BruteForceMatcher by default).
    cv::Ptr<cv::DescriptorMatcher> matcher;
    int wx /*< Width of matching window.*/, wy /*< Height of matching window*/;

    // transform
    Eigen::Matrix3d rot; ///< Rotation matrix of camera between the frames.
    Eigen::Quaterniond quat;
    Eigen::Vector3d trans; ///< Translation of the camera between the frames.

    bool projectMatches;
};


} // namespace

//#include <ndt_feature_reg/impl/ndt_frame.hpp>

#endif
