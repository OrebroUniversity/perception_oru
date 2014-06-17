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
#ifndef NDTFRAMETOOLS_HH
#define NDTFRAMETOOLS_HH

//#include <ndt_feature_reg/NDTFrame.hh>
#include <pcl/point_types.h>

namespace ndt_feature_reg
{
template<class T> std::string toString (const T& x)
{
    std::ostringstream o;

    if (!(o << x))
        throw std::runtime_error ("::toString()");

    return o.str ();
}

inline void scaleKeyPointSize(std::vector<cv::KeyPoint> &keypoints, const float &factor)
{
    for( std::vector<cv::KeyPoint>::iterator i = keypoints.begin(), ie = keypoints.end(); i != ie; ++i )
    {
        i->size *= factor;
    }
}

inline void scaleKeyPointPosition(std::vector<cv::KeyPoint> &keypoints, const float &factor)
{
    for( std::vector<cv::KeyPoint>::iterator i = keypoints.begin(), ie = keypoints.end(); i != ie; ++i )
    {
        i->pt.x *= factor;
        i->pt.y *= factor;
    }
}

/*
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
	       ret.r = 255; ret.g = 0; ret.b = 0;
	       return ret;
	  case 1:
	       ret.r = 0; ret.g = 255; ret.b = 0;
	       return ret;
	  case 2:
	       ret.r = 0; ret.g = 0; ret.b = 255;
	       return ret;
	  default:
	       ret.r = 0; ret.g = 0; ret.b = 0;
	  }
	  return ret;
     }

     inline cv::Mat getCameraMatrix(double fx, double fy, double cx, double cy)
     {
	  cv::Mat ret = cv::Mat::zeros(3,3, CV_64F);
	  ret.at<double>(0,0) = fx;
	  ret.at<double>(1,1) = fy;
	  ret.at<double>(0,2) = cx;
	  ret.at<double>(1,2) = cy;
	  ret.at<double>(2,2) = 1.;
	  return ret;
     }

     inline cv::Mat getDistVector(double d0, double d1, double d2, double d3, double d4)
     {
	  cv::Mat ret = cv::Mat(5,1, CV_64F);
	  ret.at<double>(0) = d0;
	  ret.at<double>(1) = d1;
	  ret.at<double>(2) = d2;
	  ret.at<double>(3) = d3;
	  ret.at<double>(4) = d4;
	  return ret;
     }

     inline cv::Mat getDepthPointCloudLookUpTable(const cv::Size &size, const cv::Mat &camMat, const cv::Mat &distVec, const double &dsFactor)
     {
	  cv::Mat pixels = cv::Mat(size.height * size.width,1, CV_64FC2);
	  // Fill the tmp values simply with the image coordinates

	  {
	       cv::Mat_<cv::Vec2d> _I = pixels;
	       size_t iter = 0;
	       for (int y = 0; y < size.height; y++)
	       {
		    for (int x = 0; x < size.width; x++)
		    {
			 _I(iter)[0] = x;
			 _I(iter)[1] = y;
			 iter++;
		    }
	       }
	  }
	  cv::Mat normpixels = cv::Mat(pixels.size(), CV_64FC2); // normalized undistorted pixels
	  cv::undistortPoints(pixels, normpixels, camMat, distVec);

	  cv::Mat ret = cv::Mat(normpixels.size(), CV_64FC3); // "normpixelsxyz"
	  {
	       cv::Mat_<cv::Vec2d> _J = normpixels;
	       cv::Mat_<cv::Vec3d> _I = ret;
	       size_t iter = 0;
	       for (int y = 0; y < size.height; y++)
	       {
		    for (int x = 0; x < size.width; x++)
		    {
			 _I(iter)[0] = _J(iter)[0]*dsFactor;
			 _I(iter)[1] = _J(iter)[1]*dsFactor;
			 _I(iter)[2] = dsFactor;
			 iter++;
		    }
	       }
	  }
	  return ret;
     }

     inline void colorKeyPointsInPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pc, const std::vector<std::vector<size_t> > &kptsIndices, const pcl::PointXYZRGB &color)
     {
	  for (size_t i = 0; i < kptsIndices.size(); i++)
	  {
	       for (size_t j = 0; j < kptsIndices[i].size(); j++)
	       {
		    pcl::PointXYZRGB& pt = (pc)[kptsIndices[i][j]];
		    pt.rgb = color.rgb;
	       }
	  }
     }

     inline void createColoredPointCloud(const cv::Mat &img, const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZRGB> &out)
     {
	  // Assume 1 to 1 mapping...
	  size_t width = pc.width;
	  size_t height = pc.height;
	  size_t size = pc.size();
	  out.resize(size);
	  out.width = width;
	  out.height = height;
	  out.is_dense = pc.is_dense;

	  //	  cv::Mat_<cv::Vec3b> _I = img;
	  if (img.channels() == 1)
	  {
	       for (size_t i = 0; i < size; i++)
	       {
		    out[i].x = pc[i].x;
		    out[i].y = pc[i].y;
		    out[i].z = pc[i].z;
		    out[i].r = img.data[i];
		    out[i].g = img.data[i];
		    out[i].b = img.data[i];
	       }
	  }
	  if (img.channels() == 3)
	  {
	       for (size_t i = 0; i < size; i++)
	       {
		    out[i].x = pc[i].x;
		    out[i].y = pc[i].y;
		    out[i].z = pc[i].z;
		    out[i].r = img.data[3*i];
		    out[i].g = img.data[3*i+1];
		    out[i].b = img.data[3*i+2];
	       }
	  }
     }
*/

/*
     inline size_t convertDepthImageToPointCloud(const cv::Mat &depthImg, const cv::Mat &lookUpTable, pcl::PointCloud<pcl::PointXYZ> &pc)
     {
	  if (depthImg.depth() != CV_16U)
	       return 0;
	  size_t width = depthImg.size().width;
	  size_t height = depthImg.size().height;
	  size_t size = width*height;
	  if (pc.size() != size || pc.width != width || pc.height != height || pc.is_dense != true)
	  {
	       pc.resize(size);
	       pc.is_dense = true;
	       pc.width = width;
	       pc.height = height;
	  }

	  cv::Mat_<cv::Vec3d> _I = lookUpTable;

	  const double* plt = lookUpTable.ptr<double>(0);
	  const unsigned short* pd = depthImg.ptr<unsigned short>(0);
	  for (size_t i = 0; i < size; i++)
	  {
	       if (*pd == 0)
	       {
		    float nan = std::numeric_limits<float>::quiet_NaN();
		    pc[i] = pcl::PointXYZ(nan,nan,nan);
	       }
	       else
	       {
		    double depth = *pd;
		    pc[i] = pcl::PointXYZ(depth * _I(i)[0],
					  depth * _I(i)[1],
					  depth * _I(i)[2]);
	       }
	       pd++;
	       plt += 3;
	  }
	  return size;
     }


     inline bool insideBoarder(const cv::KeyPoint &keyPoint, const pcl::PointCloud<pcl::PointXYZ> &in, int boarderSize)
     {
	  int u = static_cast<int>(keyPoint.pt.x+0.5);
	  int v = static_cast<int>(keyPoint.pt.y+0.5);
	  if ((u >= boarderSize) &&
	      (u < (int)in.width-boarderSize) &&
	       (v >= boarderSize) && v <
	       ((int)in.height-boarderSize))
	       {
		    return true;
	       }
	      return false;

	      }

     inline void selectIndicesAroundKeyPoint(const cv::KeyPoint &keyPoint, const pcl::PointCloud<pcl::PointXYZ> &in, int nbPoints, std::vector<size_t> &out)
     {
	  int u = static_cast<int>(keyPoint.pt.x+0.5);
	  int v = static_cast<int>(keyPoint.pt.y+0.5);
	  int index = v * in.width + u;
	  out.push_back(index);
	  switch (nbPoints)
	  {
	  case 21:
	  {
	       if (insideBoarder(keyPoint, in, 3))
	       {
		    out.push_back(index-3);
		    out.push_back(index+3);
		    out.push_back(index+3*in.width);
		    out.push_back(index-3*in.width);
	       }
	  }
	  case 17:
	  {
	       if (insideBoarder(keyPoint, in, 2))
	       {
		    out.push_back(index-in.width-2);
		    out.push_back(index-in.width+2);
		    out.push_back(index+in.width-2);
		    out.push_back(index+in.width+2);
	       }
	  }
	  case 13:
	  {
	       if (insideBoarder(keyPoint, in, 2))
	       {
		    out.push_back(index-2);
		    out.push_back(index+2);
		    out.push_back(index+2*in.width);
		    out.push_back(index-2*in.width);
	       }
	  }
	  case 9:
	  {
	       if (insideBoarder(keyPoint, in, 1))
	       {
		    out.push_back(index-in.width-1);
		    out.push_back(index-in.width+1);
		    out.push_back(index+in.width-1);
		    out.push_back(index+in.width+1);
	       }
	  }
	  case 5:
	  {
	       if (insideBoarder(keyPoint, in, 1))
	       {
		    out.push_back(index-1);
		    out.push_back(index+1);
		    out.push_back(index+in.width);
		    out.push_back(index-in.width);
	       }
	  }
	  default:
	       // Nothing
	       break;
	  }
     }
*/
} // namespace

#endif
