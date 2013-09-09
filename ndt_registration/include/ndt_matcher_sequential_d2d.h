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
#ifndef NDT_MATCHER_SEQUENTIAL_D2D_HH
#define NDT_MATCHER_SEQUENTIAL_D2D_HH

#include "ndt_map.h"
#include "pcl/point_cloud.h"
#include "Eigen/Core"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

namespace lslgeneric
{
struct TransformParams {
    std::vector<Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>, 
		Eigen::aligned_allocator<Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> > > fk, rg;
};

/**
 * This class implements NDT registration of a sequence of 3D point clouds to a reference system.
 */
template <typename PointSource>
class NDTMatcherSequentialD2D
{
public:
    NDTMatcherSequentialD2D(double _resolution)
    {
	current_resolution = _resolution;
	lfd1 = 1;
	lfd2 = 0.05;
	ITR_MAX = 500;
    }
    NDTMatcherSequentialD2D()
    {
	current_resolution = 0.5;
	lfd1 = 1;
	lfd2 = 0.05;
	ITR_MAX = 500;
    }
    NDTMatcherSequentialD2D(const NDTMatcherSequentialD2D& other)
    {
	current_resolution = other.current_resolution;
	lfd1 = 1;
	lfd2 = 0.05;
	ITR_MAX = 500;
    }

    bool add_cloud( pcl::PointCloud<PointSource>& cloud,
		    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& Tref);
    bool match_all(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T_correction, bool useInitialGuess=false);

    //compute the score gradient & hessian of a point cloud + transformation to an NDT
    //input: moving, fixed, tr, computeHessian
    //output: score_gradient, Hessian, returns: score!
    virtual double derivativesNDT(
        const std::vector<NDTCell<PointSource>*> &source,
        const std::vector<NDTCell<PointSource>*> &target,
        Eigen::MatrixXd &score_gradient,
        Eigen::MatrixXd &Hessian,
        bool computeHessian
    );
    
    double current_resolution;
    int ITR_MAX;

    std::vector<Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>, 
		Eigen::aligned_allocator<Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> > > transforms;
    std::vector<lslgeneric::NDTMap<PointSource>*, 
		Eigen::aligned_allocator<lslgeneric::NDTMap<PointSource>* > > maps;
    std::vector<pcl::PointCloud<PointSource>, 
		Eigen::aligned_allocator<pcl::PointCloud<PointSource>* > > pcs;
protected:

    double lfd1,lfd2;
    int iteration_counter_internal;


    //iteratively update the score gradient and hessian
    virtual bool update_gradient_hessian_local(
        Eigen::MatrixXd &score_gradient,
        Eigen::MatrixXd &Hessian,
        const Eigen::Vector3d &m1,
        const Eigen::Matrix3d &C1,
        const double &likelihood,
        const Eigen::Matrix<double,3,6> &_Jest,
        const Eigen::Matrix<double,18,6> &_Hest,
        const Eigen::Matrix<double,3,18> &_Zest,
        const Eigen::Matrix<double,18,18> &_ZHest,
        bool computeHessian);

    //pre-computes the derivative matrices Jest, Hest, Zest, ZHest
    void computeDerivativesLocal(Eigen::Vector3d &m1, Eigen::Matrix3d C1,
                                 Eigen::Matrix<double,3,6> &_Jest,
                                 Eigen::Matrix<double,18,6> &_Hest,
                                 Eigen::Matrix<double,3,18> &_Zest,
                                 Eigen::Matrix<double,18,18> &_ZHest,
                                 bool computeHessian);

    //perform line search to find the best descent rate (Mohre&Thuente)
    //adapted from NOX
    double lineSearchMT(
        Eigen::Matrix<double,6,1> &increment,
        std::vector<NDTCell<PointSource>*> &source,
        std::vector<NDTCell<PointSource>*> &target);

    //auxiliary functions for MoreThuente line search
    struct MoreThuente
    {
        static double min(double a, double b);
        static double max(double a, double b);
        static double absmax(double a, double b, double c);
        static int cstep(double& stx, double& fx, double& dx,
                         double& sty, double& fy, double& dy,
                         double& stp, double& fp, double& dp,
                         bool& brackt, double stmin, double stmax);
    }; //end MoreThuente

    double normalizeAngle(double a);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // end namespace

#include <impl/ndt_matcher_sequential_d2d.hpp>
#endif
