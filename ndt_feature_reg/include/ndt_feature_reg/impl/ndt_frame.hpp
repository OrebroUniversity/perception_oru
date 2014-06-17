//#include <sba/sba.h>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <iostream>
#include <limits>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace ndt_feature_reg
{

template <typename PointSource, typename PointTarget>
PoseEstimator<PointSource,PointTarget>::PoseEstimator(int NRansac,
        double maxidx, double maxidd)
{
    numRansac = NRansac;
    maxInlierXDist2 = maxidx*maxidx;
    maxInlierDDist2 = maxidd*maxidd;
    rot.setIdentity();
    trans.setZero();

    // matcher
    matcher = new cv::BFMatcher(cv::NORM_L2);
    wx = 92;
    wy = 48;
    windowed = false;
    projectMatches = true;

    maxDist = std::numeric_limits<double>::max();
    minDist = -1.;
}

template <typename PointSource, typename PointTarget>
void
PoseEstimator<PointSource,PointTarget>::matchFrames(const NDTFrame<PointSource>& f0, const NDTFrame<PointTarget>& f1, std::vector<cv::DMatch>& fwd_matches)
{
    cv::Mat mask;
    if (windowed)
        mask = cv::windowedMatchingMask(f0.kpts, f1.kpts, wx, wy);

    //require at least 3 kpts each
    if(f0.kpts.size() > 3 && f1.kpts.size() > 3) 
    {
	matcher->match(f0.dtors, f1.dtors, fwd_matches, mask);
    }
}


template <typename PointSource, typename PointTarget>
size_t PoseEstimator<PointSource,PointTarget>::estimate(const NDTFrame<PointSource> &f0, const NDTFrame<PointTarget> &f1)
{
    // set up match lists
    matches.clear();
    inliers.clear();

//     std::cout<<"N KPTS: "<<f0.kpts.size()<<" "<<f1.kpts.size()<<std::endl;
//     std::cout<<"descriptors: "<<f0.dtors.size().width<<"x"<<f0.dtors.size().height<<" "
//			       <<f1.dtors.size().width<<"x"<<f1.dtors.size().height<<std::endl;
    // do forward and reverse matches
    std::vector<cv::DMatch> fwd_matches, rev_matches;
    matchFrames(f0, f1, fwd_matches);
    matchFrames(f1, f0, rev_matches);
//     printf("**** Forward matches: %d, reverse matches: %d ****\n", (int)fwd_matches.size(), (int)rev_matches.size());

    // combine unique matches into one list
    for (int i = 0; i < (int)fwd_matches.size(); ++i)
    {
        if (fwd_matches[i].trainIdx >= 0)
            matches.push_back( cv::DMatch(i, fwd_matches[i].trainIdx, fwd_matches[i].distance) );
        //matches.push_back( cv::DMatch(fwd_matches[i].queryIdx, fwd_matches[i].trainIdx, fwd_matches[i].distance) );
    }
    for (int i = 0; i < (int)rev_matches.size(); ++i)
    {
        if (rev_matches[i].trainIdx >= 0 && i != fwd_matches[rev_matches[i].trainIdx].trainIdx)
            matches.push_back( cv::DMatch(rev_matches[i].trainIdx, i, rev_matches[i].distance) );
        //matches.push_back( cv::DMatch(rev_matches[i].trainIdx, rev_matches[i].queryIdx, rev_matches[i].distance) );
    }
//     printf("**** Total unique matches: %d ****\n", (int)matches.size());

    // do it
    return estimate(f0, f1, matches);
}


template <typename PointSource, typename PointTarget>
size_t PoseEstimator<PointSource,PointTarget>::estimate(const NDTFrame<PointSource>& f0, const NDTFrame<PointTarget>& f1,
        const std::vector<cv::DMatch> &matches)
{
    // convert keypoints in match to 3d points
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > p0; // homogeneous coordinates
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > p1;

    int nmatch = matches.size();
    //srand(getDoubleTime());

    // set up data structures for fast processing
    // indices to good matches
    std::vector<int> m0, m1;
    for (int i=0; i<nmatch; i++)
    {
        m0.push_back(matches[i].queryIdx);
        m1.push_back(matches[i].trainIdx);
        //std::cout<<m0[i]<<" "<<m1[i]<<std::endl;
    }

    nmatch = m0.size();

    if (nmatch < 3) return 0;   // can't do it...

    int bestinl = 0;

    // RANSAC loop
//#pragma omp parallel for shared( bestinl )
    for (int i=0; i<numRansac; i++)
    {
        //std::cout << "ransac loop : " << i << std::endl;
        // find a candidate
        int a=rand()%nmatch;
        int b = a;
        while (a==b)
            b=rand()%nmatch;
        int c = a;
        while (a==c || b==c)
            c=rand()%nmatch;

        int i0a = m0[a];
        int i0b = m0[b];
        int i0c = m0[c];
        int i1a = m1[a];
        int i1b = m1[b];
        int i1c = m1[c];

        if (i0a == i0b || i0a == i0c || i0b == i0c ||
                i1a == i1b || i1a == i1c || i1b == i1c)
            continue;

        //std::cout<<a<<" "<<b<<" "<<c<<std::endl;
        //std::cout<<i0a<<" "<<i0b<<" "<<i0c<<std::endl;
        //std::cout<<i1a<<" "<<i1b<<" "<<i1c<<std::endl;

        // get centroids
        Eigen::Vector3d p0a = f0.pts[i0a].head(3);
        Eigen::Vector3d p0b = f0.pts[i0b].head(3);
        Eigen::Vector3d p0c = f0.pts[i0c].head(3);
        Eigen::Vector3d p1a = f1.pts[i1a].head(3);
        Eigen::Vector3d p1b = f1.pts[i1b].head(3);
        Eigen::Vector3d p1c = f1.pts[i1c].head(3);

        Eigen::Vector3d c0 = (p0a+p0b+p0c)*(1.0/3.0);
        Eigen::Vector3d c1 = (p1a+p1b+p1c)*(1.0/3.0);

        //std::cout<<c0.transpose()<<std::endl;
        //std::cout<<c1.transpose()<<std::endl;
        // subtract out
        p0a -= c0;
        p0b -= c0;
        p0c -= c0;
        p1a -= c1;
        p1b -= c1;
        p1c -= c1;

        Eigen::Matrix3d H = p1a*p0a.transpose() + p1b*p0b.transpose() +
                            p1c*p0c.transpose();

        // do the SVD thang
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d V = svd.matrixV();
        Eigen::Matrix3d R = V * svd.matrixU().transpose();
        double det = R.determinant();
        //ntot++;
        if (det < 0.0)
        {
            //nneg++;
            V.col(2) = V.col(2)*-1.0;
            R = V * svd.matrixU().transpose();
        }
        Eigen::Vector3d tr = c0-R*c1;    // translation

        // transformation matrix, 3x4
        Eigen::Matrix<double,3,4> tfm;
        //        tfm.block<3,3>(0,0) = R.transpose();
        //        tfm.col(3) = -R.transpose()*tr;
        tfm.block<3,3>(0,0) = R;
        tfm.col(3) = tr;

#if 0
        // find inliers, based on image reprojection
        int inl = 0;
        for (int i=0; i<nmatch; i++)
        {
            Vector3d pt = tfm*f1.pts[m1[i]];
            Vector3d ipt = f0.cam2pix(pt);
            const cv::KeyPoint &kp = f0.kpts[m0[i]];
            double dx = kp.pt.x - ipt.x();
            double dy = kp.pt.y - ipt.y();
            double dd = f0.disps[m0[i]] - ipt.z();
            if (dx*dx < maxInlierXDist2 && dy*dy < maxInlierXDist2 &&
                    dd*dd < maxInlierDDist2)
            {
                inl+=(int)sqrt(ipt.z()); // clever way to weight closer points
//		 inl+=(int)sqrt(ipt.z()/matches[i].distance);
//		 cout << "matches[i].distance : " << matches[i].distance << endl;
//		 inl++;
            }
        }
#endif
        int inl = 0;
        for (int i=0; i<nmatch; i++)
        {
            Eigen::Vector3d pt1 = tfm*f1.pts[m1[i]];
            Eigen::Vector3d pt0 = f0.pts[m0[i]].head(3);

//	       double z = fabs(pt1.z() - pt0.z())*0.5;
            double z = pt1.z();
            double dx = pt1.x() - pt0.x();
            double dy = pt1.y() - pt0.y();
            double dd = pt1.z() - pt0.z();

            if (projectMatches)
            {
                // The central idea here is to divide by the distance (this is essentially what cam2pix does).
                dx = dx / z;
                dy = dy / z;
            }
            if (dx*dx < maxInlierXDist2 && dy*dy < maxInlierXDist2 &&
                    dd*dd < maxInlierDDist2)
            {
//----		    inl+=(int)sqrt(pt0.z()); // clever way to weight closer points
//		 inl+=(int)sqrt(ipt.z()/matches[i].distance);
//		 cout << "matches[i].distance : " << matches[i].distance << endl;
                inl++;
            }
        }

//#pragma omp critical
        if (inl > bestinl)
        {
            bestinl = inl;
            rot = R;
            trans = tr;
//	       std::cout << "bestinl : " << bestinl << std::endl;
        }
    }

    //printf("Best inliers: %d\n", bestinl);
    //printf("Total ransac: %d  Neg det: %d\n", ntot, nneg);

    // reduce matches to inliers
    std::vector<cv::DMatch> inls;    // temporary for current inliers
    inliers.clear();
    Eigen::Matrix<double,3,4> tfm;
    tfm.block<3,3>(0,0) = rot;
    tfm.col(3) = trans;

    //std::cout<<"f0: "<<f0.pts.size()<<" "<<f0.kpts.size()<<" "<<f0.pc_kpts.size()<<std::endl;
    //std::cout<<"f1: "<<f1.pts.size()<<" "<<f1.kpts.size()<<" "<<f1.pc_kpts.size()<<std::endl;

    nmatch = matches.size();
    for (int i=0; i<nmatch; i++)
    {
        Eigen::Vector3d pt1 = tfm*f1.pts[matches[i].trainIdx];
        //Eigen::Vector3d pt1_unchanged = f1.pts[matches[i].trainIdx].head(3);
        //Vector3d pt1 = pt1_unchanged;
#if 0
        Vector3d ipt = f0.cam2pix(pt);
        const cv::KeyPoint &kp = f0.kpts[matches[i].queryIdx];
        double dx = kp.pt.x - ipt.x();
        double dy = kp.pt.y - ipt.y();
        double dd = f0.disps[matches[i].queryIdx] - ipt.z();
#endif
        Eigen::Vector3d pt0 = f0.pts[matches[i].queryIdx].head(3);

        //double z = fabs(pt1.z() - pt0.z())*0.5;
        double z = pt1.z();
        double dx = pt1.x() - pt0.x();
        double dy = pt1.y() - pt0.y();
        double dd = pt1.z() - pt0.z();

        if (projectMatches)
        {
            // The central idea here is to divide by the distance (this is essentially what cam2pix does).
            dx = dx / z;
            dy = dy / z;
        }

        if (dx*dx < maxInlierXDist2 && dy*dy < maxInlierXDist2 &&
                dd*dd < maxInlierDDist2)
        {
            if (z < maxDist && z > minDist)

//	       if (fabs(f0.kpts[matches[i].queryIdx].pt.y - f1.kpts[matches[i].trainIdx].pt.y) > 300)
            {
//		   std::cout << " ---------- " << dx << "," << dy << "," << dd << ",\npt0 " << pt0.transpose() << "\npt1 " << pt1.transpose() << f0.kpts[matches[i].queryIdx].pt << "," <<
//			 f1.kpts[matches[i].trainIdx].pt << "\n unchanged pt1 " << pt1_unchanged.transpose() << std::endl;
                inliers.push_back(matches[i]);
            }
        }
    }

#if 0
    // Test with the SBA...
    {
        // system
        SysSBA sba;
        sba.verbose = 0;

#if 0
        // set up nodes
        // should have a frame => node function
        Vector4d v0 = Vector4d(0,0,0,1);
        Quaterniond q0 = Quaternion<double>(Vector4d(0,0,0,1));
        sba.addNode(v0, q0, f0.cam, true);

        Quaterniond qr1(rot);   // from rotation matrix
        Vector4d temptrans = Vector4d(trans(0), trans(1), trans(2), 1.0);

        //        sba.addNode(temptrans, qr1.normalized(), f1.cam, false);
        qr1.normalize();
        sba.addNode(temptrans, qr1, f1.cam, false);

        int in = 3;
        if (in > (int)inls.size())
            in = inls.size();

        // set up projections
        for (int i=0; i<(int)inls.size(); i++)
        {
            // add point
            int i0 = inls[i].queryIdx;
            int i1 = inls[i].trainIdx;
            Vector4d pt = f0.pts[i0];
            sba.addPoint(pt);

            // projected point, ul,vl,ur
            Vector3d ipt;
            ipt(0) = f0.kpts[i0].pt.x;
            ipt(1) = f0.kpts[i0].pt.y;
            ipt(2) = ipt(0)-f0.disps[i0];
            sba.addStereoProj(0, i, ipt);

            // projected point, ul,vl,ur
            ipt(0) = f1.kpts[i1].pt.x;
            ipt(1) = f1.kpts[i1].pt.y;
            ipt(2) = ipt(0)-f1.disps[i1];
            sba.addStereoProj(1, i, ipt);
        }

        sba.huber = 2.0;
        sba.doSBA(5,10e-4,SBA_DENSE_CHOLESKY);
        int nbad = sba.removeBad(2.0); // 2.0
        cout << endl << "Removed " << nbad << " projections > 2 pixels error" << endl;
        sba.doSBA(5,10e-5,SBA_DENSE_CHOLESKY);

//        cout << endl << sba.nodes[1].trans.transpose().head(3) << endl;

        // get the updated transform
        trans = sba.nodes[1].trans.head(3);
        Quaterniond q1;
        q1 = sba.nodes[1].qrot;
        quat = q1;
        rot = q1.toRotationMatrix();

        // set up inliers
        inliers.clear();
        for (int i=0; i<(int)inls.size(); i++)
        {
            ProjMap &prjs = sba.tracks[i].projections;
            if (prjs[0].isValid && prjs[1].isValid) // valid track
                inliers.push_back(inls[i]);
        }

        printf("Inliers: %d   After polish: %d\n", (int)inls.size(), (int)inliers.size());
#endif
    }
#endif

//     std::cout << std::endl << trans.transpose().head(3) << std::endl << std::endl;
//     std::cout << rot << std::endl;

    return inliers.size();
}
}
