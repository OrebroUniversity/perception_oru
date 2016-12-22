#include "ndt_map/ndt_cell.h"
#include "ndt_map/lazy_grid.h"
#include "ndt_map/pointcloud_utils.h"
#include "ndt_registration/ndt_matcher_d2d_sc.h"

#include "Eigen/Eigen"
#include <fstream>
#include <omp.h>
#include <sys/time.h>
namespace lslgeneric
{

Eigen::MatrixXd computeHessianMahalanobis(const Eigen::MatrixXd &Q) {
  // Compute the Hessian for the mahalanobis distance Q = the inverse of the covariance matrix(!)
  Eigen::MatrixXd H(6,6);
  H << Q(0,0)+Q(0,0), Q(1,0)+Q(0,1), Q(2,0)+Q(0,2), Q(3,0)+Q(0,3), Q(4,0)+Q(0,4), Q(5,0)+Q(0,5),
       Q(0,1)+Q(1,0), Q(1,1)+Q(1,1), Q(2,1)+Q(1,2), Q(3,1)+Q(1,3), Q(4,1)+Q(1,4), Q(5,1)+Q(1,5),
       Q(0,2)+Q(2,0), Q(1,2)+Q(2,1), Q(2,2)+Q(2,2), Q(3,2)+Q(2,3), Q(4,2)+Q(2,4), Q(5,2)+Q(2,5),
       Q(0,3)+Q(3,0), Q(1,3)+Q(3,1), Q(2,3)+Q(3,2), Q(3,3)+Q(3,3), Q(4,3)+Q(3,4), Q(5,3)+Q(3,5),
       Q(0,4)+Q(4,0), Q(1,4)+Q(4,1), Q(2,4)+Q(4,2), Q(3,4)+Q(4,3), Q(4,4)+Q(4,4), Q(5,4)+Q(4,5),
       Q(0,5)+Q(5,0), Q(1,5)+Q(5,1), Q(2,5)+Q(5,2), Q(3,5)+Q(5,3), Q(4,5)+Q(5,4), Q(5,5)+Q(5,5);

  // A 3DOF hessian...
  // H << 
  //   Q(0,0)+Q(0,0), Q(1,0)+Q(0,1), 0., 0., 0., Q(5,0)+Q(0,5),
  //   Q(0,1)+Q(1,0), Q(1,1)+Q(1,1), 0., 0., 0., Q(5,1)+Q(1,5),
  //   0., 0., 0., 0., 0., 0.,
  //   0., 0., 0., 0., 0., 0.,
  //   0., 0., 0., 0., 0., 0.,
  //   Q(0,5)+Q(5,0), Q(1,5)+Q(5,1), 0., 0., 0., Q(5,5)+Q(5,5);

  //  H.setZero();
       
  return H;
}

double computeScoreMahalanobis(const Eigen::Matrix<double,6,1> &x,
                               const Eigen::MatrixXd &Q) {
  return x.transpose()*Q*x;
  //  return 0.;
}

Eigen::Matrix<double,6,1> computeGradientMahalanobis(const Eigen::Matrix<double,6,1> &x,
                                                     const Eigen::MatrixXd &Q) {
  return computeHessianMahalanobis(Q)*x;
}


double NDTMatcherD2DSC::lineSearchMTSC(Eigen::Matrix<double,6,1> &increment,
                                       std::vector<NDTCell*> &sourceNDT,
                                       NDTMap &targetNDT,
                                       const Eigen::Matrix<double,6,1> &localpose,
                                       const Eigen::MatrixXd &Q,
                                       const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T)
{
    // default params
    double stp = 1.0; //default step
    double recoverystep = 0.1;
    double dginit = 0.0;
    double ftol = 0.11111; //epsilon 1
    double gtol = 0.99999; //epsilon 2
    double stpmax = 4.0;
    double stpmin = 0.001;
    int maxfev = 40; //max function evaluations
    double xtol = 0.01; //window of uncertainty around the optimal step

    //my temporary variables
    std::vector<NDTCell*> sourceNDTHere;
    double score_init = 0.0;

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> ps;
    ps.setIdentity();

    Eigen::Matrix<double,6,1> scg_here;
    Eigen::MatrixXd pincr(6,1), score_gradient_here(6,1);
    Eigen::MatrixXd pseudoH(6,6);
    Eigen::Vector3d eulerAngles;
    Eigen::Matrix<double,6,1> X = localpose;

    /////

    int info = 0;			// return code
    int infoc = 1;		// return code for subroutine cstep

    // Compute the initial gradient in the search direction and check
    // that s is a descent direction.
    score_gradient_here.setZero();
    score_init = derivativesNDT(sourceNDT,targetNDT,score_gradient_here,pseudoH,false);
    score_init += computeScoreMahalanobis(X,Q);
    score_gradient_here += computeGradientMahalanobis(X,Q);

    scg_here = score_gradient_here;
    dginit = increment.dot(scg_here);

    if (dginit >= 0.0)
    {
        std::cout << "MoreThuente::cvsrch - wrong direction (dginit = " << dginit << ")" << std::endl;
        //return recoverystep; //TODO TSV -1; //
        //return -1;

        increment = -increment;
        dginit = -dginit;

        if (dginit >= 0.0)
        {
            for(unsigned int i=0; i<sourceNDTHere.size(); i++)
            {
                if(sourceNDTHere[i]!=NULL)
                    delete sourceNDTHere[i];
            }
            return recoverystep;
        }
    }
    else
    {

    }

    // Initialize local variables.

    bool brackt = false;		// has the soln been bracketed?
    bool stage1 = true;		// are we in stage 1?
    int nfev = 0;			// number of function evaluations
    double dgtest = ftol * dginit; // f for curvature condition
    double width = stpmax - stpmin; // interval width
    double width1 = 2 * width;	// ???

    // initial function value
    double finit = 0.0;
    finit = score_init;

    // The variables stx, fx, dgx contain the values of the step,
    // function, and directional derivative at the best step.  The
    // variables sty, fy, dgy contain the value of the step, function,
    // and derivative at the other endpoint of the interval of
    // uncertainty.  The variables stp, f, dg contain the values of the
    // step, function, and derivative at the current step.

    double stx = 0.0;
    double fx = finit;
    double dgx = dginit;
    double sty = 0.0;
    double fy = finit;
    double dgy = dginit;

    // Get the linear solve tolerance for adjustable forcing term
    //double eta_original = -1.0;
    //double eta = 0.0;
    //eta = eta_original;

    // Start of iteration.

    double stmin, stmax;
    double fm, fxm, fym, dgm, dgxm, dgym;

    while (1)
    {
        // Set the minimum and maximum steps to correspond to the present
        // interval of uncertainty.
        if (brackt)
        {
            stmin = MoreThuente::min(stx, sty);
            stmax = MoreThuente::max(stx, sty);
        }
        else
        {
            stmin = stx;
            stmax = stp + 4 * (stp - stx);
        }

        // Force the step to be within the bounds stpmax and stpmin.
        stp = MoreThuente::max(stp, stpmin);
        stp = MoreThuente::min(stp, stpmax);

        // If an unusual termination is to occur then let stp be the
        // lowest point obtained so far.

        if ((brackt && ((stp <= stmin) || (stp >= stmax))) ||
                (nfev >= maxfev - 1) || (infoc == 0) ||
                (brackt && (stmax - stmin <= xtol * stmax)))
        {
            stp = stx;
        }

        // Evaluate the function and gradient at stp
        // and compute the directional derivative.
        ///////////////////////////////////////////////////////////////////////////

        pincr = stp*increment;

        ps = Eigen::Translation<double,3>(pincr(0),pincr(1),pincr(2))*
             Eigen::AngleAxisd(pincr(3),Eigen::Vector3d::UnitX())*
             Eigen::AngleAxisd(pincr(4),Eigen::Vector3d::UnitY())*
             Eigen::AngleAxisd(pincr(5),Eigen::Vector3d::UnitZ());

        for(unsigned int i=0; i<sourceNDTHere.size(); i++)
        {
            if(sourceNDTHere[i]!=NULL)
                delete sourceNDTHere[i];
        }
        sourceNDTHere.clear();
        for(unsigned int i=0; i<sourceNDT.size(); i++)
        {
            NDTCell *cell = sourceNDT[i];
            if(cell!=NULL)
            {
                Eigen::Vector3d mean = cell->getMean();
                Eigen::Matrix3d cov = cell->getCov();
                mean = ps*mean;
                cov = ps.rotation()*cov*ps.rotation().transpose();
                NDTCell* nd = (NDTCell*)cell->copy();
                nd->setMean(mean);
                nd->setCov(cov);
                sourceNDTHere.push_back(nd);
            }
        }

        Eigen::Affine3d T_previous = T;
        Eigen::Affine3d T_current = ps*T; // Note that ps is not an incremental part, it is redefined in each loop from frame T.
        
        Eigen::Vector3d Tposition_diff = T_current.translation() - T_previous.translation();
        
        //        std::cerr << "Tposition_diff : " << Tposition_diff.transpose() << std::endl;
        //        std::cerr << "pincr : " << pincr.transpose() << std::endl;

        double f = 0.0;
        score_gradient_here.setZero();

        f = derivativesNDT(sourceNDTHere,targetNDT,score_gradient_here,pseudoH,false);
        // X is defined in global coords...
        X(0) = localpose(0) + Tposition_diff(0);
        X(1) = localpose(1) + Tposition_diff(1);
        X(2) = localpose(2) + Tposition_diff(2);
        X(3) = localpose(3) + pincr(3);
        X(4) = localpose(3) + pincr(4);
        X(5) = localpose(3) + pincr(5);

        f += computeScoreMahalanobis(X,Q);
        score_gradient_here += computeGradientMahalanobis(X,Q);

        double dg = 0.0;
        scg_here = score_gradient_here;
        dg = increment.dot(scg_here);


        nfev ++;

///////////////////////////////////////////////////////////////////////////

        // Armijo-Goldstein sufficient decrease
        double ftest1 = finit + stp * dgtest;

        // Test for convergence.

        if ((brackt && ((stp <= stmin) || (stp >= stmax))) || (infoc == 0))
            info = 6;			// Rounding errors

        if ((stp == stpmax) && (f <= ftest1) && (dg <= dgtest))
            info = 5;			// stp=stpmax

        if ((stp == stpmin) && ((f > ftest1) || (dg >= dgtest)))
            info = 4;			// stp=stpmin

        if (nfev >= maxfev)
            info = 3;			// max'd out on fevals

        if (brackt && (stmax-stmin <= xtol*stmax))
            info = 2;			// bracketed soln

        // RPP sufficient decrease test can be different
        bool sufficientDecreaseTest = false;
        sufficientDecreaseTest = (f <= ftest1);  // Armijo-Golstein

        if ((sufficientDecreaseTest) && (fabs(dg) <= gtol*(-dginit)))
            info = 1;			// Success!!!!

        if (info != 0) 		// Line search is done
        {
            if (info != 1) 		// Line search failed
            {
                stp = recoverystep;
            }
            else 			// Line search succeeded
            {

            }
            // Returning the line search flag
            for(unsigned int i=0; i<sourceNDTHere.size(); i++)
            {
                if(sourceNDTHere[i]!=NULL)
                    delete sourceNDTHere[i];
            }
            return stp;

        } // info != 0

        // In the first stage we seek a step for which the modified
        // function has a nonpositive value and nonnegative derivative.

        if (stage1 && (f <= ftest1) && (dg >= MoreThuente::min(ftol, gtol) * dginit))
        {
            stage1 = false;
        }

        // A modified function is used to predict the step only if we have
        // not obtained a step for which the modified function has a
        // nonpositive function value and nonnegative derivative, and if a
        // lower function value has been obtained but the decrease is not
        // sufficient.

        if (stage1 && (f <= fx) && (f > ftest1))
        {

            // Define the modified function and derivative values.

            fm = f - stp * dgtest;
            fxm = fx - stx * dgtest;
            fym = fy - sty * dgtest;
            dgm = dg - dgtest;
            dgxm = dgx - dgtest;
            dgym = dgy - dgtest;

            // Call cstep to update the interval of uncertainty
            // and to compute the new step.

            infoc = MoreThuente::cstep(stx,fxm,dgxm,sty,fym,dgym,stp,fm,dgm,
                                       brackt,stmin,stmax);

            // Reset the function and gradient values for f.

            fx = fxm + stx*dgtest;
            fy = fym + sty*dgtest;
            dgx = dgxm + dgtest;
            dgy = dgym + dgtest;

        }

        else
        {

            // Call cstep to update the interval of uncertainty
            // and to compute the new step.

            infoc = MoreThuente::cstep(stx,fx,dgx,sty,fy,dgy,stp,f,dg,
                                       brackt,stmin,stmax);

        }

        // Force a sufficient decrease in the size of the
        // interval of uncertainty.

        if (brackt)
        {
            if (fabs(sty - stx) >= 0.66 * width1)
                stp = stx + 0.5 * (sty - stx);
            width1 = width;
            width = fabs(sty-stx);
        }

    } // while-loop
}



bool NDTMatcherD2DSC::match( NDTMap& targetNDT,
        NDTMap& sourceNDT,
        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T ,
        const Eigen::MatrixXd& Tcov)
{

    //locals
    bool convergence = false;
    //double score=0;
    double score_best = INT_MAX;
    //double DELTA_SCORE = 0.0005;
    //double NORM_MAX = current_resolution, ROT_MAX = M_PI/10; //
    int itr_ctr = 0;
    //double alpha = 0.95;
    double step_size = 1;
    Eigen::Matrix<double,6,1>  pose_increment_v, scg, pose_local_v, pose_increment_v_Tframe;
    Eigen::MatrixXd Hessian(6,6), score_gradient(6,1); //column vectors, pose_increment_v(6,1)
    Eigen::MatrixXd Q = Tcov.inverse();

    //    std::cout << "Q1 : " << Q << std::endl;
    if (only_xy_motion) {
      Eigen::MatrixXd Q_tmp = Q;
      Q.setZero();
      Q.block(0,0,2,2) = Q_tmp.block(0,0,2,2);
    }
    if (lock_zrp_motion) {
      Q(2,2) = 0; // !0
      Q(3,3) = 1000000000;
      Q(4,4) = 1000000000;
    }
    //    std::cout << "Q2 : " << Q << std::endl;

    Eigen::Matrix<double,6,1> X;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> TR, Tbest;
    Eigen::Vector3d transformed_vec, mean;
    bool ret = true;

    Eigen::Affine3d Tinit = T;

    Tbest = T;
    pose_local_v.setZero();


    Eigen::Array<double,6,1> weights;
    std::vector<NDTCell*> nextNDT = sourceNDT.pseudoTransformNDT(T);

    int iter = 0;
    int best_iter; 
    this->nb_match_calls++;
    bool found_improvement = false;
    
    double alpha = 0.0; // 

    while(!convergence)
    {
      iter++;
        TR.setIdentity();
        Hessian.setZero();
        score_gradient.setZero();

        double score_here = derivativesNDT(nextNDT,targetNDT,score_gradient,Hessian,true);
        X = pose_local_v;

        if (lock_zrp_motion) {
          // Assign the X to be the offset between global z and global roll, pitch.
          X(2) = T.translation()[2];
          // No Euler angles, simply compute the roll and pitch as separate angles.
          double pitch = asin(T.rotation()(2,0)); // Z offset along the x axis...
          double roll = asin(T.rotation()(2,1));  // Z offset along the y axis...
          X(3) = roll;  //T.rotation().eulerAngles(0,1,2)[0];
          X(4) = pitch; //T.rotation().eulerAngles(0,1,2)[1];
          std::cout << "[" << X.transpose() << "]";
        }

        //        std::cout << "score_here : " << score_here << std::flush;
        score_here += alpha*computeScoreMahalanobis(X,Q);
        //        std::cout << " <-> score_here2 : " << computeScoreMahalanobis(X,Q) << std::endl;
        score_gradient += alpha*computeGradientMahalanobis(X,Q);
        //        std::cout << "Hessian : \n" << Hessian << std::endl;
        Hessian += alpha*computeHessianMahalanobis(Q);
        //        std::cout << "Hessian2 : \n" << computeHessianMahalanobis(Q) << std::endl;

        scg = score_gradient;
	if(score_here < score_best) 
	{
	    Tbest = T;
	    score_best = score_here;
            if (iter > 1) {
              if (!found_improvement) {
                this->nb_success_reg++;
                found_improvement = true;
                alpha = 1.;
              }
              std::cerr << "[" << iter << "] X : " << X.transpose() << " -- " << score_here << std::endl;
            }
	}
        else {
          //            std::cerr << "X : " << X.transpose() << std::flush;
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6> > Sol (Hessian);
        Eigen::Matrix<double,6,1> evals = Sol.eigenvalues().real();
        double minCoeff = evals.minCoeff();
        double maxCoeff = evals.maxCoeff();
        if(minCoeff < 0)  //|| evals.minCoeff()) // < 10e-5*evals.maxCoeff()) 
        {
	    if(regularize) {
		Eigen::Matrix<double,6,6> evecs = Sol.eigenvectors().real();
		double regularizer = score_gradient.norm();
		regularizer = regularizer + minCoeff > 0 ? regularizer : 0.001*maxCoeff - minCoeff;
		//double regularizer = 0.001*maxCoeff - minCoeff;
		Eigen::Matrix<double,6,1> reg;
		//ugly
		reg<<regularizer,regularizer,regularizer,regularizer,regularizer,regularizer;
		evals += reg;
		Eigen::Matrix<double,6,6> Lam;
		Lam = evals.asDiagonal();
		Hessian = evecs*Lam*(evecs.transpose());
	    } else {
		if(score_here > score_best) 
		{
		    T = Tbest;
		}
		//de-alloc nextNDT
		for(unsigned int i=0; i<nextNDT.size(); i++)
		{
		    if(nextNDT[i]!=NULL)
			delete nextNDT[i];
		}
		return true;
	    }
        }
        if (score_gradient.norm()<= DELTA_SCORE)
        {
	    if(score_here > score_best) 
	    {
		T = Tbest;
	    }
            //de-alloc nextNDT
            for(unsigned int i=0; i<nextNDT.size(); i++)
            {
                if(nextNDT[i]!=NULL)
                    delete nextNDT[i];
            }
            return true;
        }

        //        std::cerr << " score_here : " << score_here << " score_gradient : " << score_gradient.transpose() << std::endl;

        pose_increment_v = -Hessian.ldlt().solve(score_gradient);

        double dginit = pose_increment_v.dot(scg);
        if(dginit > 0)
        {
            //de-alloc nextNDT
	    if(score_here > score_best) 
	    {
		T = Tbest;
	    }
            for(unsigned int i=0; i<nextNDT.size(); i++)
            {
                if(nextNDT[i]!=NULL)
                    delete nextNDT[i];
            }
            return true;
        }
        //check direction here:

	if(step_control) {
          step_size = lineSearchMTSC(pose_increment_v,nextNDT,targetNDT,pose_local_v, Q, T);
	} else {
	    step_size = 1;
	}
        pose_increment_v = step_size*pose_increment_v;

        // const double max_step = 1.0;
        // if (pose_increment_v.norm() > max_step) {
        //   pose_increment_v.normalize();
        //   pose_increment_v *= max_step;
        // }

        TR.setIdentity();
        TR =  Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
              Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;

        // Make sure that the pose_local_v is updated with the right frame(!)
        Eigen::Affine3d T_previous = T;
        T = TR*T;
        Eigen::Affine3d T_current = T;
        Eigen::Vector3d Tposition_diff = T_current.translation() - T_previous.translation();

        pose_increment_v_Tframe = pose_increment_v;
        pose_increment_v_Tframe(0) = Tposition_diff(0);
        pose_increment_v_Tframe(1) = Tposition_diff(1);
        pose_increment_v_Tframe(2) = Tposition_diff(2);
        pose_local_v += pose_increment_v_Tframe;

        for(unsigned int i=0; i<nextNDT.size(); i++)
        {
	    //TRANSFORM
	    Eigen::Vector3d meanC = nextNDT[i]->getMean();
	    Eigen::Matrix3d covC = nextNDT[i]->getCov();
	    meanC = TR*meanC;
	    covC = TR.rotation()*covC*TR.rotation().transpose();
	    nextNDT[i]->setMean(meanC);
	    nextNDT[i]->setCov(covC);
        }

        if(itr_ctr>0)
        {
            convergence = ((pose_increment_v.norm()) < DELTA_SCORE);
        }
        if(itr_ctr>ITR_MAX)
        {
            convergence = true;
            ret = false;
        }
        itr_ctr++;
    }
    
    score_gradient.setZero();
    double score_here = derivativesNDT(nextNDT,targetNDT,score_gradient,Hessian,false);
    X = pose_local_v;
    if (lock_zrp_motion) {
      // Assign the X to be the offset between global z and global roll, pitch.
      X(2) = T.translation()[2];
      // No Euler angles, simply compute the roll and pitch as separate angles.
      double pitch = asin(T.rotation()(2,0)); // Z offset along the x axis...
      double roll = asin(T.rotation()(2,1));  // Z offset along the y axis...
      X(3) = roll;  //T.rotation().eulerAngles(0,1,2)[0];
      X(4) = pitch; //T.rotation().eulerAngles(0,1,2)[1];
      std::cout << "[" << X.transpose() << "]";
    }
    
    score_here += alpha*computeScoreMahalanobis(X,Q);
    if(score_here > score_best) 
    {
	T = Tbest;
    }
    for(unsigned int i=0; i<nextNDT.size(); i++)
    {
	if(nextNDT[i]!=NULL)
	    delete nextNDT[i];
    }

    return ret;
}
} // namespace
