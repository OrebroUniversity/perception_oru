#include "ndt_map/oc_tree.h"
#include "ndt_map/ndt_cell.h"
#include "ndt_map/lazy_grid.h"
#include "pointcloud_vrml/pointcloud_utils.h"
#include "Eigen/Eigen"
#include <fstream>
#include <omp.h>

#include <gsl/gsl_math.h>
#include <gsl/gsl_deriv.h>
#include <gsl/gsl_multimin.h>

namespace lslgeneric
{

double transforms_objective_xyt (double &x, double &y, double&z, double&r,double&p,double &t, void *params) {
    double res = 0;
    TransformParams *tf = (TransformParams*) params;
    if (tf == NULL) return 0;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> G,Temp;
    Eigen::Matrix<double,4,4> I;
    I.setIdentity();
    Eigen::Vector3d et;
    
    G =  Eigen::Translation<double,3>(x,y,z)*
	Eigen::AngleAxis<double>(r,Eigen::Vector3d::UnitX())*
	Eigen::AngleAxis<double>(p,Eigen::Vector3d::UnitY())*
	Eigen::AngleAxis<double>(t,Eigen::Vector3d::UnitZ()) ;

    for(int i=0; i<tf->fk.size(); i++) 
    {
	Temp = tf->fk[i]*tf->rg[i].inverse();
	if((Temp.translation()).norm() + Temp.rotation().eulerAngles(0,1,2).norm() > 0.5) 
	{
	    std::cout<<"ignoring outlier at "<<i<<std::endl;
	    continue;
	}

	Temp = G*tf->rg[i]*G.inverse()*tf->fk[i].inverse();
	//Temp = tf->fk[i]*G.inverse()*tf->rg[i].inverse()*G;
	//Temp = G*tf->fk[i]*G.inverse()*tf->rg[i].inverse();
	//r = (Temp.translation()).norm() + Temp.rotation().eulerAngles(0,1,2).norm();
//	et = ((tf->rg[i].rotation())*(G.rotation().transpose())*(tf->fk[i].rotation().transpose()) - tf->rg[i].rotation())*G.translation() -
//	    tf->rg[i].rotation()*(G.rotation().transpose())*tf->fk[i].translation() + tf->rg[i].translation();

	//et = (G.inverse()*tf->fk[i]*G).translation() - tf->rg[i].translation();
	//this one seems nice in some way...
	//et = (G*tf->fk[i]*G.inverse()).translation() - tf->rg[i].translation();
//	r = et.norm(); //
//	std::cout<<i<<" : "<<Temp.translation().transpose()<<" "<<et.transpose()<<" R "<<r<<std::endl;
	
	//et = (G*tf->fk[i]).translation() - (tf->rg[i]*G).translation();
	//res +=et.norm();
	Eigen::AngleAxis<double> ax;
	ax.fromRotationMatrix(Temp.rotation());
	res += Temp.translation().norm();// + ax.angle();
    }

    return res;
} 

#if 0
//define objective on two vectors of Eigen transforms and a 6-d pose
double transforms_objective (const gsl_vector *v, void *params) {
    double res = 0, r=0;
    TransformParams *tf = (TransformParams*) params;
    if (tf == NULL) return 0;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> G,Temp;
    Eigen::Matrix<double,4,4> I;
    I.setIdentity();
    Eigen::Vector3d et;
/*
    G =  Eigen::Translation<double,3>(gsl_vector_get(v,0),gsl_vector_get(v,1),gsl_vector_get(v,2))*
	Eigen::AngleAxis<double>(gsl_vector_get(v,3),Eigen::Vector3d::UnitX()) *
	Eigen::AngleAxis<double>(gsl_vector_get(v,4),Eigen::Vector3d::UnitY()) *
	Eigen::AngleAxis<double>(gsl_vector_get(v,5),Eigen::Vector3d::UnitZ()) ;
	*/
    G =  Eigen::Translation<double,3>(gsl_vector_get(v,0),gsl_vector_get(v,1),0)*
	Eigen::AngleAxis<double>(gsl_vector_get(v,2),Eigen::Vector3d::UnitZ()) ;

    for(int i=0; i<tf->fk.size(); i++) 
    {
	Temp = tf->fk[i]*tf->rg[i].inverse();
	if((Temp.translation()).norm() + Temp.rotation().eulerAngles(0,1,2).norm() > 0.2) 
	{
	    std::cout<<"ignoring outlier at "<<i<<std::endl;
	    continue;
	}

	Temp = G*tf->rg[i]*G.inverse()*tf->fk[i].inverse();
	//Temp = G*tf->fk[i].inverse()*G.inverse()*tf->rg[i];
	//Temp = G*tf->fk[i]*G.inverse()*tf->rg[i].inverse();
	//r = (Temp.translation()).norm() + Temp.rotation().eulerAngles(0,1,2).norm();
	et = ((tf->rg[i].rotation())*(G.rotation().transpose())*(tf->fk[i].rotation().transpose()) - tf->rg[i].rotation())*G.translation() -
	    tf->rg[i].rotation()*(G.rotation().transpose())*tf->fk[i].translation() + tf->rg[i].translation();
	et = (G*tf->fk[i]*G.inverse()).translation() - tf->rg[i].translation();
	//r = Temp.translation().norm();//
	r = et.norm(); //
//	std::cout<<i<<" : "<<Temp.translation().transpose()<<" "<<et.transpose()<<" R "<<r<<std::endl;
	
	res +=r;
    }

    return res;
} 
#endif

template <typename PointSource>
void deallocate(std::vector<NDTCell<PointSource>* > &ndts) {
	    for(unsigned int i=0; i<ndts.size(); i++)
	    {
		if(ndts[i]!=NULL)
		    delete ndts[i];
	    }
}

//#define DO_DEBUG_PROC

template <typename PointSource>
bool NDTMatcherSequentialD2D<PointSource>::add_cloud( pcl::PointCloud<PointSource>& cloud,
	Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& Tref) 
{
    pcs.push_back(cloud);
    LazyGrid<PointSource> prototype(current_resolution);
    NDTMap<PointSource> *NDT = new NDTMap<PointSource>( &prototype );
    NDT->loadPointCloud( cloud );
    NDT->computeNDTCells();

    transforms.push_back(Tref);
    maps.push_back(NDT);
}

template <typename PointSource>
bool NDTMatcherSequentialD2D<PointSource>::match_all(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& RES, bool useInitialGuess) 
{

    double __res[] = {0.1,0.2};
    //double __res[] = {0.5,1};
    std::vector<double> resolutions (__res, __res+sizeof(__res)/sizeof(double));
    std::vector<Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>, 
	Eigen::aligned_allocator<Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> > > reg_transforms, rel_transforms, err_transforms;

    lslgeneric::NDTMatcherD2D<PointSource,PointSource> matcherD2D(false, false, resolutions);
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tout, tempT; 
    Tout.setIdentity();

    char fname[50];
    FILE *fout;

    for(int i=1; i<pcs.size(); ++i) 
    {
	Tout.setIdentity();
	Tout = (transforms[i-1].inverse()*transforms[i]);
	std::cout<<Tout.matrix()<<std::endl;
	bool ret = matcherD2D.match(pcs[i-1],pcs[i],Tout,true);
	reg_transforms.push_back(Tout);
	
	snprintf(fname,49,"c_offset_%02d.wrl",i);
        fout = fopen(fname,"w");
        fprintf(fout,"#VRML V2.0 utf8\n");
        lslgeneric::writeToVRML<PointSource>(fout,pcs[i-1],Eigen::Vector3d(1,0,0));
        //lslgeneric::writeToVRML<PointSource>(fout,pcs[i],Eigen::Vector3d(1,1,1));
	pcl::PointCloud<PointSource> tmp1 = pcs[i];
	tempT =  (transforms[i-1].inverse()*transforms[i]);
        lslgeneric::transformPointCloudInPlace<PointSource>(tempT,tmp1);
        lslgeneric::writeToVRML<PointSource>(fout,tmp1,Eigen::Vector3d(1,1,1));
	pcl::PointCloud<PointSource> tmp2 = pcs[i];
        lslgeneric::transformPointCloudInPlace<PointSource>(Tout,tmp2);
        lslgeneric::writeToVRML<PointSource>(fout,tmp2,Eigen::Vector3d(0,1,0));
        fclose(fout);
	
/*        
*/
    }

    for(int i=1; i<transforms.size(); ++i) 
    {
	std::cout<<"rel: "<<i<<std::endl;
	rel_transforms.push_back( (transforms[i-1].inverse()*transforms[i]));
	std::cout<<rel_transforms[i-1].translation().transpose()<<" "<<rel_transforms[i-1].rotation().eulerAngles(0,1,2).transpose()<<std::endl;
	std::cout<<reg_transforms[i-1].translation().transpose()<<" "<<reg_transforms[i-1].rotation().eulerAngles(0,1,2).transpose()<<std::endl;
    }
#if 0
    Eigen::Vector3d v,x;
    Eigen::Matrix3d M, U, L;
    //now the optimization part...
    //go through transforms vector and compute relative transforms
    for(int i=1; i<transforms.size(); ++i) 
    {
	Tout.setIdentity();
	U = (rel_transforms[i-1].rotation()).llt().matrixL();
	L = (reg_transforms[i-1].rotation()).llt().matrixL();
	Tout.rotate(U.transpose()*L);
	v = rel_transforms[i-1].translation() - Tout.rotation().transpose()*(reg_transforms[i-1].translation());
	M = Tout.rotation().transpose()*reg_transforms[i-1].rotation() - Eigen::Matrix3d::Identity();

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
        x = svd.solve(v);
	std::cout<<"M = "<<M<<std::endl;
	std::cout<<"v = "<<v.transpose()<<std::endl;
	std::cout<<"x = "<<x.transpose()<<std::endl;
	Tout.translation() = x;	
	
	std::cout<<Tout.translation().transpose()<<" "<<Tout.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
	err_transforms.push_back(Tout);
    }

#endif

#if 0
    TransformParams tf;
    tf.fk = rel_transforms;
    tf.rg = reg_transforms;

    const gsl_multimin_fminimizer_type *T = 
	gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = NULL;
    gsl_vector *ss, *x;
    gsl_multimin_function minex_func;

    size_t iter = 0;
    int status;
    double size;

    /* Starting point */
    x = gsl_vector_alloc (3);
    gsl_vector_set_all (x, 0.0);
 /*   gsl_vector_set(x, 0, 1.18);
    gsl_vector_set(x, 1, -0.3);
    gsl_vector_set(x, 2, 0);
 */ 
    /* Set initial step sizes to 1 */
    ss = gsl_vector_alloc (3);
    gsl_vector_set_all (ss, 1);

    /* Initialize method and iterate */
    minex_func.n = 3;
    minex_func.f = transforms_objective;
    minex_func.params = &tf;

    s = gsl_multimin_fminimizer_alloc (T, 3);
    gsl_multimin_fminimizer_set (s, &minex_func, x, ss);

    do
    {
	iter++;
	status = gsl_multimin_fminimizer_iterate(s);

	if (status) 
	    break;

	size = gsl_multimin_fminimizer_size (s);
	status = gsl_multimin_test_size (size, 1e-2);

	if (status == GSL_SUCCESS)
	{
	    printf ("converged to minimum at\n");
	}

	printf ("%5d %10.3e %10.3e %10.3ef() = %7.3f size = %.3f\n", 
		iter,
		gsl_vector_get (s->x, 0), 
		gsl_vector_get (s->x, 1), 
		gsl_vector_get (s->x, 2), 
//		gsl_vector_get (s->x, 3), 
//		gsl_vector_get (s->x, 4), 
//		gsl_vector_get (s->x, 5), 
		s->fval, size);
    }
    while (status == GSL_CONTINUE && iter < 200);
    RES =  Eigen::Translation<double,3>(gsl_vector_get(s->x,0),gsl_vector_get(s->x,1),0)* //gsl_vector_get(s->x,2))*
//	Eigen::AngleAxis<double>(gsl_vector_get(s->x,3),Eigen::Vector3d::UnitX()) *
//	Eigen::AngleAxis<double>(gsl_vector_get(s->x,4),Eigen::Vector3d::UnitY()) *
	Eigen::AngleAxis<double>(gsl_vector_get(s->x,2),Eigen::Vector3d::UnitZ()) ;

    gsl_vector_free(x);
    gsl_vector_free(ss);
    gsl_multimin_fminimizer_free (s);
#endif

    TransformParams tf;
    tf.fk = rel_transforms;
    tf.rg = reg_transforms;

    double res_min = 0.004, res_max = 0.01;
    double res = res_max;

    double xmin = -0.1, xmax =0.1;
    double ymin = -0.1, ymax =0.1;
    double zmin = -0.1, zmax =0.1;
    double rmin = -0.1, rmax =0.1;
    double pmin = -0.1, pmax =0.1;
    double tmin = -0.1, tmax =0.1;

    double xx=0,yy=0,zz=0,rr=0,pp=0,tt=0;
    int ctr =0;
    double score_min = INT_MAX;
    double score_here;

    double y = 0;
//    double z = 0;
    double r=0; 
    double p=0;
    while ( res > res_min) {
	for(double x=xmin; x<xmax; x+=res) {
//	    for(double y=ymin; y<ymax; y+=res) {
		for(double z=zmin; z<zmax; z+=res) {
//		    for(double r=rmin; r<rmax; r+=res) {
//			for(double p=pmin; p<pmax; p+=res) {
			    for(double t=tmin; t<tmax; t+=res) {
				score_here = transforms_objective_xyt (x,y,z,r,p,t,(void*)&tf);
				if(score_here < score_min) {
				    xx = x;
				    yy = y;
				    zz = z;
				    rr = r;
				    pp = p;
				    tt = t;
				    score_min = score_here;
				    std::cout<<"new min of "<<score_here<<" at "<<x<<" "<<y<<" "<<" "<<z<<" "<<r<<" "<<p<<" "<<t<<std::endl;
				}		
				//if(ctr%100 == 0) std::cout<<"x y t"<<x<<" "<<y<<" "<<t<<std::endl;
				ctr++;		    
			    }
//			}
//		    }
		}
//	    }
	}
	res = res/2;
	xmin = xx-10*res;
	ymin = yy-10*res;
	zmin = zz-10*res;
	rmin = rr-2*res;
	pmin = pp-2*res;
	tmin = tt-10*res;
	xmax = xx+10*res;
	ymax = yy+10*res;
	zmax = zz+10*res;
	rmax = rr+2*res;
	pmax = pp+2*res;
	tmax = tt+10*res;
	std::cout<<"going to RES "<<res<<" bounds +/- "<<xmax<<" "<<ymax<<" "<<zmax<<" "<<rmax<<" "<<pmax<<" "<<tmax<<std::endl;
    }
    
    std::cout<<"FINAL min at "<<score_min<<" at "<<xx<<" "<<yy<<" "<<zz<<" "<<rr<<" "<<pp<<" "<<tt<<std::endl;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> RES2;
    RES =  Eigen::Translation<double,3>(xx,yy,zz)* //gsl_vector_get(s->x,2))*
	Eigen::AngleAxis<double>(tt,Eigen::Vector3d::UnitZ()) ;

    for(int i=1; i<pcs.size(); ++i) 
    {
	snprintf(fname,49,"calib_offset_%02d.wrl",i);
        fout = fopen(fname,"w");
        fprintf(fout,"#VRML V2.0 utf8\n");
	pcl::PointCloud<PointSource> tmp0 = pcs[i-1];
	//RES2 = transforms[i-1].inverse()*RES*transforms[i-1];
	//std::cout<<"Rinv = "<<RES2.matrix()<<std::endl;
	//tempT = RES*transforms[i-1];
	tempT = transforms[i-1]*RES;
        lslgeneric::transformPointCloudInPlace<PointSource>(tempT,tmp0);
        lslgeneric::writeToVRML<PointSource>(fout,tmp0,Eigen::Vector3d(1,0,0));
	pcl::PointCloud<PointSource> tmp1 = pcs[i];
	//RES2 = transforms[i].inverse()*RES*transforms[i];
	tempT = transforms[i]*RES;
        lslgeneric::transformPointCloudInPlace<PointSource>(tempT,tmp1);
        lslgeneric::writeToVRML<PointSource>(fout,tmp1,Eigen::Vector3d(1,1,1));
	tmp0 = pcs[i-1];
	tempT = transforms[i-1];
        lslgeneric::transformPointCloudInPlace<PointSource>(tempT,tmp0);
        lslgeneric::writeToVRML<PointSource>(fout,tmp0,Eigen::Vector3d(1,1,0));
	tmp1 = pcs[i];
	tempT = transforms[i];
        lslgeneric::transformPointCloudInPlace<PointSource>(tempT,tmp1);
        lslgeneric::writeToVRML<PointSource>(fout,tmp1,Eigen::Vector3d(1,0,1));
        fclose(fout);


    }
}

#if 0
template <typename PointSource>
bool NDTMatcherSequentialD2D<PointSource>::match_all(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T, bool useInitialGuess) 
{

    //locals
    bool convergence = false;
    //double score=0;
    double DELTA_SCORE = 10e-3*current_resolution;
    //double DELTA_SCORE = 0.0005;
    //double NORM_MAX = current_resolution, ROT_MAX = M_PI/10; //
    int itr_ctr = 0;
    //double alpha = 0.95;
    double step_size = 1;
    Eigen::Matrix<double,6,1>  pose_increment_v, pose_accum, scg;
    Eigen::MatrixXd Hessian(6,6), score_gradient(6,1); //column vectors, pose_increment_v(6,1)
    Eigen::MatrixXd Hessian_loc(6,6), score_gradient_loc(6,1); //column vectors, pose_increment_v(6,1)

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> TR, Tloc;
    Eigen::Vector3d transformed_vec, mean;
    bool ret = true;
    if(!useInitialGuess)
    {
        T.setIdentity();
    }

    Eigen::Array<double,6,1> weights;
    Eigen::Quaterniond qsum, qloc;
    Eigen::Vector4d qsum_vec, qloc_vec;

//    std::cout<<"pose(:,"<<1<<") = ["<<T.translation().transpose()<<" "<<T.rotation().eulerAngles(0,1,2).transpose()<<"]';\n";
    while(!convergence)
    {
	pose_accum.setZero();
	double score_here = 0;
	int nsuccess =0;
	for(int i=0; i<maps.size(); i++) 
	{
	    TR.setIdentity();
	    Hessian.setZero();
	    score_gradient.setZero();
	    Tloc = transforms[i]*T;
	    std::vector<NDTCell<PointSource>*> nextNDT = maps[i]->pseudoTransformNDT(Tloc);
	    std::vector<NDTCell<PointSource>*> fixedNDT;
//	    for(int j=0; j<maps.size(); j++) 
	    for(int j=i-3; j<i+3; j++) 
	    {
		if(j < 0 || j >= maps.size()) continue;
		if(i == j) continue;
		Tloc = transforms[j]*T;
		std::vector<NDTCell<PointSource>*> locNDT = maps[j]->pseudoTransformNDT(Tloc);
		fixedNDT.insert(fixedNDT.begin(), locNDT.begin(), locNDT.end());
	    }
	    score_here += derivativesNDT(nextNDT,fixedNDT,score_gradient,Hessian,true);
	    
	    scg = score_gradient;
	    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6> > Sol (Hessian);
	    Eigen::Matrix<double,6,1> evals = Sol.eigenvalues().real();
	    double minCoeff = evals.minCoeff();
	    double maxCoeff = evals.maxCoeff();
	    if(minCoeff < 0 )   
	    {
		Eigen::Matrix<double,6,6> evecs = Sol.eigenvectors().real();
		double regularizer = 0.01*maxCoeff - minCoeff;
		Eigen::Matrix<double,6,1> reg;
		//ugly
		reg<<regularizer,regularizer,regularizer,regularizer,regularizer,regularizer;
		evals += reg;
		Eigen::Matrix<double,6,6> Lam;
		Lam = evals.asDiagonal();
		Hessian = evecs*Lam*(evecs.transpose());
		std::cerr<<"regularizing\n";
	    }

	    //std::cout<<"s("<<itr_ctr+1<<") = "<<score_here<<";\n";
	    //std::cout<<"H(:,:,"<<itr_ctr+1<<")  =  ["<< Hessian<<"];\n"<<std::endl;				  //
	    //std::cout<<"grad (:,"<<itr_ctr+1<<")= ["<<score_gradient.transpose()<<"];"<<std::endl;         //

	    if (score_gradient.norm()<= DELTA_SCORE)
	    {
		//	    std::cout<<"incr(:,"<<itr_ctr+1<<") = [0 0 0 0 0 0]';\n";
		std::cout<<"\%gradient vanished\n";
		deallocate(nextNDT);
		deallocate(fixedNDT);
		continue;
	    }

	    pose_increment_v = -Hessian.ldlt().solve(score_gradient);
	    double dginit = pose_increment_v.dot(scg);
	    if(dginit > 0)
	    {
		std::cout<<"\%can't decrease in this direction any more, done \n";
		deallocate(nextNDT);
		deallocate(fixedNDT);
		continue;
	    }

	    step_size = lineSearchMT(pose_increment_v,nextNDT,fixedNDT);
//	    step_size = 0.1;
	    pose_increment_v = step_size*pose_increment_v;
	    std::cout<<"\%iteration "<<itr_ctr<<" pose "<<(pose_increment_v.transpose())<<" score "<<score_here<<" step "<<step_size<<std::endl;
	    if(pose_increment_v.norm() > 1) {
		deallocate(nextNDT);
		deallocate(fixedNDT);
		continue;
	    }
	    qloc = Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;

	    qloc_vec(0) = qloc.x();
	    qloc_vec(1) = qloc.y();
	    qloc_vec(2) = qloc.z();
	    qloc_vec(3) = qloc.w();
	    qsum_vec = qsum_vec+qloc_vec;
	    std::cout<<"Q "<<qsum_vec.transpose()<<"--"<<qloc_vec.transpose()<<std::endl;
	    pose_accum += pose_increment_v;
	    nsuccess ++;
	    
	    deallocate(nextNDT);
	    deallocate(fixedNDT);

	}
	pose_accum /= nsuccess;
	qsum_vec = qsum_vec/nsuccess;

	std::cout<<"qs "<<qsum_vec.transpose()<<std::endl;	
	qsum.x() = qsum_vec(0);
	qsum.y() = qsum_vec(1);
	qsum.z() = qsum_vec(2);
	qsum.w() = qsum_vec(3);
	qsum.normalize();

	TR.setIdentity();
	TR =  Eigen::Translation<double,3>(pose_accum(0),pose_accum(1),0)*qsum;

        T = TR*T;
	std::cout<<"incr(:,"<<itr_ctr+1<<") = ["<<TR.translation().transpose()<<" "<<TR.rotation().eulerAngles(0,1,2).transpose()<<"]';\n";
	std::cout<<"pose(:,"<<itr_ctr+2<<") = ["<<T.translation().transpose()<<" "<<T.rotation().eulerAngles(0,1,2).transpose()<<"]';\n";

        if(itr_ctr>0)
        {
            convergence = ((TR.translation().norm() + TR.rotation().eulerAngles(0,1,2).norm()) < DELTA_SCORE);
            //convergence = ((score_gradient.norm()) < DELTA_SCORE);
        }
        if(itr_ctr>ITR_MAX)
        {
            convergence = true;
            ret = false;
        }
        itr_ctr++;
        //step_size *= alpha;
        //std::cout<<"step size "<<step_size<<std::endl;
    }
//    std::cout<<"incr(:,"<<itr_ctr+1<<") = [0 0 0 0 0 0]';\n";
//    std::cout<<"grad(:,"<<itr_ctr+1<<") = [0 0 0 0 0 0]';\n";
    /*
        snprintf(fname,49,"final.wrl");
        fout = fopen(fname,"w");
        fprintf(fout,"#VRML V2.0 utf8\n");
        targetNDT.writeToVRML(fout,Eigen::Vector3d(1,0,0));
        for(unsigned int i=0; i<nextNDT.size(); i++)
        {
    	if(nextNDT[i]!=NULL)
    	{
    	    nextNDT[i]->writeToVRML(fout,Eigen::Vector3d(0,1,0));
    	}
        }
        fclose(fout);
    */

    return ret;
}
#endif

//iteratively update the score gradient and hessian
template <typename PointSource>
bool NDTMatcherSequentialD2D<PointSource>::update_gradient_hessian_local(
    Eigen::MatrixXd &score_gradient,
    Eigen::MatrixXd &Hessian,
    const Eigen::Vector3d & x,
    const Eigen::Matrix3d & B,
    const double &likelihood,
    const Eigen::Matrix<double,3,6> &_Jest,
    const Eigen::Matrix<double,18,6> &_Hest,
    const Eigen::Matrix<double,3,18> &_Zest,
    const Eigen::Matrix<double,18,18> &_ZHest,
    bool computeHessian)
{


    //vars for gradient
    Eigen::Matrix<double,6,1> _xtBJ, _xtBZBx, _Q;
    //vars for hessian
    Eigen::Matrix<double,6,6> _xtBZBJ, _xtBH, _xtBZBZBx, _xtBZhBx;
    Eigen::Matrix<double,1,3> _TMP1, _xtB;

    _xtBJ.setZero();
    _xtBZBx.setZero();
    _Q.setZero();
    _xtBZBJ.setZero();
    _xtBH.setZero();
    _xtBZBZBx.setZero();
    _xtBZhBx.setZero();
    _TMP1.setZero();
    _xtB.setZero();

    _xtB = x.transpose()*B;
    _xtBJ = _xtB*_Jest;

    for(unsigned int i=0; i<6; i++)
    {
        _TMP1 = _xtB*_Zest.block<3,3>(0,3*i)*B;
        _xtBZBx(i) = _TMP1*x;
        if(computeHessian)
        {
            _xtBZBJ.col(i) = (_TMP1*_Jest).transpose(); //-
            for(unsigned int j=0; j<6; j++)
            {
                _xtBH(i,j) = _xtB*_Hest.block<3,1>(3*i,j);
                _xtBZBZBx(i,j) = _TMP1*_Zest.block<3,3>(0,3*j)*B*x;
                _xtBZhBx(i,j) = _xtB*_ZHest.block<3,3>(3*i,3*j)*B*x;
            }
        }
    }
    _Q = 2*_xtBJ-_xtBZBx;
    double factor = -(lfd2/2)*likelihood;
    score_gradient += _Q*factor;

    if(computeHessian)
    {
        Hessian += factor*(2*_Jest.transpose()*B*_Jest+2*_xtBH -_xtBZhBx -2*_xtBZBJ.transpose()
                           -2*_xtBZBJ +_xtBZBZBx +_xtBZBZBx.transpose() -lfd2*_Q*_Q.transpose()/2 ); // + Eigen::Matrix<double,6,6>::Identity();

    }
    return true;
}

//pre-computes the derivative matrices Jest, Hest, Zest, ZHest
template <typename PointSource>
void NDTMatcherSequentialD2D<PointSource>::computeDerivativesLocal(Eigen::Vector3d &x, Eigen::Matrix3d C1,
        Eigen::Matrix<double,3,6> &_Jest,
        Eigen::Matrix<double,18,6> &_Hest,
        Eigen::Matrix<double,3,18> &_Zest,
        Eigen::Matrix<double,18,18> &_ZHest,
        bool computeHessian)
{

    _Jest(0,4) = x(2);
    _Jest(0,5) = -x(1);
    _Jest(1,3) = -x(2);
    _Jest(1,5) = x(0);
    _Jest(2,3) = x(1);
    _Jest(2,4) = -x(0);
    Eigen::Vector3d a,b,c,d,e,f;
    a<<0,-x(1),-x(2);
    b<<0,x(0),0;
    c<<0,0,x(0);
    d<<-x(0),0,-x(2);
    e<<0,0,x(1);
    f<<-x(0),-x(1),0;

    Eigen::Matrix3d myBlock;
    //_Zest
    myBlock<<
           0,       -C1(0,2),      C1(0,1),
                    -C1(0,2),     -2*C1(1,2), -C1(2,2) + C1(1,1),
                    C1(0,1), -C1(2,2) + C1(1,1),    2*C1(1,2);
    _Zest.block<3,3>(0,9) = myBlock;
    myBlock<<
           2*C1(0,2), C1(1,2), -C1(0,0) + C1(2,2),
             C1(1,2),    0,       -C1(0,1),
             -C1(0,0) + C1(2,2),  -C1(0,1),     -2*C1(0,2);
    _Zest.block<3,3>(0,12) = myBlock;
    myBlock<<
           -2*C1(0,1), -C1(1,1) + C1(0,0),  -C1(1,2),
           -C1(1,1) + C1(0,0),    2*C1(0,1), C1(0,2),
           -C1(1,2),      C1(0,2),    0;
    _Zest.block<3,3>(0,15) = myBlock;

    if(computeHessian)
    {
        //Hest
        _Hest.block<3,1>(9,3) = a;
        _Hest.block<3,1>(12,3) = b;
        _Hest.block<3,1>(15,3) = c;
        _Hest.block<3,1>(9,4) = b;
        _Hest.block<3,1>(12,4) = d;
        _Hest.block<3,1>(15,4) = e;
        _Hest.block<3,1>(9,5) = c;
        _Hest.block<3,1>(12,5) = e;
        _Hest.block<3,1>(15,5) = f;

        //_ZHest
        myBlock<<
               0,          -C1(0,1),          -C1(0,2),
                           -C1(0,1), 2*C1(2,2) - 2*C1(1,1),        -4*C1(1,2),
                           -C1(0,2),        -4*C1(1,2), 2*C1(1,1) - 2*C1(2,2);
        _ZHest.block<3,3>(9,9) =   myBlock;

        myBlock<<
               0, C1(0,0) - C1(2,2),    C1(1,2),
                  C1(0,0) - C1(2,2),     2*C1(0,1),  2*C1(0,2),
                  C1(1,2),     2*C1(0,2), -2*C1(0,1);
        _ZHest.block<3,3>(9,12) = myBlock;

        myBlock<<
               0,    C1(1,2), C1(0,0) - C1(1,1),
                     C1(1,2), -2*C1(0,2),     2*C1(0,1),
                     C1(0,0) - C1(1,1),  2*C1(0,1),     2*C1(0,2);
        _ZHest.block<3,3>(9,15) = myBlock;

        myBlock<<
               2*C1(2,2) - 2*C1(0,0), -C1(0,1),        -4*C1(0,2),
                 -C1(0,1),    0,          -C1(1,2),
                 -4*C1(0,2), -C1(1,2), 2*C1(0,0) - 2*C1(2,2);
        _ZHest.block<3,3>(12,12) = myBlock;

        myBlock<<
               -2*C1(1,2),       C1(0,2),     2*C1(0,1),
               C1(0,2),         0, C1(1,1) - C1(0,0),
               2*C1(0,1), C1(1,1) - C1(0,0),     2*C1(1,2);
        _ZHest.block<3,3>(12,15) = myBlock;

        myBlock<<
               2*C1(1,1) - 2*C1(0,0),        -4*C1(0,1), -C1(0,2),
                 -4*C1(0,1), 2*C1(0,0) - 2*C1(1,1), -C1(1,2),
                 -C1(0,2),          -C1(1,2),    0;
        _ZHest.block<3,3>(15,15)= myBlock;

        _ZHest.block<3,3>(12,9) =    _ZHest.block<3,3>(9,12);
        _ZHest.block<3,3>(15,9) =    _ZHest.block<3,3>(9,15);
        _ZHest.block<3,3>(15,12)=    _ZHest.block<3,3>(12,15);
    }
}


template <typename PointSource>
//compute the score gradient of a point cloud + transformation to an NDT
double NDTMatcherSequentialD2D<PointSource>::derivativesNDT(
    const std::vector<NDTCell<PointSource>*> &sourceNDT,
    const std::vector<NDTCell<PointSource>*> &targetNDT,
    Eigen::MatrixXd &score_gradient,
    Eigen::MatrixXd &Hessian,
    bool computeHessian
)
{


    struct timeval tv_start, tv_end;
    double score_here = 0;
    int n_dimensions = score_gradient.rows();

    gettimeofday(&tv_start,NULL);
    score_gradient.setZero();
    Hessian.setZero();

    Eigen::MatrixXd score_gradient_omp;
    Eigen::MatrixXd score_here_omp;
    Eigen::MatrixXd Hessian_omp;

#define n_threads 8

    //n_threads = omp_get_num_threads();
    score_gradient_omp.resize(n_dimensions,n_threads);
    score_here_omp.resize(1,n_threads);
    Hessian_omp.resize(n_dimensions,n_dimensions*n_threads);

    score_gradient_omp.setZero();
    score_here_omp.setZero();
    Hessian_omp.setZero();
    //std::cout<<n_threads<<" "<<omp_get_thread_num()<<std::endl;

    #pragma omp parallel num_threads(n_threads)
    {
        #pragma omp for
        for(unsigned int i=0; i<sourceNDT.size(); i++)
        {
            PointSource point;
            Eigen::Vector3d transformed;
            Eigen::Vector3d meanMoving, meanFixed;
            Eigen::Matrix3d CMoving, CFixed, CSum, Cinv, R;
            Eigen::MatrixXd score_gradient_omp_loc(n_dimensions,1);
            Eigen::MatrixXd Hessian_omp_loc(n_dimensions,n_dimensions);
            Eigen::Matrix<double,3,6> _Jest;
            Eigen::Matrix<double,18,6> _Hest;
            Eigen::Matrix<double,3,18> _Zest;
            Eigen::Matrix<double,18,18> _ZHest;
            double score_here_loc=0;
            int thread_id = omp_get_thread_num();
            NDTCell<PointSource> *cell;
            bool exists = false;
            double det = 0;


            score_gradient_omp_loc.setZero();
            Hessian_omp_loc.setZero();
            _Jest.setZero();
            _Jest.block<3,3>(0,0).setIdentity();
            _Hest.setZero();
            _Zest.setZero();
            _ZHest.setZero();

            meanMoving = sourceNDT[i]->getMean();
            CMoving= sourceNDT[i]->getCov();
            computeDerivativesLocal(meanMoving, CMoving, _Jest, _Hest, _Zest, _ZHest, computeHessian);

            point.x = meanMoving(0);
            point.y = meanMoving(1);
            point.z = meanMoving(2);
            std::vector<NDTCell<PointSource>*> cells = targetNDT; //targetNDT.getCellsForPoint(point,2); //targetNDT.getAllCells(); //
            for(int j=0; j<cells.size(); j++)
            {
                cell = cells[j];
                if(cell == NULL)
                {
                    continue;
                }
                if(cell->hasGaussian_)
                {
                    transformed = meanMoving - cell->getMean();
		    if(transformed.norm() > 8*current_resolution) continue;
                    CFixed = cell->getCov();
                    CSum = (CFixed+CMoving);
                    CSum.computeInverseAndDetWithCheck(Cinv,det,exists);
                    if(!exists)
                    {
                        continue;
                    }
                    double l = (transformed).dot(Cinv*(transformed));
                    if(l*0 != 0)
                    {
                        continue;
                    }
                    //if(l > 120) continue;
                    double sh = -lfd1*(exp(-lfd2*l/2));
                    if(!update_gradient_hessian_local(score_gradient_omp_loc,Hessian_omp_loc,transformed, Cinv, sh,
                                                      _Jest, _Hest, _Zest, _ZHest, computeHessian))
                    {
                        continue;
                    }
                    score_here_loc += sh;
                    cell = NULL;
                }
            }
            //score_gradient_omp.block(0,thread_id,n_dimensions,1) += score_gradient_omp_loc;
            score_gradient_omp.col(thread_id) += score_gradient_omp_loc;
            Hessian_omp.block(0,n_dimensions*thread_id,n_dimensions,n_dimensions) += Hessian_omp_loc;
            score_here_omp(0,thread_id) += score_here_loc;

        }
    } //end pragma block
    //std::cout<<"sgomp: "<<score_gradient_omp<<std::endl;
    //std::cout<<"somp: "<<score_here_omp<<std::endl;

    score_gradient = score_gradient_omp.rowwise().sum();
    score_here = score_here_omp.sum();
    if(computeHessian)
    {
        //std::cout<<"Homp: "<<Hessian_omp<<std::endl;
        for(int i=0; i<n_threads; ++i)
        {
            Hessian += Hessian_omp.block(0,n_dimensions*i,n_dimensions,n_dimensions);
        }
    }
    /*
    if(computeHessian) {

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6> > Sol (Hessian);
        Eigen::Matrix<double,6,1> evals = Sol.eigenvalues().real();

        double minCoeff = evals.minCoeff();
        double maxCoeff = evals.maxCoeff();

        if(minCoeff < 0 ) { //evals.minCoeff() < 0.01*evals.maxCoeff()) {
    	Eigen::Matrix<double,6,6> evecs = Sol.eigenvectors().real();
    	double regularizer = 0.01*maxCoeff - minCoeff;
    	Eigen::Matrix<double,6,1> reg;
    	//ugly
    	reg<<regularizer,regularizer,regularizer,regularizer,regularizer,regularizer;
    	evals += reg;
    	Eigen::Matrix<double,6,6> Lam;
    	Lam = evals.asDiagonal();
    	Hessian = evecs*Lam*(evecs.transpose());
    	std::cerr<<"regularizing\n";
        }
        */
    /*
    if(minCoeff < 0.001*maxCoeff) {
    Eigen::Matrix<double,6,6> evecs = Sol.eigenvectors().real();
    for(int q=0;q<6;++q) {
        if(evals(q) < 0.001*maxCoeff) {
    	evals(q)=0.001*maxCoeff;
        } else{
    	break;
        }
    }
    Eigen::Matrix<double,6,6> Lam;
    Lam = evals.asDiagonal();
    Hessian = evecs*Lam*(evecs.transpose());
    std::cerr<<"BAD_HESSIAN\n";
    }
    }
    */
    gettimeofday(&tv_end,NULL);

    double time_load = (tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.;
    //std::cout<<"time derivatives took is: "<<time_load<<std::endl;
    return score_here;
}

//perform line search to find the best descent rate (More&Thuente)
template <typename PointSource>
double NDTMatcherSequentialD2D<PointSource>::lineSearchMT(
    Eigen::Matrix<double,6,1> &increment,
    std::vector<NDTCell<PointSource>*> &sourceNDT,
    std::vector<NDTCell<PointSource>*> &targetNDT
)
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
    std::vector<NDTCell<PointSource>*> sourceNDTHere;
    double score_init = 0.0;

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> ps;
    ps.setIdentity();

    Eigen::Matrix<double,6,1> scg_here;
    Eigen::MatrixXd pincr(6,1), score_gradient_here(6,1);
    Eigen::MatrixXd pseudoH(6,6);
    Eigen::Vector3d eulerAngles;
    /////

    int info = 0;			// return code
    int infoc = 1;		// return code for subroutine cstep

    // Compute the initial gradient in the search direction and check
    // that s is a descent direction.

    //we want to maximize s, so we should minimize -s
    //score_init = scoreNDT(sourceNDT,targetNDT);

    //gradient directions are opposite for the negated function
    //score_gradient_init = -score_gradient_init;

//  cout<<"score_init "<<score_init<<endl;
//  cout<<"score_gradient_init "<<score_gradient_init.transpose()<<endl;
//  cout<<"increment "<<increment.transpose()<<endl;

    score_gradient_here.setZero();
    score_init = derivativesNDT(sourceNDT,targetNDT,score_gradient_here,pseudoH,false);
    scg_here = score_gradient_here;
    dginit = increment.dot(scg_here);
//  cout<<"dginit "<<dginit<<endl;

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
//     cout<<"correct direction (dginit = " << dginit << ")" << endl;
    }

    // Initialize local variables.

    bool brackt = false;		// has the soln been bracketed?
    bool stage1 = true;		// are we in stage 1?
    int nfev = 0;			// number of function evaluations
    double dgtest = ftol * dginit; // f for curvature condition
    double width = stpmax - stpmin; // interval width
    double width1 = 2 * width;	// ???

    //cout<<"dgtest "<<dgtest<<endl;
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
            NDTCell<PointSource> *cell = sourceNDT[i];
            if(cell!=NULL)
            {
                Eigen::Vector3d mean = cell->getMean();
                Eigen::Matrix3d cov = cell->getCov();
                mean = ps*mean;
                cov = ps.rotation()*cov*ps.rotation().transpose();
                NDTCell<PointSource>* nd = (NDTCell<PointSource>*)cell->copy();
                nd->setMean(mean);
                nd->setCov(cov);
                sourceNDTHere.push_back(nd);
            }
        }

        double f = 0.0;
        score_gradient_here.setZero();

        /*f = scoreNDT(sourceNDT,targetNDT,ps);
        derivativesNDT(sourceNDT,targetNDT,ps,score_gradient_here,pseudoH,false);
        std::cout<<"scg1  " <<score_gradient_here.transpose()<<std::endl;
        */

        //option 2:
        //f = scoreNDT(sourceNDTHere,targetNDT);
        f = derivativesNDT(sourceNDTHere,targetNDT,score_gradient_here,pseudoH,false);
        //std::cout<<"scg2  " <<score_gradient_here.transpose()<<std::endl;


        //cout<<"incr " <<pincr.transpose()<<endl;
        //cout<<"score (f) "<<f<<endl;

        double dg = 0.0;
        scg_here = score_gradient_here;
        dg = increment.dot(scg_here);


        //VALGRIND_CHECK_VALUE_IS_DEFINED(dg);
        //cout<<"dg = "<<dg<<endl;
        nfev ++;

///////////////////////////////////////////////////////////////////////////

        //cout<<"consider step "<<stp<<endl;
        // Armijo-Goldstein sufficient decrease
        double ftest1 = finit + stp * dgtest;
        //cout<<"ftest1 is "<<ftest1<<endl;

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

        //cout<<"ftest2 "<<gtol*(-dginit)<<endl;
        //cout<<"sufficientDecrease? "<<sufficientDecreaseTest<<endl;
        //cout<<"curvature ok? "<<(fabs(dg) <= gtol*(-dginit))<<endl;
        if ((sufficientDecreaseTest) && (fabs(dg) <= gtol*(-dginit)))
            info = 1;			// Success!!!!

        if (info != 0) 		// Line search is done
        {
            if (info != 1) 		// Line search failed
            {
                // RPP add
                // counter.incrementNumFailedLineSearches();

                //if (recoveryStepType == Constant)
                stp = recoverystep;

                //newgrp.computeX(oldgrp, dir, stp);

                //message = "(USING RECOVERY STEP!)";

            }
            else 			// Line search succeeded
            {
                //message = "(STEP ACCEPTED!)";
            }

            //print.printStep(nfev, stp, finit, f, message);

            // Returning the line search flag
            //cout<<"LineSearch::"<<message<<" info "<<info<<endl;
            for(unsigned int i=0; i<sourceNDTHere.size(); i++)
            {
                if(sourceNDTHere[i]!=NULL)
                    delete sourceNDTHere[i];
            }
//      std::cout<<"nfev = "<<nfev<<std::endl;
            return stp;

        } // info != 0

        // RPP add
        //counter.incrementNumIterations();

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

            //VALGRIND_CHECK_VALUE_IS_DEFINED(dgm);
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

            //VALGRIND_CHECK_VALUE_IS_DEFINED(dg);
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

template <typename PointSource>
int NDTMatcherSequentialD2D<PointSource>::MoreThuente::cstep(double& stx, double& fx, double& dx,
        double& sty, double& fy, double& dy,
        double& stp, double& fp, double& dp,
        bool& brackt, double stmin, double stmax)
{
    int info = 0;

    // Check the input parameters for errors.

    if ((brackt && ((stp <= MoreThuente::min(stx, sty)) || (stp >= MoreThuente::max(stx, sty)))) ||
            (dx * (stp - stx) >= 0.0) || (stmax < stmin))
        return info;

    // Determine if the derivatives have opposite sign.

    double sgnd = dp * (dx / fabs(dx));

    // First case. A higher function value.  The minimum is
    // bracketed. If the cubic step is closer to stx than the quadratic
    // step, the cubic step is taken, else the average of the cubic and
    // quadratic steps is taken.

    bool bound;
    double theta;
    double s;
    double gamma;
    double p,q,r;
    double stpc, stpq, stpf;

    if (fp > fx)
    {
        info = 1;
        bound = 1;
        theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
        //VALGRIND_CHECK_VALUE_IS_DEFINED(theta);
        //VALGRIND_CHECK_VALUE_IS_DEFINED(dx);
        //VALGRIND_CHECK_VALUE_IS_DEFINED(dp);
        s = MoreThuente::absmax(theta, dx, dp);
        gamma = s * sqrt(((theta / s) * (theta / s)) - (dx / s) * (dp / s));
        if (stp < stx)
            gamma = -gamma;

        p = (gamma - dx) + theta;
        q = ((gamma - dx) + gamma) + dp;
        r = p / q;
        stpc = stx + r * (stp - stx);
        stpq = stx + ((dx / ((fx - fp) / (stp - stx) + dx)) / 2) * (stp - stx);
        if (fabs(stpc - stx) < fabs(stpq - stx))
            stpf = stpc;
        else
            stpf = stpc + (stpq - stpc) / 2;

        brackt = true;
    }

    // Second case. A lower function value and derivatives of opposite
    // sign. The minimum is bracketed. If the cubic step is closer to
    // stx than the quadratic (secant) step, the cubic step is taken,
    // else the quadratic step is taken.

    else if (sgnd < 0.0)
    {
        info = 2;
        bound = false;
        theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
        s = MoreThuente::absmax(theta,dx,dp);
        gamma = s * sqrt(((theta/s) * (theta/s)) - (dx / s) * (dp / s));
        if (stp > stx)
            gamma = -gamma;
        p = (gamma - dp) + theta;
        q = ((gamma - dp) + gamma) + dx;
        r = p / q;
        stpc = stp + r * (stx - stp);
        stpq = stp + (dp / (dp - dx)) * (stx - stp);
        if (fabs(stpc - stp) > fabs(stpq - stp))
            stpf = stpc;
        else
            stpf = stpq;
        brackt = true;
    }

    // Third case. A lower function value, derivatives of the same sign,
    // and the magnitude of the derivative decreases.  The cubic step is
    // only used if the cubic tends to infinity in the direction of the
    // step or if the minimum of the cubic is beyond stp. Otherwise the
    // cubic step is defined to be either stmin or stmax. The
    // quadratic (secant) step is also computed and if the minimum is
    // bracketed then the the step closest to stx is taken, else the
    // step farthest away is taken.

    else if (fabs(dp) < fabs(dx))
    {
        info = 3;
        bound = true;
        theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
        s = MoreThuente::absmax(theta, dx, dp);

        // The case gamma = 0 only arises if the cubic does not tend
        // to infinity in the direction of the step.

        gamma = s * sqrt(max(0,(theta / s) * (theta / s) - (dx / s) * (dp / s)));
        if (stp > stx)
            gamma = -gamma;

        p = (gamma - dp) + theta;
        q = (gamma + (dx - dp)) + gamma;
        r = p / q;
        if ((r < 0.0) && (gamma != 0.0))
            stpc = stp + r * (stx - stp);
        else if (stp > stx)
            stpc = stmax;
        else
            stpc = stmin;

        stpq = stp + (dp/ (dp - dx)) * (stx - stp);
        if (brackt)
        {
            if (fabs(stp - stpc) < fabs(stp - stpq))
                stpf = stpc;
            else
                stpf = stpq;
        }
        else
        {
            if (fabs(stp - stpc) > fabs(stp - stpq))
                stpf = stpc;
            else
                stpf = stpq;
        }
    }

    // Fourth case. A lower function value, derivatives of the same
    // sign, and the magnitude of the derivative does not decrease. If
    // the minimum is not bracketed, the step is either stmin or
    // stmax, else the cubic step is taken.

    else
    {
        info = 4;
        bound = false;
        if (brackt)
        {
            theta = 3 * (fp - fy) / (sty - stp) + dy + dp;
            s = MoreThuente::absmax(theta, dy, dp);
            gamma = s * sqrt(((theta/s)*(theta/s)) - (dy / s) * (dp / s));
            if (stp > sty)
                gamma = -gamma;
            p = (gamma - dp) + theta;
            q = ((gamma - dp) + gamma) + dy;
            r = p / q;
            stpc = stp + r * (sty - stp);
            stpf = stpc;
        }
        else if (stp > stx)
            stpf = stmax;
        else
            stpf = stmin;
    }

    // Update the interval of uncertainty. This update does not depend
    // on the new step or the case analysis above.

    if (fp > fx)
    {
        sty = stp;
        fy = fp;
        dy = dp;
    }
    else
    {
        if (sgnd < 0.0)
        {
            sty = stx;
            fy = fx;
            dy = dx;
        }
        stx = stp;
        fx = fp;
        dx = dp;
    }

    // Compute the new step and safeguard it.

    stpf = MoreThuente::min(stmax, stpf);
    stpf = MoreThuente::max(stmin, stpf);
    stp = stpf;
    if (brackt && bound)
    {
        if (sty > stx)
            stp = min(stx + 0.66 * (sty - stx), stp);
        else
            stp = max(stx + 0.66 * (sty - stx), stp);
    }

    return info;

}

template <typename PointSource>
double NDTMatcherSequentialD2D<PointSource>::MoreThuente::min(double a, double b)
{
    return (a < b ? a : b);
}

template <typename PointSource>
double NDTMatcherSequentialD2D<PointSource>::MoreThuente::max(double a, double b)
{
    return (a > b ? a : b);
}

template <typename PointSource>
double NDTMatcherSequentialD2D<PointSource>::MoreThuente::absmax(double a, double b, double c)
{
    a = fabs(a);
    b = fabs(b);
    c = fabs(c);

    if (a > b)
        return (a > c) ? a : c;
    else
        return (b > c) ? b : c;
}

template <typename PointSource>
double NDTMatcherSequentialD2D<PointSource>::normalizeAngle(double a)
{
    //set the angle between -M_PI and M_PI
    return atan2(sin(a), cos(a));

}



}
