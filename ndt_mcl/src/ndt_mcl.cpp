#include <ndt_mcl/ndt_mcl.h>

void NDTMCL::updateAndPredict(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud){
    Eigen::Vector3d tr = Tmotion.translation();
    Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);


    /** Arla Motion model
      if(rot[2]<(0.5 * M_PI/180.0) && tr[0]>=0){ 
      pf.predict(mcl::pose(tr[0],tr[1],rot[2]), mcl::pose(tr[0]*0.2 + 0.005,tr[1]*0.1+ 0.005,rot[2]*0.2+0.001));
      }else if(tr[0]>=0){
      pf.predict(mcl::pose(tr[0],tr[1],rot[2]), mcl::pose(tr[0] + 0.01,tr[1] + 0.01,rot[2]*0.5+0.001));
      }else{
      pf.predict(mcl::pose(tr[0],tr[1],rot[2]), mcl::pose(tr[0] + 0.02,tr[1] + 0.01,rot[2]*0.8+0.001));
      }
     **/
    pf.predict(mcl::pose(tr[0],tr[1],rot[2]), mcl::pose(tr[0]*0.1 + 0.005,tr[1]*0.1+ 0.005,rot[2]*0.1+0.001));

    lslgeneric::NDTMap local_map(new lslgeneric::LazyGrid(resolution));

    /*
       pcl::PointCloud<PointT> cl_f;
       pcl::PointCloud<PointT> cl_z;
       for(int i=0;i<cloud.size();i++){
       if(cloud.points[i].z > 1.95 && cloud.points[i].z <2.05 ) cl_z.push_back(cloud.points[i]);
       if(cloud.points[i].z > zfilt_min ) cl_f.push_back(cloud.points[i]);
       }


       fprintf(stderr,"2D scan = %d Points\n",cl_z.size());

     */
    //fprintf(stderr,"Could = %d Points\n",cloud.size());
    local_map.addPointCloudSimple(cloud);
    //local_map.computeNDTCells();
    local_map.computeNDTCells(CELL_UPDATE_MODE_STUDENT_T);
    int Nn = 0;
    //			#pragma omp parallel for
    for(int i=0;i<pf.NumOfParticles;i++){
	Eigen::Affine3d T = getAsAffine(i);

	std::vector<lslgeneric::NDTCell*> ndts;
	ndts = local_map.pseudoTransformNDT(T);
	double score=1;

	if(ndts.size()==0) fprintf(stderr,"ERROR no gaussians in measurement!!!\n");
	Nn = ndts.size();
	for(int n=0;n<ndts.size();n++){
	    Eigen::Vector3d m = ndts[n]->getMean();	
	    if(m[2]<zfilt_min) continue;

	    lslgeneric::NDTCell *cell;
	    pcl::PointXYZ p;
	    p.x = m[0];p.y=m[1];p.z=m[2];

	    //if(map.getCellAtPoint(p,cell)){
	    if(map.getCellForPoint(p,cell)){
		if(cell == NULL) continue;
		if(cell->hasGaussian_){
		    Eigen::Matrix3d covCombined = cell->getCov() + ndts[n]->getCov();
		    Eigen::Matrix3d icov;
		    bool exists;
		    double det = 0;
		    covCombined.computeInverseAndDetWithCheck(icov,det,exists);
		    if(!exists) continue;
		    double l = (cell->getMean() - m).dot(icov*(cell->getMean() - m));
		    if(l*0 != 0) continue;
		    //if(l > 120) continue;
		    score += 0.1 + 0.9 * exp(-0.05*l/2.0);
		}else{
		}
	    }
	}

	/*  -lfd1*(exp(-lfd2*l/2));*/

	pf.Particles[i].lik = score;
	for(unsigned int j=0;j<ndts.size();j++){
	    delete ndts[j];
	}

	}

	pf.normalize();


	if(forceSIR){

	    fprintf(stderr, "forceSIR(%d) ",forceSIR);
	    pf.SIRUpdate();
	}else{

	    double varP=0;
	    for(int i = 0; i<pf.NumOfParticles;i++){
		varP += (pf.Particles[i].p - 1.0/pf.NumOfParticles)*(pf.Particles[i].p - 1.0/pf.NumOfParticles);
	    }
	    varP /= pf.NumOfParticles;
	    varP = sqrt(varP); 
	    fprintf(stderr,"Var P=%lf (Npf=%d, Nm=%d)",varP,pf.NumOfParticles, Nn);
	    if(varP > 0.006 || sinceSIR >25){
		fprintf(stderr,"-SIR- ");
		sinceSIR = 0;
		pf.SIRUpdate();
	    }else{
		sinceSIR++;
	    }

	}
    }
