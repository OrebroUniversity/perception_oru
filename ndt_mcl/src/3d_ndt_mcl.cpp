#include <ndt_mcl/3d_ndt_mcl.h>

void NDTMCL3D::updateAndPredict(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud){
    Eigen::Vector3d tr = Tmotion.translation();
    Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);

    double t_start = getDoubleTime();
    pf.predict(Tmotion, tr[0]*0.1, tr[1]*0.1, tr[2]*0.1, rot[0]*0.1, rot[1]*0.1, rot[2]*0.1);
    double t_pred = getDoubleTime() - t_start;	

    //pf.predict(mcl::pose(tr[0],tr[1],rot[2]), mcl::pose(tr[0]*0.1 + 0.005,tr[1]*0.1+ 0.005,rot[2]*0.1+0.001));

    lslgeneric::NDTMap local_map(new lslgeneric::LazyGrid(resolution_sensor));
    std::cerr<<"cloud points "<<cloud.points.size()<<std::endl;
    local_map.addPointCloudSimple(cloud);
    local_map.computeNDTCells();
    //local_map.computeNDTCells(CELL_UPDATE_MODE_STUDENT_T);
    int Nn = 0;
    //		#pragma omp parallel for
    double t_pseudo = 0;

    for(int i=0;i<pf.size();i++){
	Eigen::Affine3d T = pf.pcloud[i].T;

	std::vector<lslgeneric::NDTCell*> ndts;
	double tictime = getDoubleTime();
	ndts = local_map.pseudoTransformNDT(T);
	t_pseudo += getDoubleTime()-tictime;
	double score=1;

	if(ndts.size()==0) fprintf(stderr,"ERROR no gaussians in measurement!!!\n");
	Nn = ndts.size();

	for(int n=0;n<ndts.size();n++){
	    Eigen::Vector3d m = ndts[n]->getMean();	
	    if(m[2]<zfilt_min) continue;

	    lslgeneric::NDTCell *cell;
	    pcl::PointXYZ p;
	    p.x = m[0];p.y=m[1];p.z=m[2];

	    if(map.getCellAtPoint(p,cell)){
		//if(map.getCellForPoint(p,cell)){
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
		    score += 0.1 + 0.9 * exp(-0.05*l/2.0);
		}else{
		}
	    }
	    }

	    pf.pcloud[i].lik = score;
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
	    for(int i = 0; i<pf.size();i++){
		varP += (pf.pcloud[i].p - 1.0/pf.size())*(pf.pcloud[i].p - 1.0/pf.size());
	    }
	    varP /= pf.size();
	    varP = sqrt(varP); 
	    fprintf(stderr,"Var P=%lf (Npf=%d, Nm=%d) (t_pred = %.3lf t_pseudo=%.3lf)",varP,pf.size(), Nn, t_pred,t_pseudo);
	    if(varP > /*0.006*/SIR_varP_threshold || sinceSIR > /*25*/SIR_max_iters_wo_resampling){
		fprintf(stderr,"-SIR- ");
		sinceSIR = 0;
		pf.SIRUpdate();
	    }else{
		sinceSIR++;
	    }

	}
    }

void NDTMCL3D::predict(Eigen::Affine3d Tmotion) {
   Eigen::Vector3d tr = Tmotion.translation();
    Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);


    Eigen::Matrix<double, 6,6> motion_model_m(motion_model.data());
    
    Eigen::Matrix<double,6,1> incr;
    incr << fabs(tr[0]),fabs(tr[1]),fabs(tr[2]), fabs(rot[0]), fabs(rot[1]), fabs(rot[2]);
    Eigen::Matrix<double,6,1> m = motion_model_m*incr; 

    // std::cerr << "incr : " << incr.transpose() << std::endl;
    // std::cerr << "motion var : " << m.transpose() << std::endl;
    
    for (size_t i = 0; i < motion_model_offset.size(); i++) {
      m[i] += motion_model_offset[i];
    }

    // std::cerr << "motion var(2) : " << m.transpose() << std::endl;
    
    pf.predict(Tmotion, 
               m[0], m[1], m[2], m[3], m[4], m[5]);
} 

void NDTMCL3D::updateAndPredictEff(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, double subsample_level){
    if(subsample_level < 0 || subsample_level > 1) subsample_level = 1;

    Eigen::Vector3d tr = Tmotion.translation();
    Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);

    double time_start = getDoubleTime();
    
    // pf.predict(Tmotion, 
    //            fabs(tr[0])*motion_model[0] + motion_model_offset[1], 
    //            fabs(tr[1])*motion_model[1] + motion_model_offset[2], 
    //            fabs(tr[2])*motion_model[2] + motion_model_offset[3],
    //            fabs(rot[0])*motion_model[5]+ motion_model_offset[4],
    //            fabs(rot[1])*motion_model[6]+ motion_model_offset[5],
    //            fabs(rot[2])*motion_model[7]+ motion_model_offset[6]);

    
    Eigen::Matrix<double, 6,6> motion_model_m(motion_model.data());
    
    Eigen::Matrix<double,6,1> incr;
    incr << fabs(tr[0]),fabs(tr[1]),fabs(tr[2]), fabs(rot[0]), fabs(rot[1]), fabs(rot[2]);
    Eigen::Matrix<double,6,1> m = motion_model_m*incr; 

    // std::cerr << "incr : " << incr.transpose() << std::endl;
    // std::cerr << "motion var : " << m.transpose() << std::endl;
    
    for (size_t i = 0; i < motion_model_offset.size(); i++) {
      m[i] += motion_model_offset[i];
    }

    // std::cerr << "motion var(2) : " << m.transpose() << std::endl;
    
    pf.predict(Tmotion, 
               m[0], m[1], m[2], m[3], m[4], m[5]);


    // if(rot[2]<(0.5 * M_PI/180.0) && tr[0]>=0){ 
    //     pf.predict(Tmotion, tr[0]*pos_factor[0] + pos_offset, tr[1]*pos_scale[1] + pos_offset[1], tr[2]*pos_factor[2]/2.+pos_offset[2] ,rot[0]*rot_factor[0]+rot_offset[0],rot[1]*rot_factor[1]+rot_offset[1], rot[2]*rot_factor[2]+rot_offset[2]);
    // }else if(tr[0]>=0){
    //   pf.predict(Tmotion,tr[0]*tr_scale*2.5 + tr_offset, tr[1]*tr_scale/2.+ tr_offset, tr[2]*tr_scale/2.+tr_offset,rot[0]*rot_scale+rot_offset,rot[1]*rot_scale+rot_offset, rot[2]*rot_scale*2+rot_offset);
    // }else{
    //     pf.predict(Tmotion, tr[0]*tr_scale + tr_offset, tr[1]*tr_scale / 2.+ tr_offset, tr[2]*tr_scale+tr_offset ,rot[0]*rot_scale+rot_offset,rot[1]*rot_scale+rot_offset, rot[2]*rot_scale+rot_offset);
    // }


    double t_pred = getDoubleTime() - time_start;	

    std::cerr<<"cloud points "<<cloud.points.size()<<" res :"<<resolution<<" sres: "<<resolution_sensor<<std::endl;
    lslgeneric::NDTMap local_map(new lslgeneric::LazyGrid(resolution_sensor));
    //local_map.guessSize(0,0,0,30,30,10); //sensor_range,sensor_range,map_size_z);
    local_map.loadPointCloud(cloud);//,30); //sensor_range);
    local_map.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

    /*lslgeneric::NDTMap<PointT> local_map(new lslgeneric::LazyGrid<PointT>(resolution_sensor));		
      local_map.addPointCloudSimple(cloud);
    //local_map.computeNDTCells();
    local_map.computeNDTCellsSimple();
     */
    std::vector<lslgeneric::NDTCell*> ndts0 = local_map.getAllCells();
    std::vector<lslgeneric::NDTCell*> ndts;
    std::cerr<<"ndts: "<<ndts0.size()<<std::endl;

    if(subsample_level != 1) {
	srand((int)(t_pred*10000));
	for(int i=0; i<ndts0.size(); ++i) {
	    double p = ((double)rand())/RAND_MAX;
	    if(p < subsample_level) {
		ndts.push_back(ndts0[i]);
	    } else {
		delete ndts0[i];
	    }
	}	
    } else {
	ndts = ndts0;
    }
    std::cerr<<"resampled ndts: "<<ndts.size()<<std::endl;

    int Nn = 0;
    //		#pragma omp parallel for
    double t_pseudo = getDoubleTime();
#pragma omp parallel num_threads(4)
    {
#pragma omp for
	for(int i=0;i<pf.size();i++){
	    Eigen::Affine3d T = pf.pcloud[i].T;


	    //ndts = local_map.pseudoTransformNDT(T);
	    double score=1;

	    if(ndts.size()==0) fprintf(stderr,"ERROR no gaussians in measurement!!!\n");
	    Nn = ndts.size();

	    for(int n=0;n<ndts.size();n++){
		Eigen::Vector3d m = T*ndts[n]->getMean();	

		if(m[2]<zfilt_min) continue;

		lslgeneric::NDTCell *cell;
		pcl::PointXYZ p;
		p.x = m[0];p.y=m[1];p.z=m[2];

		if(map.getCellAtPoint(p,cell)){
		    //if(map.getCellForPoint(p,cell)){
		    if(cell == NULL) continue;
		    if(cell->hasGaussian_){
			Eigen::Matrix3d covCombined = cell->getCov() + T.rotation()*ndts[n]->getCov() *T.rotation().transpose();
			Eigen::Matrix3d icov;
 			bool exists;
			double det = 0;
			covCombined.computeInverseAndDetWithCheck(icov,det,exists);
			if(!exists) continue;
			double l = (cell->getMean() - m).dot(icov*(cell->getMean() - m));
			if(l*0 != 0) continue;
			score += 0.1 + 0.9 * exp(-0.05*l/2.0);
		    }else{
		    }
		}
		}

		pf.pcloud[i].lik = score;


	    }
	}///#pragma

    t_pseudo = getDoubleTime() - t_pseudo;
	for(unsigned int j=0;j<ndts.size();j++){
	    delete ndts[j];
	}


	pf.normalize();


	if(forceSIR){	
	    fprintf(stderr, "forceSIR(%d) ",forceSIR);
	    pf.SIRUpdate();
	}else{

	    double varP=0;
	    for(int i = 0; i<pf.size();i++){
		varP += (pf.pcloud[i].p - 1.0/pf.size())*(pf.pcloud[i].p - 1.0/pf.size());
	    }
	    varP /= pf.size();
	    varP = sqrt(varP); 
	    fprintf(stderr,"Var P=%lf (Npf=%d, Nm=%d) (t_pred = %.3lf t_pseudo=%.3lf)",varP,pf.size(), Nn, t_pred,t_pseudo);
	    if(varP > /*0.006*/SIR_varP_threshold || sinceSIR > /*25*/SIR_max_iters_wo_resampling){
		fprintf(stderr,"-SIR- ");
		sinceSIR = 0;
		pf.SIRUpdate();
	    }else{
		sinceSIR++;
	    }

	}
    }

