#include "mcl_ndt/mcl_ndt.h"
#include <ndt_dl/point_curv.h>
namespace GraphMapLocalisation{
MCLNDTType::MCLNDTType(LocalisationParamPtr param):LocalisationType(param){
  if(MCLNDTParamPtr mclParam=boost::dynamic_pointer_cast<MCLNDTParam>(param)){


//    NDTMapPtr current_map = boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentNode()->GetMap());
//    if (current_map != NULL) {
//      resolution=boost::dynamic_pointer_cast<NDTMapType>(graph_map_->GetCurrentNode()->GetMap())->GetResolution();
//    }
//    else {
//      ROS_INFO_STREAM("Not a NDTMapType used(!)");
//    }

    resolution=mclParam->resolution;
    forceSIR=mclParam->forceSIR;
    motion_model=mclParam->motion_model;
    motion_model_offset=mclParam->motion_model_offset;
    n_particles_=mclParam->n_particles_;
    SIR_varP_threshold=mclParam->SIR_varP_threshold;
    counter=0;
    z_filter_min=mclParam->z_filter_min;
    score_cell_weight=mclParam->score_cell_weight;
    initialized_ = false;
    forceSIR = false;
    resolution_sensor=resolution;
      ROS_ERROR("created MCLNDTType!");
  }
  else{
    std::cerr<<"Cannot create MCLNDType. Illegal type for parameter param"<<endl;
    exit(0);
  }
}

NDTMap* MCLNDTType::GetCurrentNodeNDTMap() {
  // Currently exist two different maps of NDT, NDT and NDTDL, find out which we have in the graph and return it...
  {
    NDTMapPtr p = boost::dynamic_pointer_cast< NDTMapType >(graph_map_->GetCurrentNode()->GetMap());
    if (p != NULL) {
      return p->GetNDTMap();
    }
  }

  {
    NDTDLMapPtr p = boost::dynamic_pointer_cast< NDTDLMapType >(graph_map_->GetCurrentNode()->GetMap());
    return p->GetNDTMapFlat();
  }
  return NULL;
}

void MCLNDTType::UpdateAndPredict(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tmotion){
  counter++;
  if(counter%5==0)
    GraphPlot::plotParticleCloud(graph_map_->GetCurrentNodePose(),pf.pcloud);

  pose_=graph_map_->GetCurrentNodePose()*pf.getMean();
  bool new_ref_frame=false;
  graph_map_->SwitchToClosestMapNode(pose_,new_ref_frame); //frame was changed

  Eigen::Vector3d tr = Tmotion.translation();
  Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);
  //cout<<"Tmotion="<<Tmotion.translation()<<endl;
  double time_start = getDoubleTime();

  Eigen::Matrix<double, 6,6> motion_model_m(motion_model.data());
  Eigen::Matrix<double,6,1> incr;
  incr << fabs(tr[0]),fabs(tr[1]),fabs(tr[2]), fabs(rot[0]), fabs(rot[1]), fabs(rot[2]);
  Eigen::Matrix<double,6,1> m = motion_model_m*incr;

  // std::cerr << "incr : " << incr.transpose() << std::endl;
  // std::cerr << "motion var : " << m.transpose() << std::endl;

  for (size_t i = 0; i < motion_model_offset.size(); i++) {
    m[i] += motion_model_offset[i];
  }
  pose_=graph_map_->GetCurrentNodePose()*pf.getMean();
  // std::cerr << "motion var(2) : " << m.transpose() << std::endl;
  Eigen::Affine3d remap_prev_to_new_frame;
  if(new_ref_frame){
    remap_prev_to_new_frame=(graph_map_->GetPreviousNodePose().inverse()*graph_map_->GetCurrentNodePose()).inverse();
    cout<<"change frame, transf="<<remap_prev_to_new_frame.translation().transpose()<<endl;
    pf.predict(Tmotion,m[0], m[1], m[2], m[3], m[4], m[5],remap_prev_to_new_frame);
    //map_= boost::dynamic_pointer_cast< NDTMapType >(graph_map_->GetCurrentNode()->GetMap())->GetNDTMap();
    map_ = this->GetCurrentNodeNDTMap();
  }
  else
    pf.predict(Tmotion,m[0], m[1], m[2], m[3], m[4], m[5]);


  // if(rot[2]<(0.5 * M_PI/180.0) && tr[0]>=0){
  //     pf.predict(Tmotion, tr[0]*pos_factor[0] + pos_offset, tr[1]*pos_scale[1] + pos_offset[1], tr[2]*pos_factor[2]/2.+pos_offset[2] ,rot[0]*rot_factor[0]+rot_offset[0],rot[1]*rot_factor[1]+rot_offset[1], rot[2]*rot_factor[2]+rot_offset[2]);
  // }else if(tr[0]>=0){
  //   pf.predict(Tmotion,tr[0]*tr_scale*2.5 + tr_offset, tr[1]*tr_scale/2.+ tr_offset, tr[2]*tr_scale/2.+tr_offset,rot[0]*rot_scale+rot_offset,rot[1]*rot_scale+rot_offset, rot[2]*rot_scale*2+rot_offset);
  // }else{
  //     pf.predict(Tmotion, tr[0]*tr_scale + tr_offset, tr[1]*tr_scale / 2.+ tr_offset, tr[2]*tr_scale+tr_offset ,rot[0]*rot_scale+rot_offset,rot[1]*rot_scale+rot_offset, rot[2]*rot_scale+rot_offset);
  // }

  double t_pred = getDoubleTime() - time_start;

  //std::cerr<<"cloud points "<<cloud.points.size()<<" res :"<<resolution<<" sres: "<<resolution_sensor<<std::endl;
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

  if(subsample_level_ != 1) {
    srand((int)(t_pred*10000));
    for(int i=0; i<ndts0.size(); ++i) {
      double p = ((double)rand())/RAND_MAX;
      if(p < subsample_level_) {
        ndts.push_back(ndts0[i]);
      } else {
        delete ndts0[i];
      }
    }
  } else {
    ndts = ndts0;
  }
//  std::cerr<<"resampled ndts: "<<ndts.size()<<std::endl;

  int Nn = 0;
  //		#pragma omp parallel for
  double t_pseudo = getDoubleTime();
#pragma omp parallel num_threads(8)
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
         if(m[2]<z_filter_min) continue;

        lslgeneric::NDTCell *cell;
        pcl::PointXYZ p;
        p.x = m[0];p.y=m[1];p.z=m[2];


        if(map_->getCellAtPoint(p,cell)){
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
            score += score_cell_weight + (1.0 - score_cell_weight) * exp(-0.05*l/2.0);
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
    //fprintf(stderr, "forceSIR(%d) ",forceSIR);
    pf.SIRUpdate();
  }else{

    double varP=0;
    for(int i = 0; i<pf.size();i++){
      varP += (pf.pcloud[i].p - 1.0/pf.size())*(pf.pcloud[i].p - 1.0/pf.size());
    }
    varP /= pf.size();
    varP = sqrt(varP);
    //fprintf(stderr,"Var P=%lf (Npf=%d, Nm=%d) (t_pred = %.3lf t_pseudo=%.3lf) itr since SIR= %d\n",varP,pf.size(), Nn, t_pred,t_pseudo,sinceSIR_);
    if(varP > /*0.006*/SIR_varP_threshold || sinceSIR_ > /*25*/SIR_max_iters_wo_resampling_){
      //fprintf(stderr,"-SIR- ");
      sinceSIR_ = 0;
      pf.SIRUpdate();
    }else{
      sinceSIR_++;
    }

  }
  pose_=graph_map_->GetCurrentNodePose()*pf.getMean();

  GraphPlot::PlotMap(local_map,1,pose_,plotmarker::sphere/*plotmarker::point*/, "ndtmcl");

}


void MCLNDTType::UpdateAndPredict(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud, const Eigen::Affine3d &Tmotion){

  // Filter out the flat parts...
  pcl::PointCloud<pcl::PointXYZ> cornerPointsSharp;
  pcl::PointCloud<pcl::PointXYZ> cornerPointsLessSharp;
  pcl::PointCloud<pcl::PointXYZ> surfPointsFlat;
  pcl::PointCloud<pcl::PointXYZ> surfPointsLessFlat;

  //segmentPointCurvature(cloud, cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat);

  //UpdateAndPredict(surfPointsLessFlat, Tmotion);

  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  pcl::copyPointCloud(cloud, cloud_xyz);
  UpdateAndPredict(cloud_xyz, Tmotion);
}



std::string MCLNDTType::ToString(){

  std::stringstream ss;
  ss<<LocalisationType::ToString();
  ss<<graph_map_->ToString();
  ss<<"NDTMCL:"<<endl;
  ss<<"number of particles: "<<n_particles_<<endl;
  ss<<"resolution: "<<resolution<<endl;
  ss<<"force SIR: "<<std::boolalpha<<forceSIR<<endl;
  ss<<"SIR var Probability threshold: "<<SIR_varP_threshold<<endl;
  ss<<"motion model: ";
  for (int i = 0; i < motion_model.size(); i++) { if (i % 6 == 0) {ss << endl;} ss<<motion_model[i] << " " << endl; }
  ss<<"motion model offset: ";
  for (int i = 0; i < motion_model_offset.size(); i++) { if (i % 6 == 0) {ss << endl;}; ss<<motion_model_offset[i] << " " << endl; }
  return ss.str();
}

void MCLNDTType::InitializeLocalization(const Eigen::Affine3d &pose,const Vector6d &variance){ //Vector3d variance={Vx, Vy, Vz Vp Vr Vy}
  pose_=pose;
  graph_map_->SwitchToClosestMapNode(pose_);//specified in local frame, should be specified in gloval
  //map_= boost::dynamic_pointer_cast< NDTMapType >(graph_map_->GetCurrentNode()->GetMap())->GetNDTMap();
  map_ = this->GetCurrentNodeNDTMap();
  Eigen::Vector3d pos=pose_.translation();
  Eigen::Vector3d euler = pose_.rotation().eulerAngles(0,1,2);
  normalizeEulerAngles(euler);
  pf.initializeNormalRandom(n_particles_, pos(0),pos(1),pos(2),euler(0),euler(1), euler(2), variance(0),variance(1),variance(2),variance(3),variance(4),variance(5));
  initialized_=true;
}
double MCLNDTType::getDoubleTime()
{
  struct timeval time;
  gettimeofday(&time,NULL);
  return time.tv_sec + time.tv_usec * 1e-6;
}

void MCLNDTType::normalizeEulerAngles(Eigen::Vector3d &euler) {
  euler[0] = angles::normalize_angle(euler[0]);
  euler[1] = angles::normalize_angle(euler[1]);
  euler[2] = angles::normalize_angle(euler[2]);

  if (fabs(euler[0]) > M_PI/2) {
    euler[0] += M_PI;
    euler[1] = -euler[1] + M_PI;
    euler[2] += M_PI;

    euler[0] = angles::normalize_angle(euler[0]);
    euler[1] = angles::normalize_angle(euler[1]);
    euler[2] = angles::normalize_angle(euler[2]);
  }
}

MCLNDTParam::MCLNDTParam(){

  motion_model.push_back(0.05);
  motion_model.push_back(0.05);
  motion_model.push_back(0.02);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.02);

  motion_model.push_back(0.05);
  motion_model.push_back(0.1);
  motion_model.push_back(0.02);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.02);


  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.1);
  motion_model.push_back(0.001);
  motion_model.push_back(0.001);
  motion_model.push_back(0.001);

  motion_model.push_back(0.001);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.1);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);

  motion_model.push_back(0.01);
  motion_model.push_back(0.001);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.1);
  motion_model.push_back(0.01);

  motion_model.push_back(0.1);
  motion_model.push_back(0.01);
  motion_model.push_back(0.001);
  motion_model.push_back(0.01);
  motion_model.push_back(0.01);
  motion_model.push_back(0.1);

  /*motion_model_offset.push_back(0.00);
  motion_model_offset.push_back(0.002);
  motion_model_offset.push_back(0.0000001);//0.002
  motion_model_offset.push_back(0.0000001);//0.001
  motion_model_offset.push_back(0.0000001);//0.001
  motion_model_offset.push_back(0.001);*/

  motion_model_offset.push_back(0.005);
  motion_model_offset.push_back(0.005);
  motion_model_offset.push_back(0.0000001);//0.002
  motion_model_offset.push_back(0.0000001);//0.001
  motion_model_offset.push_back(0.0000001);//0.001
  motion_model_offset.push_back(0.003);
}
}
