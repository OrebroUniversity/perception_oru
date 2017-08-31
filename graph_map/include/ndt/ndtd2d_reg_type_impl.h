namespace libgraphMap{

template<class PointT>
bool NDTD2DRegType::Register(MapTypePtr maptype,Eigen::Affine3d &Tnow,pcl::PointCloud<PointT> &cloud,Matrix6d cov) {

  if(!enableRegistration_||!maptype->Initialized()){
    cout<<"Registration disabled - motion based on odometry"<<endl;
    return true;
  }
  cout<<" registration ";
  ///Create local map
  lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(resolution_*resolutionLocalFactor_));
  ndlocal.guessSize(0,0,0,sensorRange_,sensorRange_,mapSizeZ_);
  ndlocal.loadPointCloud(cloud,sensorRange_);
  ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
  Eigen::Affine3d Tinit = Tnow;//registration prediction
  //NDTMap * ptrmap=&ndlocal;
  //graphPlot::SendLocalMapToRviz(ptrmap,0,sensorPose_);

  //Get ndt map pointer
  NDTMapPtr MapPtr = boost::dynamic_pointer_cast< NDTMapType >(maptype);
  NDTMap *globalMap=MapPtr->GetNDTMap();
  // cout<<"number of cell in (global/local) map"<<globalMap->getAllCells().size()<<","<<ndlocal.getAllCells().size()<<endl;
  bool matchSuccesfull;
  if(registration2d_){
    matchSuccesfull=matcher2D_.match(*globalMap, ndlocal,Tinit,true);
  }
  else if(!registration2d_){
    matchSuccesfull=matcher3D_.match( *globalMap, ndlocal,Tinit,true);
  }

  if(matchSuccesfull){//if succesfull match, make sure the registration is consistent
    Eigen::Affine3d diff = Tnow.inverse()*Tinit;//difference between prediction and registration
    Vector3d diff_angles=Vector3d(diff.rotation().eulerAngles(0,1,2));
    Eigen::AngleAxisd diff_rotation_(diff.rotation());


    if(checkConsistency_ && diff.translation().norm() > maxTranslationNorm_ ){
      cerr<<"registration failure: Translation too high"<<endl;
      cerr<<"movement="<<diff.translation().norm()<<"m  >  "<<diff.translation().norm()<<endl;
      return false;
    }
    else if(checkConsistency_ &&(diff_rotation_.angle() > maxRotationNorm_) ){
      cerr<<"registration failure: Rotation too high"<<endl;
      cerr<<"movement="<<diff_rotation_.angle()<<"rad  >  "<<maxRotationNorm_<<endl;
      //Tnow = Tnow * Tmotion;
      return false;
    }
    else{
      Tnow = Tinit;//return registered value
      return true;
    }
  }
  else{
    cerr<<"Registration unsuccesfull"<<endl;
    return false;
  }
}

}
