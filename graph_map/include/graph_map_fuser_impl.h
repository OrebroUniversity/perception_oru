namespace libgraphMap{

/*!
 * \brief graphMapFuser::ProcessFrame
 * \param cloud frame: lidar
 * \param Tnow frame: world
 */
template<class PointT>
void GraphMapFuser::ProcessFrame(pcl::PointCloud<PointT> &cloud, Eigen::Affine3d &Tnow, const Eigen::Affine3d &Tmotion){
  //plotGTCloud(cloud);
  bool map_node_created,registration_succesfull=true,fuse_this_frame=false;
  static bool map_node_changed=false;
  Tnow=Tnow*Tmotion;
  if(!initialized_){
    cerr<<"fuser not initialized"<<endl;
    return;
  }
  fuse_this_frame=KeyFrameBasedFuse(Tnow);//fuse frame based on distance traveled
  Eigen::Affine3d T_world_to_local_map=graph_map_->GetCurrentNodePose().inverse(); //transformation from node to world frame
  Tnow=T_world_to_local_map*Tnow;//change frame to local map

  Matrix6d motion_cov=motion_model_2d_.getCovMatrix6(Tmotion, 1., 1., 1.);
  lslgeneric::transformPointCloudInPlace(sensorPose_, cloud);//Transform cloud into robot frame before registrating
  if(fuse_this_frame||map_node_changed){
    registration_succesfull = registrator_->Register<PointT>(graph_map_->GetCurrentNode()->GetMap(),Tnow,cloud,motion_cov);//Tnow will be updated to the actual pose of the robot according to ndt-d2d registration

  }

  if(graph_map_->AutomaticMapInterchange(Tnow,motion_cov,T_world_to_local_map,map_node_changed,map_node_created) && map_node_changed)
  {
    //double score;
    //Affine3d Tdiff=Graph_nav_->GetPreviousNodePose().inverse()*Graph_nav_->GetCurrentNodePose();
    //Tdiff.translation()=Tdiff.translation()+Vector3d(0.1,0.1,0.1);
    //cout<<"Tdiff with offset=\n"<<Tdiff.translation()<<endl;
    //registrator_->RegisterMap2Map(Graph_nav_->GetPreviousNode()->GetMap(),Graph_nav_->GetCurrentNode()->GetMap(),Tdiff,score);
    //Graph_nav_->AddFactor(Graph_nav_->GetPreviousNode(),Graph_nav_->GetCurrentNode(),Tdiff,unit_covar);
  }
  if(!registration_succesfull){
    Tnow=T_world_to_local_map.inverse()*Tnow;//remap Tnow to global map frame
    cerr<<"REGISTRATION ERROR"<<endl;
    nr_frames_++;
    return;
  }
  if(fuse_this_frame||map_node_changed){
    lslgeneric::transformPointCloudInPlace(Tnow, cloud);// The cloud should now be centered around the robot pose in the map frame
    graph_map_->GetCurrentNode()->updateMap/*<PointT>*/(Tnow*sensorPose_,cloud);//Update map, provided transform is the pose of the sensor in the world which is where the scan was taken from
  }

  Tnow=T_world_to_local_map.inverse()*Tnow;//remap Tnow to global map frame
  if(fuse_this_frame||map_node_changed)
    pose_last_fuse_=Tnow;

  nr_frames_++;

}

}
