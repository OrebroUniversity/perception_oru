#include <ndt_evaluation/registration_score_eval.h>
#include <ndt_generic/serialization.h>
//#include <graph_map/graph_map.h>


RegistrationScoreEval::RegistrationScoreEval(const std::string &gt_file,
                                             const std::string &odom_file,
                                             const std::string &base_name_pcd,
                                             const Eigen::Affine3d &sensor_pose,
                                             const lslgeneric::MotionModel3d::Params &motion_params) : base_name_pcd_(base_name_pcd), sensor_pose_(sensor_pose) {


  std::cout << "loading: " << gt_file << std::endl;
  Tgt = ndt_generic::loadAffineFromEvalFile(gt_file);
  std::cout << " got # poses : " << Tgt.size() << std::endl;
  std::cout << "loading: " << odom_file << std::endl;
  Todom = ndt_generic::loadAffineFromEvalFile(odom_file);
  std::cout << " got #poses : " << Todom.size() << std::endl;

  motion_model.setParams(motion_params);
  resolution = 1.;
  alpha_ = 1.;

  offset_size = 100;
  incr_dist = 0.01;
  incr_ang = 0.002;

  T_glb_d2d_.setIdentity();
  T_glb_d2d_sc_.setIdentity();
  T_glb_icp_.setIdentity();
  T_glb_filter_.setIdentity();
  T_glb_icp_filter_.setIdentity();
  T_glb_gt_.setIdentity();
  T_glb_odom_.setIdentity();
}

bool
RegistrationScoreEval::setGlobalMap(const std::string &file_name, const Eigen::Affine3d &T) {
    try {
      std::ifstream ifs(file_name.c_str());
      boost::archive::text_iarchive ia(ifs);
      ia & global_ndtmap_;
      global_ndtmap_T_ = T;
      return true;
    }
  catch (std::exception const& e)
    {
      std::cerr << "Error: " << e.what() << std::endl;
      return false;
  }

}

bool
RegistrationScoreEval::setGlobalGraphMap(const std::string &file_name, int node_idx) {
  try {
    std::ifstream ifs(file_name.c_str());
    boost::archive::text_iarchive ia(ifs);

//    libgraphMap::GraphMap graph_map;

//    ia & graph_map;
//    global_ndtmap_T_ = graph_map.;
    return true;
  }
catch (std::exception const& e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
    return false;
}

}


std::vector<Eigen::Affine3d> RegistrationScoreEval::generateOffsetSet() {

  std::vector<Eigen::Affine3d> ret;
  std::vector<double> incr(6);
  incr.resize(6);

  for (int i = 0; i < 3; i++) {
    incr[i] = incr_dist;
    incr[i+3] = incr_ang;
  }

  Eigen::VectorXd offset(6);
  for (int j = 0; j < 6; j++) {
    offset.setZero();
    for (int i = -offset_size; i <= offset_size; i++) {
      offset[j] = i*incr[j];
      ret.push_back(ndt_generic::vectorToAffine3d(offset));
    }

  }
  return ret;
}

std::vector<Eigen::Affine3d> RegistrationScoreEval::generateOffset2DSet(int dimidx1, int dimidx2) {

  std::vector<Eigen::Affine3d> ret;
  std::vector<double> incr(6);
  incr.resize(6);

  for (int i = 0; i < 3; i++) {
    incr[i] = incr_dist;
    incr[i+3] = incr_ang;
  }

  Eigen::VectorXd offset(6);
  offset.setZero();
  for (int j = -offset_size ; j <= offset_size; j++) {
    offset[dimidx1] = j*incr[dimidx1];
    for (int i = -offset_size; i <= offset_size; i++) {
      offset[dimidx2] = i*incr[dimidx2];
      ret.push_back(ndt_generic::vectorToAffine3d(offset));
    }
  }

  return ret;
}

int RegistrationScoreEval::nbPoses() const {
  if (Todom.size() < Tgt.size())
    return Todom.size();
  return Tgt.size();
}

void RegistrationScoreEval::computeScoreSetsGlobalMap(int idx2, int dimidx1, int dimidx2) {
  // The pcd is given in sensor frame...
  pcl::PointCloud<pcl::PointXYZ> pc2;
  ndt_generic::loadCloud(base_name_pcd_, idx2, pc2);

  if (pc2.empty()) {
    std::cerr << "no points(!) found" << std::endl;
    return;
  }

  std::cout << "loaded pc2 # points : " << pc2.size() << std::endl;
  std::cout << "Sensor pose used : " << ndt_generic::affine3dToStringRotMat(sensor_pose_);

  // Work in the vehicle frame... move the points
  lslgeneric::transformPointCloudInPlace(sensor_pose_, pc2);

  // Compute the ndtmap
  lslgeneric::NDTMap nd2(new lslgeneric::LazyGrid(resolution), true);
  nd2.loadPointCloud(pc2);
  nd2.computeNDTCellsSimple();

  // Relative odometry
  T_rel_odom_ = global_ndtmap_T_.inverse() * Todom[idx2];

  std::cout << "T_rel_odom : " << ndt_generic::affine3dToStringRPY(T_rel_odom_) << std::endl;

  // Compute the covariance of the motion
  Eigen::MatrixXd Tcov = motion_model.getCovMatrix(T_rel_odom_);

  // Compute matches - currently we're only using the D2D.
  lslgeneric::NDTMatcherD2D matcher_d2d;

  //matcher_d2d.ITR_MAX = 1000;
  //matcher_d2d.DELTA_SCORE = 0.00001;

  matcher_d2d.match(global_ndtmap_, nd2, T_d2d_, true);

  std::vector<Eigen::Affine3d> Ts = generateOffset2DSet(dimidx1,dimidx2);
  results.resize(Ts.size());

  T_rel_ = T_d2d_;

  for (int i = 0; i < Ts.size(); i++) {
    double score_d2d;
    std::cout << "." << std::flush;
    std::vector<lslgeneric::NDTCell*> nextNDT = nd2.pseudoTransformNDT(T_rel_*Ts[i]);
    score_d2d = matcher_d2d.scoreNDT(nextNDT, global_ndtmap_);
    for(unsigned int j=0; j<nextNDT.size(); j++)
    {
        if(nextNDT[j]!=NULL)
      delete nextNDT[j];
    }
    results[i].offset = Ts[i];
    results[i].score_d2d = score_d2d;
  }

  // Update the relative transform.
  T_rel_gt_ = global_ndtmap_T_.inverse() * Tgt[idx2];

  // Store the transformed pcd
  pc_odom_ = lslgeneric::transformPointCloud(T_rel_odom_, pc2);
  pc_gt_ = lslgeneric::transformPointCloud(T_rel_gt_, pc2);
  pc_d2d_ = lslgeneric::transformPointCloud(T_d2d_, pc2);

  // Update the global transf (using odom and gt as well here incase the idx changes)
  T_glb_d2d_ = T_glb_d2d_ * T_d2d_;
  T_glb_gt_ = T_glb_gt_ * T_rel_gt_;
  T_glb_odom_ = T_glb_odom_ * T_rel_odom_;

  Ts_glb_d2d_.push_back(T_glb_d2d_);
  Ts_glb_gt_.push_back(T_glb_gt_);
  Ts_glb_odom_.push_back(T_glb_odom_);
}

void RegistrationScoreEval::computeScoreSets(int idx1, int idx2, int dimidx1, int dimidx2) {

  // The pcd is given in sensor frame...
  pcl::PointCloud<pcl::PointXYZ> pc1, pc2;
  ndt_generic::loadCloud(base_name_pcd_, idx1, pc1);
  ndt_generic::loadCloud(base_name_pcd_, idx2, pc2);

  if (pc1.empty() || pc2.empty()) {
    std::cerr << "no points(!) found" << std::endl;
    return;
  }

  std::cout << "loaded pc1 # points : " << pc1.size() << std::endl;
  std::cout << "loaded pc2 # points : " << pc2.size() << std::endl;
  std::cout << "Sensor pose used : " << ndt_generic::affine3dToStringRotMat(sensor_pose_);

  // Work in the vehicle frame... move the points
  lslgeneric::transformPointCloudInPlace(sensor_pose_, pc1);
  lslgeneric::transformPointCloudInPlace(sensor_pose_, pc2);

  // Compute the ndtmap
  lslgeneric::NDTMap nd1(new lslgeneric::LazyGrid(resolution), true);
  nd1.loadPointCloud(pc1);
  nd1.computeNDTCellsSimple();

  lslgeneric::NDTMap nd2(new lslgeneric::LazyGrid(resolution), true);
  nd2.loadPointCloud(pc2);
  nd2.computeNDTCellsSimple();

  // Relative odometry
  T_rel_odom_ = Todom[idx1].inverse() * Todom[idx2];

  std::cout << "T_rel_odom : " << ndt_generic::affine3dToStringRPY(T_rel_odom_) << std::endl;

  // Compute the covariance of the motion
  Eigen::MatrixXd Tcov = motion_model.getCovMatrix(T_rel_odom_);


  // Compute matches
  lslgeneric::NDTMatcherD2D matcher_d2d;
  lslgeneric::NDTMatcherD2DSC matcher_d2d_sc;
  T_d2d_ = T_rel_odom_;
  T_d2d_sc_ = T_rel_odom_;

  //matcher_d2d.ITR_MAX = 1000;
  //matcher_d2d.DELTA_SCORE = 0.00001;

  // Compute the matching time...
  double t0 = ndt_generic::getDoubleTime();
  matcher_d2d.match(nd1, nd2, T_d2d_, true);
  double t1 = ndt_generic::getDoubleTime();
  matcher_d2d_sc.match(nd1, nd2, T_d2d_sc_, Tcov);
  double t2 = ndt_generic::getDoubleTime();
  time_d2d_.push_back(t1 - t0);
  time_d2d_sc_.push_back(t2 - t1);

  std::vector<Eigen::Affine3d> Ts = generateOffset2DSet(dimidx1,dimidx2);
  results.resize(Ts.size());

  T_rel_ = T_rel_odom_;
  if (use_d2d_in_grid) {
    T_rel_ = T_d2d_; // Will only work with the d2d not d2d SC since the d2d SC has an impact on the offset used when computing the score. To get SC to work the offset's must be adjusted accordingly.
  }
  else if (use_d2d_sc_in_grid) {
    T_rel_ = T_d2d_sc_;
  }

  for (int i = 0; i < Ts.size(); i++) {
    double score_d2d, score_d2d_sc;
    std::cout << "." << std::flush;
    if (use_d2d_in_grid) {
      matcher_d2d_sc.scoreComparision(nd1, nd2, T_d2d_, Tcov, score_d2d, score_d2d_sc, Ts[i], Ts[i], alpha_);
    }
    else if (use_d2d_sc_in_grid) {
      // The provided offset needs to be relative to the proided
      matcher_d2d_sc.scoreComparision(nd1, nd2, T_d2d_sc_, Tcov, score_d2d, score_d2d_sc, Ts[i], T_rel_odom_.inverse() * T_d2d_sc_ * Ts[i], alpha_);
    }
    else {
      matcher_d2d_sc.scoreComparision(nd1, nd2, T_rel_odom_, Tcov, score_d2d, score_d2d_sc, Ts[i], Ts[i], alpha_);
    }
    results[i].offset = Ts[i];
    results[i].score_d2d = score_d2d;
    results[i].score_d2d_sc = score_d2d_sc;
  }

  // Compute a "filtered" match - based on the d2d output and odometry weighed by the covariances.
  Eigen::MatrixXd T_d2d_cov(6,6);
  if (matcher_d2d.covariance(nd1, nd2, T_d2d_, T_d2d_cov)) {
    std::cout << "T_d2d_cov : " << T_d2d_cov << std::endl;
    std::cout << "T_d2d_cov condition : " << ndt_generic::getCondition(T_d2d_cov) << std::endl;
    std::cout << "T_d2d_cov.inverse() : " << T_d2d_cov.inverse() << std::endl;
    T_filter_ = ndt_generic::getWeightedPose(T_rel_odom_, Tcov, T_d2d_, T_d2d_cov);
  }
  else {
    T_filter_ = T_d2d_;
  }

  // Compute matches using ICP along with the cov
  ICPMatcherP2P matcher_icp;
  matcher_icp.match(pc1, pc2, T_icp_);
  pc_icp_final_ = matcher_icp.getFinalPC();
  Eigen::MatrixXd T_icp_cov(6,6);
  if (matcher_icp.covariance(pc1, pc2, T_icp_, T_icp_cov)) {
    std::cout << "T_icp_cov : " << T_icp_cov << std::endl;
    std::cout << "T_icp_cov condition : " << ndt_generic::getCondition(T_icp_cov) << std::endl;
    std::cout << "T_icp_cov.inverse() : " << T_icp_cov.inverse() << std::endl;

    T_icp_filter_ = ndt_generic::getWeightedPose(T_rel_odom_, Tcov, T_icp_, T_icp_cov);
  }
  else {
    // Should not happen
    T_icp_filter_ = T_icp_;
  }


  // Update the relative transform.
  T_rel_gt_ = Tgt[idx1].inverse() * Tgt[idx2];

  // Store the transformed pcd
  pc_odom_ = lslgeneric::transformPointCloud(T_rel_odom_, pc2);
  pc_gt_ = lslgeneric::transformPointCloud(T_rel_gt_, pc2);
  pc_d2d_ = lslgeneric::transformPointCloud(T_d2d_, pc2);
  pc_d2d_sc_ = lslgeneric::transformPointCloud(T_d2d_sc_, pc2);
  pc_icp_ = lslgeneric::transformPointCloud(T_icp_, pc2);
  pc1_ = pc1;

  // Update the global transf (using odom and gt as well here incase the idx changes)
  T_glb_d2d_ = T_glb_d2d_ * T_d2d_;
  T_glb_d2d_sc_ = T_glb_d2d_sc_ * T_d2d_sc_;
  T_glb_icp_ = T_glb_icp_ * T_icp_;
  T_glb_filter_ = T_glb_filter_ * T_filter_;
  T_glb_icp_filter_ = T_glb_icp_filter_ * T_icp_filter_;
  T_glb_gt_ = T_glb_gt_ * T_rel_gt_;
  T_glb_odom_ = T_glb_odom_ * T_rel_odom_;

  Ts_glb_d2d_.push_back(T_glb_d2d_);
  Ts_glb_d2d_sc_.push_back(T_glb_d2d_sc_);
  Ts_glb_icp_.push_back(T_glb_icp_);
  Ts_glb_filter_.push_back(T_glb_filter_);
  Ts_glb_icp_filter_.push_back(T_glb_icp_filter_);
  Ts_glb_gt_.push_back(T_glb_gt_);
  Ts_glb_odom_.push_back(T_glb_odom_);
}

void RegistrationScoreEval::save(const std::string &filename, int grididx) const {

  std::cout << "saving to : " << filename << std::endl;
  // Save the output for plotting...
  std::ofstream ofs;
  ofs.open(filename.c_str());
  if (!ofs.is_open())
    return;

  for (int i = 0; i < results.size(); i++) {
    ofs << ndt_generic::affine3dToStringRPY(results[i].offset) << " " << results[i].score_d2d << " " << results[i].score_d2d_sc << std::endl;

    // for gnuplot - if you want to plot a grid you need to throw in some blank lines
    if (i < results.size()-1) {
      if (results[i].offset.translation()[grididx] !=
          results[i+1].offset.translation()[grididx]) {
        ofs << std::endl;
      }
    }
  }
  ofs.close();
}

// Save in a 2d grid, to be able to plot a 2d grid with different colors (gnuplot pm3d)
void RegistrationScoreEval::save2D(const std::string &filename, int dimidx1, int dimidx2) const {

  std::cout << "saving to : " << filename << std::endl;
  // Save the output for plotting...
  std::ofstream ofs;
  ofs.open(filename.c_str());
  if (!ofs.is_open())
    return;

  for (int i = 0; i < results.size(); i++) {
    Eigen::VectorXd x = ndt_generic::affine3dToVector(results[i].offset);
    ofs << x[dimidx1] << " " << x[dimidx2] << " " << results[i].score_d2d << " " << results[i].score_d2d_sc << std::endl;

    // for gnuplot - if you want to plot a grid you need to throw in some blank lines
    if (i < results.size()-1) {
      if (results[i].offset.translation()[dimidx1] !=
          results[i+1].offset.translation()[dimidx1]) {
        ofs << std::endl;
      }
    }
  }
  ofs.close();

}

void RegistrationScoreEval::savePCD(const std::string &filename) const {
  pcl::io::savePCDFile<pcl::PointXYZ>(filename + ".gt.pcd", pc_gt_);
  pcl::io::savePCDFile<pcl::PointXYZ>(filename + ".odom.pcd", pc_odom_);
  pcl::io::savePCDFile<pcl::PointXYZ>(filename + ".d2d.pcd", pc_d2d_);
  pcl::io::savePCDFile<pcl::PointXYZ>(filename + ".d2d_sc.pcd", pc_d2d_sc_);
  pcl::io::savePCDFile<pcl::PointXYZ>(filename + ".icp.pcd", pc_icp_);
  pcl::io::savePCDFile<pcl::PointXYZ>(filename + ".icp_final.pcd", pc_icp_final_);
  pcl::io::savePCDFile<pcl::PointXYZ>(filename + ".pc1.pcd", pc1_);
}


// Saves in the same frame as with the offset is used (the odometry frame)
void RegistrationScoreEval::savePoseEstInOdomFrame(const std::string &filename) const {
  ndt_generic::saveAffine3dRPY(filename + std::string(".gt"), T_rel_odom_.inverse()*T_rel_gt_);
  ndt_generic::saveAffine3dRPY(filename + std::string(".d2d"), T_rel_odom_.inverse()*T_d2d_);
  ndt_generic::saveAffine3dRPY(filename + std::string(".d2d_sc"), T_rel_odom_.inverse()*T_d2d_sc_);
  ndt_generic::saveAffine3dRPY(filename + std::string(".icp"), T_rel_odom_.inverse()*T_icp_);
  ndt_generic::saveAffine3dRPY(filename + std::string(".filter"), T_rel_odom_.inverse()*T_filter_);
  ndt_generic::saveAffine3dRPY(filename + std::string(".odom"), T_rel_odom_.inverse()*T_rel_odom_);
}

void RegistrationScoreEval::saveTsToEvalFiles(const std::string &filename) const {
  ndt_generic::saveAffineToEvalFile(filename + std::string(".gt"), Ts_glb_gt_);
  ndt_generic::saveAffineToEvalFile(filename + std::string(".d2d"), Ts_glb_d2d_);
  ndt_generic::saveAffineToEvalFile(filename + std::string(".d2d_sc"), Ts_glb_d2d_sc_);
  ndt_generic::saveAffineToEvalFile(filename + std::string(".icp"), Ts_glb_icp_);
  ndt_generic::saveAffineToEvalFile(filename + std::string(".filter"), Ts_glb_filter_);
  ndt_generic::saveAffineToEvalFile(filename + std::string(".icp_filter"), Ts_glb_icp_);
  ndt_generic::saveAffineToEvalFile(filename + std::string(".odom"), Ts_glb_odom_);
}

void RegistrationScoreEval::saveComputationTime(const std::string &filename) const {
  ndt_generic::saveDoubleVecTextFile(time_d2d_, filename + std::string(".d2d"));
  ndt_generic::saveDoubleVecTextFile(time_d2d_sc_, filename + std::string(".d2d_sc"));
}

std::vector<std::vector<boost::tuple<double, double, double> > >
RegistrationScoreEval::getScoreSegments(int dimidx1, int dimidx2, bool d2d_sc_score) const {

  std::vector<std::vector<boost::tuple<double, double, double> > > ret;
  std::vector<boost::tuple<double, double, double> > segm;
  for (int i = 0; i < results.size(); i++) {
    Eigen::VectorXd x = ndt_generic::affine3dToVector(results[i].offset);
    double score = results[i].score_d2d;
    if (d2d_sc_score) {
      score = results[i].score_d2d_sc;
    }
    segm.push_back(boost::make_tuple(x[dimidx1], x[dimidx2], score));
    std::cout << "." << std::flush;

    if (i < results.size()-1) {
      if (results[i].offset.translation()[dimidx1] !=
          results[i+1].offset.translation()[dimidx1]) {
        std::cout << std::endl;
        ret.push_back(segm);
        segm.resize(0);
      }
    }

  }
  return ret;
}

std::vector<std::vector<boost::tuple<double, double, double> > >
RegistrationScoreEval::getRelPoseGT(int dimidx1, int dimidx2) const {

  std::vector<std::vector<boost::tuple<double, double, double> > > ret;
  std::vector<boost::tuple<double, double, double> > segm;
  Eigen::VectorXd t = ndt_generic::affine3dToVector(T_rel_.inverse() * T_rel_gt_);
  segm.push_back(boost::tuple<double,double,double>(t[dimidx1],t[dimidx2], 0.));
  ret.push_back(segm);
  return ret;
}

std::vector<boost::tuple<double, double, double> >
RegistrationScoreEval::getRelPoseD2D(int dimidx1, int dimidx2) const {

  std::vector<boost::tuple<double, double, double> > ret;
  Eigen::VectorXd t = ndt_generic::affine3dToVector(T_rel_.inverse() * T_d2d_);
  ret.push_back(boost::tuple<double,double,double>(t[dimidx1],t[dimidx2], 0.));
  return ret;
}

std::vector<boost::tuple<double, double, double> >
RegistrationScoreEval::getRelPoseD2D_SC(int dimidx1, int dimidx2) const {

  std::vector<boost::tuple<double, double, double> > ret;
  Eigen::VectorXd t = ndt_generic::affine3dToVector(T_rel_.inverse() * T_d2d_sc_);
  ret.push_back(boost::tuple<double,double,double>(t[dimidx1],t[dimidx2], 0.));
  return ret;
}

std::vector<boost::tuple<double, double, double> >
RegistrationScoreEval::getRelPoseICP(int dimidx1, int dimidx2) const {

  std::vector<boost::tuple<double, double, double> > ret;
  Eigen::VectorXd t = ndt_generic::affine3dToVector(T_rel_odom_.inverse() * T_icp_);
  ret.push_back(boost::tuple<double,double,double>(t[dimidx1],t[dimidx2], 0.));
  return ret;
}

std::vector<boost::tuple<double, double, double> >
RegistrationScoreEval::getRelPoseFilter(int dimidx1, int dimidx2) const {

  std::vector<boost::tuple<double, double, double> > ret;
  Eigen::VectorXd t = ndt_generic::affine3dToVector(T_rel_.inverse() * T_filter_);
  ret.push_back(boost::tuple<double,double,double>(t[dimidx1],t[dimidx2], 0.));
  return ret;
}

std::vector<boost::tuple<double, double, double> >
RegistrationScoreEval::getRelPoseICPFilter(int dimidx1, int dimidx2) const {

  std::vector<boost::tuple<double, double, double> > ret;
  Eigen::VectorXd t = ndt_generic::affine3dToVector(T_rel_.inverse() * T_icp_filter_);
  ret.push_back(boost::tuple<double,double,double>(t[dimidx1],t[dimidx2], 0.));
  return ret;
}

std::vector<boost::tuple<double, double, double> >
RegistrationScoreEval::getRelPoseOdom(int dimidx1, int dimidx2) const {
  // Always 0,0,0,0,0,0....
  std::vector<boost::tuple<double, double, double> > ret;
  Eigen::VectorXd t = ndt_generic::affine3dToVector(T_rel_.inverse() * T_rel_odom_);
  ret.push_back(boost::tuple<double,double,double>(t[dimidx1],t[dimidx2]));
  return ret;
}


