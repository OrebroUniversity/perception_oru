#pragma once

#include <ndt_calibration/ndt_calib_affine3d.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <eigen_conversions/eigen_msg.h>

#include <ndt_map/pointcloud_utils.h>
#include <ndt_registration/ndt_matcher_d2d.h> // scoreNDT().
#include <ndt_offline/PoseInterpolationNavMsgsOdo.h>

template<class T> std::string toString (const T& x)
{
    std::ostringstream o;
  
    if (!(o << x))
	throw std::runtime_error ("::toString()");
  
    return o.str ();
}


class NDTCalibScan {
public:
  pcl::PointCloud<pcl::PointXYZ> cloud; // cloud in sensor coords

  Eigen::Affine3d pose; // estimated pose in global frame (same as originalPose) but with a corrected pose based on a correction time_offset factor, this variable is special since this could be changed in the optimization wheras the others are constants

  Eigen::Affine3d originalPose;  // estimated pose in global frame (preferably from a GT system)...
  Eigen::Affine3d estSensorPose; // estimated sensor pose in global frame (typically from a SLAM / registration system)
  double stamp;  // timestamp of originalPose
  lslgeneric::NDTMap* ndtmap;
  
  

  NDTCalibScan() : ndtmap(NULL) {
    
  }
  
 NDTCalibScan(const Eigen::Affine3d &_pose, const Eigen::Affine3d &_estSensorPose, double _stamp) : pose(_pose), originalPose(_pose), estSensorPose(_estSensorPose), stamp(_stamp), ndtmap(NULL) {
    
  }
  
 NDTCalibScan(const pcl::PointCloud<pcl::PointXYZ> &_cloud, const Eigen::Affine3d &_pose, double _stamp) : cloud(_cloud), pose(_pose), originalPose(_pose), stamp(_stamp), ndtmap(NULL) {
  }

 NDTCalibScan(const pcl::PointCloud<pcl::PointXYZ> &_cloud, const Eigen::Affine3d &_pose, const Eigen::Affine3d &_estSensorPose, double _stamp)  : cloud(_cloud), pose(_pose), originalPose(_pose), estSensorPose(_estSensorPose), stamp(_stamp), ndtmap(NULL)
  {
    
  }
  
  ~NDTCalibScan() {
    if (ndtmap != NULL) {
      delete &ndtmap;
    }
  }
    
  void computeNDTMap(double resolution) {
    ndtmap = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution), true);
    ndtmap->loadPointCloud(this->cloud);
    ndtmap->computeNDTCellsSimple();
  }

  void appendPointCloudGlobal(const Eigen::Affine3d &Ts, pcl::PointCloud<pcl::PointXYZ> &p) const {
    
    pcl::PointCloud<pcl::PointXYZ> c =cloud;
    Eigen::Affine3d T = pose * Ts;
    lslgeneric::transformPointCloudInPlace(T, c);
    p += c;
  }

  // Simply the euclidean offset between the estimated sensor pose and the given gt poses
  double scoreEstSensorPose(const Eigen::Affine3d &Ts) const {
    return this->getDifferenceVector(Ts).norm();
  }

    // Return Ts from the global estimate sensor pose and the vehicle pose (both in the same global reference frame).  
  Eigen::Affine3d getTs() const {
    return (pose.inverse() * estSensorPose);
  }

  // Compute difference between the relative estimated and the predicted sensor pose
  Eigen::Affine3d getDifference(const Eigen::Affine3d &Ts) const {
    Eigen::Affine3d T = getSensorPoseFromPose(Ts);
    
    return estSensorPose.inverse()*T;
  }

  // Get the difference in x,y,z and euler angles
  Eigen::VectorXd getDifferenceVector(const Eigen::Affine3d &Ts)  const {
    return getTranslationEulerAnglesVectorFromAffine3d(getDifference(Ts));
  }

  
  Eigen::Affine3d getSensorPoseFromPose(const Eigen::Affine3d &Ts) const  {
    return pose*Ts;
  }
};


class NDTCalibScanPair {

public:	
  NDTCalibScan first;
  NDTCalibScan second;

  NDTCalibScanPair() {

  }
  NDTCalibScanPair(const pcl::PointCloud<pcl::PointXYZ> &p1, const Eigen::Affine3d &pose1,
                   const pcl::PointCloud<pcl::PointXYZ> &p2, const Eigen::Affine3d &pose2) :
    first(NDTCalibScan(p1, pose1, -1.)), second(NDTCalibScan(p2, pose2, -1.))
  {    

  }

  Eigen::Affine3d getRelativePose() const {
    return this->first.pose.inverse() * this->second.pose;
  }

  Eigen::Affine3d getRelativeEstSensorPose() const {
    return this->first.estSensorPose.inverse() * this->second.estSensorPose;
  }
  
  // Compute the score for a relative sensor pose offset - that is the pose and the est sensor pose don't have to be in the same coordinate frame.
  double scoreEstSensorPoseRel(const Eigen::Affine3d &Ts) const {
    return this->getDifferenceVector(Ts).norm();
  }

  // Compute the predicted relative estimated sensor pose.
  Eigen::Affine3d getPredictedRelativeEstSensorPose(const Eigen::Affine3d &Ts) const {
    Eigen::Affine3d rel = getRelativePose();
    return (Ts.inverse() * rel * Ts);
  }

  // Compute difference between the relative estimated and the predicted sensor pose
  Eigen::Affine3d getDifference(const Eigen::Affine3d &Ts) const {
    Eigen::Affine3d T = getPredictedRelativeEstSensorPose(Ts);
    Eigen::Affine3d rel_est = getRelativeEstSensorPose();
    
    return rel_est.inverse()*T;
  }

  // Get the difference in x,y,z and euler angles
  Eigen::VectorXd getDifferenceVector(const Eigen::Affine3d &Ts)  const {
    return getTranslationEulerAnglesVectorFromAffine3d(getDifference(Ts));
  }
  
  ///Get point cloud in global coords
  void appendPointCloudGlobal(const Eigen::Affine3d &Ts, pcl::PointCloud<pcl::PointXYZ> &p) const {
    
    this->first.appendPointCloudGlobal(Ts, p);
    this->second.appendPointCloudGlobal(Ts, p);
  }

  // Compute ICP score for sensor offset
  double scoreICP(const Eigen::Affine3d &Ts) const {
    Eigen::Affine3d Tsecond = getPredictedRelativeEstSensorPose(Ts);
    pcl::PointCloud<pcl::PointXYZ> c1=this->first.cloud;
    // Enough to transform one map
    pcl::PointCloud<pcl::PointXYZ> c2=this->second.cloud;
    Eigen::Affine3d p2 = Tsecond;
    lslgeneric::transformPointCloudInPlace(p2, c2);
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; // TODO, compute the KD-tree once to gain some more speed.
    typename pcl::KdTree<pcl::PointXYZ>::PointCloudPtr mp (new pcl::PointCloud<pcl::PointXYZ>);
    if(c1.size()==0){
        std::cerr << " Check -> num points = " << c1.size() << std::endl;
    }
    (*mp) = c1;
    kdtree.setInputCloud (mp);
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    double e=0, error_th=0.1*0.1;
    
    for(unsigned int i=0;i<c2.size();i++){
      if ( kdtree.nearestKSearch (c2[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
        if(pointNKNSquaredDistance[0] > error_th){
          e+=error_th;
        }
        else{
          e+= pointNKNSquaredDistance[0];
        }
      }
    }
    
    return e;
  }

  void computeNDTMap(double resolution) {
    if (first.ndtmap == NULL) {
      first.computeNDTMap(resolution);
    }
    if (second.ndtmap == NULL) {
      second.computeNDTMap(resolution);
    }
  }

  double scoreNDT(const Eigen::Affine3d &Ts) const  {
    if (first.ndtmap == NULL || second.ndtmap == NULL) {
      std::cout << "NDT map not computed(!)" << std::endl;
      return -1;
    }
	
    lslgeneric::NDTMatcherD2D matcher;
    // Enough to transform one map.
    Eigen::Affine3d Tsecond = getPredictedRelativeEstSensorPose(Ts);
    std::vector<lslgeneric::NDTCell*> m2 = second.ndtmap->pseudoTransformNDT(Tsecond);
    double score = matcher.scoreNDT(m2, *first.ndtmap);
    for (size_t i = 0; i < m2.size(); i++) {
      delete m2[i];
    }
    return score;
  }
};

// Container class for at set of calibration pairs
class NDTCalibScanPairs : public std::vector<NDTCalibScanPair> { 

public:
  void getGlobalPointCloud(const Eigen::Affine3d &Ts, pcl::PointCloud<pcl::PointXYZ> &p) {
    p.clear();
    for (size_t i = 0;  i < this->size(); i++) {
      (*this)[i].appendPointCloudGlobal(Ts, p);
    }
  }

  void computeNDTMap(double resolution) {
    for (size_t i = 0;  i < this->size(); i++) {
      (*this)[i].computeNDTMap(resolution);
    }
  }
};




class NDTCalibOptimize {
public:
  enum ScoreType
  {
    SCORE_ICP = 0,
    SCORE_EST_SENSOR_POSE,
    SCORE_EST_SENSOR_POSE_REL,
    SCORE_NDT
  };
    
    // Helper class to determine what parameters should be optimized.
    class ObjectiveType : public std::vector<bool> {
    public:
        ObjectiveType(bool x, bool y, bool z, bool roll, bool pitch, bool yaw, bool dt) {
            this->resize(7);
            (*this)[0] = x;
            (*this)[1] = y;
            (*this)[2] = z;
            (*this)[3] = roll;
            (*this)[4] = pitch;
            (*this)[5] = yaw;
            (*this)[6] = dt;
        }

        // Return if time should be optimized
        bool optimizeTime() const {
            return (*this)[6];
        }
        
        // Number of parameters to optimize.
        int getNbParameters() const {
            int ret = 0;
            for (size_t i = 0; i < this->size(); i++) {
                if ((*this)[i])
                    ret++;
            }
            return ret;
        }

        // Given the initial parameters, only extract the active parameters.
        void getInitialActiveParameters(const Eigen::VectorXd &initParams, Eigen::VectorXd &params) const {
            assert(initParams.size() == this->size());
            
            params.resize(this->getNbParameters());
            for (size_t i = 0, j = 0; i < initParams.size(); i++) {
                if ((*this)[i]) {
                    params[j] = initParams[i];
                    j++;
                }
            }
        }
        
        // Given the initial parameters, the active parameters, return a merged version where all active perameters has overwritten the initial ones.
        void getAllParameters(const Eigen::VectorXd &initParams, const Eigen::VectorXd &activeParams, Eigen::VectorXd &params) {
            params.resize(initParams.size());
            
            for (size_t i = 0, j = 0; i < initParams.size(); i++) {
                if ((*this)[i]) {
                    params[i] = activeParams[j];
                    j++;
                }
                else {
                    params[i] = initParams[i];
                }
            }

        }
    };

    NDTCalibOptimize(NDTCalibScanPairs &pairs, int scoreType, ObjectiveType objectiveType, PoseInterpolationNavMsgsOdo &poseInterp) : _pairs(pairs), _scoreType(scoreType), _objectiveType(objectiveType), _poseInterp(poseInterp) { }
  
    // Main function to call. Provide the initial sensor pose and time offset. These will be updated with the calibrated results.
  bool calibrate(Eigen::Affine3d &initT, double &sensorTimeOffset);
  
  // Number of optimization paramters. Need to be accessable from the optimization functions.
  int getNbParameters() const;

    // Score value. Need to be accessable from the optimization functions.
double getScore(const Eigen::VectorXd &x);

    // Score value given a sensor pose (in order to get the score with another time offset use interpPairPoses first).
    double getScore6d(const Eigen::Affine3d &T) const;

    // Update the poses with a sensor time offset.
    void interpPairPoses(double sensorTimeOffset);

private:

    // Called once in the calibration...
    void setInitialParameters(const Eigen::Affine3d &Ts, const double &t);
    void extractActiveInitialParameters(Eigen::VectorXd &params) const;
    void assignParameters(const Eigen::VectorXd &params, Eigen::Affine3d &Ts, double &t);


  void interpPose(double time, Eigen::Affine3d &T);
  double getScoreTime(double sensorTimeOffset);
  
  PoseInterpolationNavMsgsOdo &_poseInterp;
  NDTCalibScanPairs &_pairs;
  int _scoreType;
  ObjectiveType _objectiveType;

   Eigen::VectorXd _initParameters; // Contains all initial parameters, always length 7.
};

// Load the evaluation files that are generated by the fuser, <timestamp> x y x qx qy qz qw.
std::vector<Eigen::Affine3d> loadAffineFromEvalFile(const std::string &fileName) {
  std::vector<Eigen::Affine3d> ret;
  std::string line;
  std::ifstream myfile (fileName.c_str());
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      double time, x, y, z, qx, qy, qz, qw;
      std::istringstream ss(line);
      ss >> time >> x >> y >> z >> qx >> qy >> qz >> qw;
      ret.push_back(Eigen::Translation3d(x,y,z)*Eigen::Quaterniond(qw, qx, qy, qz));
    }
    myfile.close();
  }
  else {
    std::cout << "Unable to open file : " << fileName << std::endl;
  } 
  
  return ret;
}

// Load timestamps from the evaluation files that are generated by the fuser...
std::vector<double> loadTimeStampFromEvalFile(const std::string &fileName) {
  std::vector<double> ret;
  std::string line;
  std::ifstream myfile (fileName.c_str());
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      double time, x, y, z, qx, qy, qz, qw;
      std::istringstream ss(line);
      ss >> time >> x >> y >> z >> qx >> qy >> qz >> qw;
      ret.push_back(time);
   }
   myfile.close();
 }
  else std::cout << "Unable to open file : " << fileName << std::endl;; 
  
  return ret;
}


void loadCloud(const std::string &base_name_pcd, int counter, pcl::PointCloud<pcl::PointXYZ> &cloud) {
    std::string pcd_file = base_name_pcd + std::string("cloud") + toString(counter) + std::string(".pcd");
    std::cout << "loading : " << pcd_file << std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, cloud);
}


// Specialied load function to load data generated from the offline fuser.
void loadNDTCalibScanPairs(const std::string &gt_file, const std::string &est_sensorpose_file, const std::string &base_name_pcd, NDTCalibScanPairs &scans, double max_translation, double min_rotation) {
  scans.resize(0);

  // Load the data...
  std::vector<double> stamps = loadTimeStampFromEvalFile(gt_file);
  std::vector<Eigen::Affine3d> Tgt = loadAffineFromEvalFile(gt_file);
  std::vector<Eigen::Affine3d> Test_sensorpose = loadAffineFromEvalFile(est_sensorpose_file);

  if (Tgt.size() != stamps.size() || Tgt.size() != Test_sensorpose.size()) {
    std::cerr << "Warning: the length of the provided files to not match(!)" << std::endl;
  }

  // Need to find a pair of scans which is useful in the optimization scheme. Pair of scans with limited translation but with some rotation...
  size_t j = 0;
  for (size_t i = 1; i < Tgt.size(); i++) {

    Eigen::Affine3d Tmotion = Tgt[j].inverse()*Tgt[i];
    // while (Tmotion.translation().norm() > 3) {
    //   j++;
    //   Tmotion = Tgt[i] * Tgt[j].inverse();
    // }
    if (Tmotion.translation().norm() > max_translation) {
      j = i; // reset -> make this better?, could be that many potential pairs are left out..., add a while loop instead.
      continue;
    }
    Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);
    normalizeEulerAngles(rot);
    if (rot.norm() < min_rotation) {
      continue;
    }

    // Good pair found...
    NDTCalibScanPair pair;
    pair.first = NDTCalibScan(Tgt[j], Test_sensorpose[j], stamps[j]);
    pair.second = NDTCalibScan(Tgt[i], Test_sensorpose[i], stamps[i]);

    scans.push_back(pair);
    
    loadCloud(base_name_pcd, j+1, scans.back().first.cloud);  // cloud1.pcd -> first line in the eval files...
    loadCloud(base_name_pcd, i+1, scans.back().second.cloud);

    j = i; // reset
  }
 
}
