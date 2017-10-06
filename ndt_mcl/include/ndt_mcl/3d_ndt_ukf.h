#pragma once

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <ndt_generic/eigen_utils.h>
#include <ndt_generic/pcl_utils.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/pointcloud_utils.h>
#include <tf_conversions/tf_eigen.h>

//! Implements a UKF in 3D (using x,y,z and euler angles)
class UKF3D {
public:

    class Params {
    public:
        Params() : 
            alpha(0.01),
            beta(2.),
            kappa(0.),
            range_var(1.),
            min_pos_var(0.01),
            min_rot_var(0.01),
            range_filter_max_dist(1.),
            nb_ranges_in_update(100),
            nb_updates(1000)
        {
        }
        double alpha;
        double beta;
        double kappa;
        double range_var;
        double min_pos_var;
        double min_rot_var;
        double range_filter_max_dist;
        int nb_ranges_in_update;
        int nb_updates;
    };

    UKF3D() : N_(6) {

        allocateSigmas();
    }

    void setParams(const UKF3D::Params &params) {
        params_ = params;
    }

    std::string getDebugString() const {
        std::ostringstream stream;
        stream << "alpha : " << params_.alpha << " beta : " << params_.beta << " kappa : " << params_.kappa << " N : " << N_ << " # sigma points : " << sigmas_.size() << std::endl;
        for (int i = 0; i < sigmas_.size(); i++) {
            stream << "[" << i << "] : " << sigmas_[i].transpose() << std::endl;
        }
        return stream.str();
    }

    int getNbSigmaPoints() const {
        return 2*N_+1;
    }

    std::vector<double> getMeanWeights() const;
    std::vector<double> getCovWeights() const;

    Eigen::Affine3d getMean() const{

        return T_;
    }

    Eigen::MatrixXd getCov() const{
        return P_;
    }


    Eigen::VectorXd computePoseMean() const;
    Eigen::MatrixXd computePoseCov(const Eigen::VectorXd &mean) const;

    double getWeightMean(int idx) const;
    double getWeightCov(int idx) const;

    // Sigma points, stored as x,y,z, roll,pitch, yaw.
    std::vector<Eigen::VectorXd> sigmas_;
    // Current pose estimate
    Eigen::Affine3d T_;
    // Current pose covariance
    Eigen::MatrixXd P_; 
    // UKF parameters
    Params params_;
    // State dim
    int N_;

    double getLambda() const {
        return params_.alpha*params_.alpha*(N_ + params_.kappa) - N_;
    }
    
    double getXsi() const {
        return sqrt(N_+getLambda());
    }
    
    const Eigen::VectorXd& getSigma(int i)  const {
        return sigmas_[i];
    }

    void allocateSigmas() {
        assert(!sigmas_.empty());
        for (int i = 0; i < 2*N_+1; i++) {
            Eigen::VectorXd s(N_);
            sigmas_.push_back(s);
        }
    }

    std::vector<Eigen::Affine3d> getSigmasAsAffine3d() const;


    void assignSigmas(const Eigen::VectorXd &mean, const Eigen::MatrixXd &cov);
        
    void initializeFilter(const Eigen::Affine3d &pose,
                          const Eigen::MatrixXd &posecov) {
        T_ = pose;
        P_ = posecov;
    }

    void normalizeEuler() {
        for (int i = 0; i < sigmas_.size(); i++) {
            ndt_generic::normalizeEulerAngles6dVec(sigmas_[i]);
        }
    }


    void predict(const Eigen::Affine3d &incr, const Eigen::MatrixXd &incrCov);

    // Assure that the diagonal vairances is never less than ...
    void assignMinDiagVariances();

    
    // Filter the predicted and raw readings to assure that the distance between them are less than a threshold.
    std::vector<int> filterMeasurements(Eigen::MatrixXd &pred_ranges, Eigen::VectorXd &raw_ranges,
                                        Eigen::MatrixXd &filter_pred_ranges, Eigen::VectorXd &filter_raw_ranges) const;
    
    // The update step (fuse the sensor readings and predicted readings)
    void update(const Eigen::MatrixXd &pred_ranges,
                const Eigen::VectorXd &raw_ranges);
    
    // Divides the sensory readings / prediction and performs a set of udpate steps.
    void updateSeq(const Eigen::MatrixXd &pred_ranges,
                   const Eigen::VectorXd &raw_ranges);
        
};

/**
 * NDT UKF 3D - Class implementation, puts the NDT map and UKF together.
 */
class NDTUKF3D{
public:
    perception_oru::NDTMap map;
    double resolution;
    double resolution_sensor;
    int counter;
    std::vector<double> motion_model, motion_model_offset;
    
    /**
     * Constructor
     */
    NDTUKF3D(double map_resolution, perception_oru::NDTMap &nd_map):
        map(new perception_oru::LazyGrid(map_resolution))
    {
        isInit = false;
        resolution=map_resolution;
        resolution_sensor = resolution;
        counter = 0;
        
        ///First, lets make our target map match the given map
        ///This is done because we want (possibly) lower resolution target map
        double cx,cy,cz;
        if(!nd_map.getCentroid(cx, cy, cz)){
            fprintf(stderr,"Centroid NOT Given-abort!\n");
            exit(1);
        }else{
            fprintf(stderr,"Centroid(%lf,%lf,%lf)\n",cx,cy,cz);
        }

        double wx,wy,wz;

        if(!nd_map.getGridSizeInMeters(wx, wy, wz)){
            fprintf(stderr,"Grid size NOT Given-abort!\n");
            exit(1);
        }else{
            fprintf(stderr,"GridSize(%lf,%lf,%lf)\n",wx,wy,wz);
        }

        map.initialize(cx,cy,cz,wx,wy,wz);

        std::vector<perception_oru::NDTCell*> ndts;
        ndts = nd_map.getAllCells();
        fprintf(stderr,"NDT MAP with %d components",(int)ndts.size());
        for(unsigned int i=0;i<ndts.size();i++){
            Eigen::Vector3d m = ndts[i]->getMean();	
            Eigen::Matrix3d cov = ndts[i]->getCov();
            unsigned int nump = ndts[i]->getN();
            map.addDistributionToCell(cov, m,nump);
        }

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

        motion_model_offset.push_back(0.002);
        motion_model_offset.push_back(0.002);
        motion_model_offset.push_back(0.002);
        motion_model_offset.push_back(0.001);
        motion_model_offset.push_back(0.001);
        motion_model_offset.push_back(0.001);
    }

    void computeMeasurements(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tsensor, Eigen::MatrixXd &meassurements, Eigen::VectorXd &raw_ranges);

    void predict(Eigen::Affine3d Tmotion);

    void updateAndPredict(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tsensor);

    //    void updateAndPredictEff(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, double subsample_level);

    Eigen::Affine3d getMean() const {
        return ukf_.getMean();
    }

    void initializeFilter(const Eigen::Affine3d &T,
                          double varx, double vary, double varz, double varR, double varP, double varY) {
        Eigen::MatrixXd cov(6,6);
        cov.setZero();
        cov(0,0) = varx; cov(1,1) = vary; cov(2,2) = varz; cov(3,3) = varR; cov(4,4) = varP; cov(5,5) = varY;
        
        ukf_.initializeFilter(T, cov);
    }



    std::vector<Eigen::Affine3d> getSigmasAsAffine3d() const {
        return ukf_.getSigmasAsAffine3d();
    }
     
    void setParamsUKF(const UKF3D::Params &params) {
        ukf_.setParams(params);
    }

    void updateVisualizationClouds(const std::vector<int> &idx, const pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::MatrixXd &filter_pred_ranges, const Eigen::Affine3d &Tsensor);

    const pcl::PointCloud<pcl::PointXYZ>& getFilterRaw() const {
        return pc_filtered_raw_;
    }

    const pcl::PointCloud<pcl::PointXYZ>& getFilterPred() const {
        return pc_filtered_pred_;
    }

private:
    bool isInit;
    double getDoubleTime()
    {
        struct timeval time;
        gettimeofday(&time,NULL);
        return time.tv_sec + time.tv_usec * 1e-6;
    }

    // UKF variables
    UKF3D ukf_;

    // Store some point clouds for debugging / visualization
    pcl::PointCloud<pcl::PointXYZ> pc_filtered_raw_;
    pcl::PointCloud<pcl::PointXYZ> pc_filtered_pred_;
};


