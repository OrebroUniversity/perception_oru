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
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/pointcloud_utils.h>
#include <tf_conversions/tf_eigen.h>

// Generic point cloud tools
void computeDirectionsAndRangesFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Vector3d &origin, std::vector<Eigen::Vector3d> &dirs, Eigen::VectorXd &ranges);






// Updates T and Tcov with an incremental update incr using covariance incrcov.
void addAffine3dCovWithIncr(Eigen::Affine3d &T, Eigen::MatrixXd &Tcov,
                            const Eigen::Affine3d &incr, const Eigen::MatrixXd &incrcov)
{

    T = T * incr; // The mean - nice and easy.
    
    // To update the covariance is more tricky, here we instead take a de-tour using quaternions (note that the covariance matrix utilize euler angles).

    
    
    
    // TODO

    // Need the Jacobians for the increments and for the existing Jacobian

    //       double c = cos( origin.mean[2]);
    //       double s = sin( origin.mean[2]);
    //       J_1 << 1, 0, -s * incr.mean[0] - c * incr.mean[1],
    //            0, 1, c * incr.mean[0] - s * incr.mean[1],
    //            0, 0, 1;
	  
    //       J_2 << c, -s, 0,
    //            s, c, 0, 
    //            0, 0, 1;
    
    //    this->P_ = J_1 * this->P_ * J_1.transpose() + J_2 * Q * J_2.transpose();
}



 

class UKF3D {
public:

    UKF3D() : alpha_(0.01), beta_(2.), kappa_(0.), N_(6) {

        allocateSigmas();
    }

    void setParams(double alpha, double beta, double kappa) {
        alpha_ = alpha;
        beta_ = beta;
        kappa_ = kappa;
    }

    std::string getDebugString() const {
        std::ostringstream stream;
        stream << "alpha : " << alpha_ << " beta : " << beta_ << " kappa : " << kappa_ << " N : " << N_ << " # sigma points : " << sigmas_.size() << std::endl;
        for (int i = 0; i < sigmas_.size(); i++) {
            stream << "[" << i << "] : " << sigmas_[i].transpose() << std::endl;
        }
        return stream.str();
    }

    int getNbSigmaPoints() const {
        return 2*N_+1;
    }

    std::vector<double> getMeanWeights() const {
        std::vector<double> ret;
        
        for (int i = 0; i < 2*N_+1; i++) {
            ret.push_back(getWeightMean(i));
        }
        return ret;
    }

    std::vector<double> getCovWeights() const {
        std::vector<double> ret;
        
        for (int i = 0; i < 2*N_+1; i++) {
            ret.push_back(getWeightCov(i));
        }
        return ret;
    }


    Eigen::Affine3d getMean() const{

        return T_;
    }

    Eigen::MatrixXd getCov() const{
        return P_;
    }

    Eigen::MatrixXd computePoseCov(const Eigen::VectorXd &mean) const {
        Eigen::MatrixXd ret(N_,N_);
        ret.setZero();
        assert(mean.size() != N_);

        // for (int i = 0; i < 2*N_+1; i++) {

        //     Eigen::VectorXd diff = mean - this->getSigma(i);
        //     // Normalize the diff angles...
        //     diff(4) = angles::normalize_angle(diff(4));
        //     diff(5) = angles::normalize_angle(diff(5));
        //     diff(6) = angles::normalize_angle(diff(6));

        //     ret += getWeightCov(i) * diff * diff.transpose();
        // }

        std::vector<Eigen::Affine3d> T_sigmas = this->getSigmasAsAffine3d();
        Eigen::Affine3d T_mean = ndt_generic::vectorToAffine3d(mean);

        for (int i = 0; i < T_sigmas.size(); i++) {
            Eigen::Affine3d T_diff = T_sigmas[i]*T_mean.inverse();
            Eigen::VectorXd diff = ndt_generic::affine3dToVector(T_diff);

            // diff(4) = angles::normalize_angle(diff(4));
            // diff(5) = angles::normalize_angle(diff(5));
            // diff(6) = angles::normalize_angle(diff(6));

            ndt_generic::normalizeEulerAngles6dVec(diff);

            std::cout << "diff [" << i << "] : " << diff.transpose() << std::endl;

            ret += getWeightCov(i) * diff * diff.transpose();
        }

        return ret;
    }

    Eigen::VectorXd computePoseMean() const {
        Eigen::VectorXd ret(N_);
        ret.setZero();

        std::vector<double> weights = getMeanWeights();
        std::vector<Eigen::Affine3d> T_sigmas = getSigmasAsAffine3d();
        
        std::cout << "-------------------------------" << std::endl;
        std::cout << this->getDebugString() << std::endl;
    

        Eigen::Affine3d T_mean = ndt_generic::getAffine3dMeanWeightsUsingQuatNaive(T_sigmas, weights);
        

        ret = ndt_generic::affine3dToVector(T_mean);
        
        ndt_generic::normalizeEulerAngles6dVec(ret);
        return ret;
    }

    double getWeightMean(int idx) const {
        const double lambda = this->getLambda();
        if (idx == 0) {
            return (lambda / (N_ + lambda)); 
        }
        return 1./(2*(N_ + lambda));
    }

    double getWeightCov(int idx) const {
        const double lambda = this->getLambda();
        if (idx == 0) {
            return lambda/(N_ + lambda) + (1 - alpha_*alpha_ + beta_);
        }
        return 1./(2*(N_ + lambda));
    }

    // Sigma points, stored as x,y,z, roll,pitch, yaw.
    std::vector<Eigen::VectorXd> sigmas_;
    // Current pose estimate
    Eigen::Affine3d T_;
    // Current pose covariance
    Eigen::MatrixXd P_; 
    double kappa_, alpha_, beta_;
    int N_;

    double getLambda() const {
        return alpha_*alpha_*(N_ + kappa_) - N_;
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

    std::vector<Eigen::Affine3d> getSigmasAsAffine3d() const {
        std::vector<Eigen::Affine3d> ret;
        if (sigmas_.empty()) {
            return ret;
        }
        assert(sigmas_.size() != 2*N+1);
        
        for (int i = 0; i < 2*N_+1; i++) {
            ret.push_back(ndt_generic::vectorToAffine3d(sigmas_[i]));
        }

        return ret;
    }

    void assignSigmas(const Eigen::VectorXd &mean, const Eigen::MatrixXd &cov) {

        // Sigmas is a 6d vector with x,y,z,r,p,y
        sigmas_[0] = mean;
        
        // Compute the cholesky decomposition (chol(cov)' * chol(cov) = cov)
        Eigen::MatrixXd L( cov.llt().matrixL() );

        int idx = 1;
        double xsi = getXsi();
        for (int i = 0; i < N_; i++) {
            sigmas_[idx] = mean - xsi * L.col(i);
            idx++;
            sigmas_[idx] = mean + xsi * L.col(i);
            idx++;
        }

        //        this->normalizeEuler();

        P_ = cov; ////
    }
        
    void initializeFilter(const Eigen::Affine3d &pose,
                          const Eigen::MatrixXd &posecov) {
        assignSigmas(ndt_generic::affine3dToVector(pose), posecov);
    }

    void normalizeEuler() {
        for (int i = 0; i < sigmas_.size(); i++) {
            ndt_generic::normalizeEulerAngles6dVec(sigmas_[i]);
        }
    }


    void predict(const Eigen::Affine3d &incr, const Eigen::MatrixXd &incrCov) {
        
        // Add the incremental pose to all sigmas.
        for (int i = 0; i < sigmas_.size(); i++) {
            Eigen::Affine3d s = ndt_generic::vectorToAffine3d(sigmas_[i]) * incr;
            sigmas_[i] = ndt_generic::affine3dToVector(s);
        }

        // Compute the mean
        Eigen::VectorXd mean = this->computePoseMean();
        
        std::cout << "[mean...] : " << mean.transpose() << std::endl;
        T_ = ndt_generic::vectorToAffine3d(mean);

        // Compute and add the covariance - TODO, do this better?
        P_ = this->computePoseCov(mean);// + incrCov;

        std::cout << "P_ : \n" << P_ << std::endl;

        // Assign the new sigma points
        assignSigmas(mean, P_);
    }

    void update(const Eigen::MatrixXd &pred_ranges,
                const Eigen::VectorXd &raw_ranges) {
        
        // Compute the mean of the ray traced measurements
        Eigen::VectorXd mean(raw_ranges.size());
        mean.setZero();

        double weight_m0 = this->getWeightMean(0);
        double weight_m = this->getWeightMean(1);

        for (int i = 0; i < raw_ranges.size(); i++) {
            mean(i) = weight_m0 * pred_ranges(0,i);
            for (int j = 1; j < 2*this->N_+1; j++) {
                mean(i) += weight_m * pred_ranges(j,i);
            }
        }

        // Compute the covariance of the ray traced masurements
        Eigen::MatrixXd Q(raw_ranges.size(), raw_ranges.size());
        Q.setIdentity();
        Q *= 0.01;  // Range variance.

        Eigen::MatrixXd S(raw_ranges.size(), raw_ranges.size());
        S.setZero();
        
        // Keep the range differences
        std::vector<Eigen::VectorXd> diffs;

        double weight_c0 = this->getWeightCov(0);
        double weight_c = this->getWeightCov(1);

        for (int i = 0; i < 2*this->N_+1; i++)  {
            Eigen::VectorXd diff = pred_ranges.row(i) - raw_ranges;
            if (i == 0)
                S += weight_c0 * diff * diff.transpose();
            else
                S += weight_c * diff * diff.transpose();
            diffs.push_back(diff);
        }
        S += Q;


        // Correct the pose
        Eigen::MatrixXd J_t; 
        J_t.setZero();
        
        // Need to have all the states in a vector format.
        // Current pose, this is already predicted.
        Eigen::VectorXd pred_s = ndt_generic::affine3dToVector(this->T_);

        // Compute the difference between the predicted state and the sigma points
        std::vector<Eigen::VectorXd> s_diffs;
        for (int i = 0; i < 2*N_+1; i++) {
            s_diffs.push_back(this->sigmas_[i] - pred_s); // This need to be specifically checked to avoid 2*M_PI overflows.
        }
        
        for (int i = 0; i < 2*this->N_+1; i++) {
            J_t += this->getWeightCov(i)*diffs[i]*(s_diffs[i].transpose());
        }
        
        Eigen::MatrixXd K_t = J_t.transpose() * S.inverse();
    
        // Update the state
        Eigen::VectorXd new_state = pred_s + K_t*(raw_ranges - mean);
        T_ = ndt_generic::vectorToAffine3d(new_state);
        
        P_ = P_ - K_t * S * K_t.transpose();
    }
    

};

/**
 * NDT UKF 3D - Class implementation
 */
class NDTUKF3D{
public:
    lslgeneric::NDTMap map;
    double resolution;
    double resolution_sensor;
    int counter;
    std::vector<double> motion_model, motion_model_offset;
    
    /**
     * Constructor
     */
    NDTUKF3D(double map_resolution, lslgeneric::NDTMap &nd_map):
        map(new lslgeneric::LazyGrid(map_resolution))
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

        std::vector<lslgeneric::NDTCell*> ndts;
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
};


