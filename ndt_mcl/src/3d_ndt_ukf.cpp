#include <ndt_mcl/3d_ndt_ukf.h>
#include <pcl_ros/impl/transforms.hpp>
#include <ndt_generic/pcl_utils.h>
#include <ndt_generic/eigen_utils.h>

Eigen::VectorXd UKF3D::computePoseMean() const {
        Eigen::VectorXd ret(N_);
        ret.setZero();

        std::vector<double> weights = getMeanWeights();
        std::vector<Eigen::Affine3d> T_sigmas = getSigmasAsAffine3d();
        
        Eigen::Affine3d T_mean = ndt_generic::getAffine3dMeanWeightsUsingQuatNaive(T_sigmas, weights);
        
        ret = ndt_generic::affine3dToVector(T_mean);
        
        ndt_generic::normalizeEulerAngles6dVec(ret);
        return ret;
    }

Eigen::MatrixXd UKF3D::computePoseCov(const Eigen::VectorXd &mean) const
 {
        Eigen::MatrixXd ret(N_,N_);
        ret.setZero();
        assert(mean.size() != N_);

        for (int i = 0; i < 2*N_+1; i++) {

            Eigen::VectorXd diff = mean - this->getSigma(i);
            
            // Normalize the diff angles...
            diff(4) = angles::normalize_angle(diff(4));
            diff(5) = angles::normalize_angle(diff(5));
            diff(6) = angles::normalize_angle(diff(6));

            ret += getWeightCov(i) * diff * diff.transpose();

            if (fabs(diff(4)) > M_PI/2.) {
                std::cout << "diff way to large" << std::endl;
                std::cout << "diff : " << diff << std::endl;
            }
            if (fabs(diff(5)) > M_PI/2.) {
                std::cout << "diff way to large" << std::endl;
                std::cout << "diff : " << diff << std::endl;
            }
            if (fabs(diff(5)) > M_PI/2.) {
                std::cout << "diff way to large" << std::endl;
                std::cout << "diff : " << diff << std::endl;
            }


        }


        // This don't work since the x,y,z will be dependent on the angles.
        // std::vector<Eigen::Affine3d> T_sigmas = this->getSigmasAsAffine3d();
        // Eigen::Affine3d T_mean = ndt_generic::vectorToAffine3d(mean);

        // for (int i = 0; i < T_sigmas.size(); i++) {
        //     Eigen::Affine3d T_diff = T_sigmas[i]*T_mean.inverse();
        //     Eigen::VectorXd diff = ndt_generic::affine3dToVector(T_diff);

        //     // diff(4) = angles::normalize_angle(diff(4));
        //     // diff(5) = angles::normalize_angle(diff(5));
        //     // diff(6) = angles::normalize_angle(diff(6));

        //     ndt_generic::normalizeEulerAngles6dVec(diff);

        //     std::cout << "diff [" << i << "] : " << diff.transpose() << std::endl;

        //     ret += getWeightCov(i) * diff * diff.transpose();
        // }

        return ret;
    }


std::vector<double> UKF3D::getMeanWeights() const {
        std::vector<double> ret;
        
        for (int i = 0; i < 2*N_+1; i++) {
            ret.push_back(getWeightMean(i));
        }
        return ret;
    }

std::vector<double> UKF3D::getCovWeights() const {
        std::vector<double> ret;
        
        for (int i = 0; i < 2*N_+1; i++) {
            ret.push_back(getWeightCov(i));
        }
        return ret;
    }


double UKF3D::getWeightMean(int idx) const {
        const double lambda = this->getLambda();
        if (idx == 0) {
            return (lambda / (N_ + lambda)); 
        }
        return 1./(2*(N_ + lambda));
    }

double UKF3D::getWeightCov(int idx) const {
        const double lambda = this->getLambda();
        if (idx == 0) {
            return lambda/(N_ + lambda) + (1 - params_.alpha*params_.alpha + params_.beta);
        }
        return 1./(2*(N_ + lambda));
    }


std::vector<Eigen::Affine3d> UKF3D::getSigmasAsAffine3d() const {
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

void UKF3D::assignSigmas(const Eigen::VectorXd &mean, const Eigen::MatrixXd &cov) {

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

        P_ = cov;
    }

void UKF3D::predict(const Eigen::Affine3d &incr, const Eigen::MatrixXd &incrCov) {
        
        // Take the current estimate with cov and compute the sigma points.
        Eigen::VectorXd mean = ndt_generic::affine3dToVector(T_);
        // Assign the new sigma points
        assignSigmas(mean, P_);

        // Add the incremental pose to all sigmas (noise could be added here).
        for (int i = 0; i < sigmas_.size(); i++) {
            Eigen::Affine3d s = ndt_generic::vectorToAffine3d(sigmas_[i]) * incr;
            sigmas_[i] = ndt_generic::affine3dToVector(s);
        }
        
        // Compute the new mean and cov
        mean = this->computePoseMean();
        T_ = ndt_generic::vectorToAffine3d(mean);

        std::cout << "P_ old [pred] : " << P_ << std::endl;
        P_ = this->computePoseCov(mean) + incrCov;
        std::cout << "P_ new [pred] : " << P_ << std::endl;

        assignSigmas(mean, P_);
    }

void UKF3D::update(const Eigen::MatrixXd &pred_ranges,
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
        Q *= params_.range_var;  // Range variance.

        Eigen::MatrixXd S(raw_ranges.size(), raw_ranges.size());
        S.setZero();
        
        // Keep the range differences
        std::vector<Eigen::VectorXd> diffs;

        double weight_c0 = this->getWeightCov(0);
        double weight_c = this->getWeightCov(1);

        for (int i = 0; i < 2*this->N_+1; i++)  {
            Eigen::VectorXd diff = pred_ranges.row(i).transpose() - raw_ranges;
            if (i == 0) {
                S += weight_c0 * diff * diff.transpose();
            }
            else {
                S += weight_c * diff * diff.transpose();
            }
            diffs.push_back(diff);
        }
        S += Q;

        // Correct the pose
        Eigen::MatrixXd J_t(raw_ranges.size(), 6); 
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

        // The inversion is very costly...
        Eigen::MatrixXd K_t = J_t.transpose() * S.colPivHouseholderQr().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));

        // Update the state
        Eigen::VectorXd new_state = pred_s + K_t*(raw_ranges - mean);
        // std::cout << "pred state : " << pred_s << std::endl;
        // std::cout << "New state : " << new_state << std::endl;
        // std::cout << "K_t*(raw_ranges - mean) : " <<  K_t*(raw_ranges - mean) << std::endl;
        // std::cout << "raw_ranges - mean : " << raw_ranges - mean << std::endl;

        T_ = ndt_generic::vectorToAffine3d(new_state);
        
        // std::cout << "K_t * S * K_t.transpose() : " << K_t * S * K_t.transpose() << std::endl;
        std::cout << "P_ old [update] : " << P_ << std::endl;
        P_ = P_ - K_t * S * K_t.transpose();
        std::cout << "P_ new [update] : " << P_ << std::endl;
    }



void UKF3D::assignMinDiagVariances() {
        double min_var;
        for (int i = 0; i < N_; i++) {
            if (i < 3)
                min_var = params_.min_pos_var;
            else
                min_var = params_.min_rot_var;

            if (P_(i,i) < min_var) {
                P_(i,i) = min_var;
            }
        }
    }

std::vector<int> UKF3D::filterMeasurements(Eigen::MatrixXd &pred_ranges, Eigen::VectorXd &raw_ranges,
                                        Eigen::MatrixXd &filter_pred_ranges, Eigen::VectorXd &filter_raw_ranges) const {
        
        // Check the difference between the measured ones and the predicted.
        // If the difference is > 1 meter then skip this measurement
        std::vector<int> idx;
        for (int i = 0; i < raw_ranges.size(); i++) {
            bool use = true;
            for (int j = 0; j < 2*this->N_+1; j++) {
                double diff = fabs(pred_ranges(j,i) - raw_ranges(i));
                if (diff > params_.range_filter_max_dist) {
                    use = false;
                }
            }
            if (use) {
                idx.push_back(i);
            }
        }

        filter_pred_ranges.resize(2*N_+1, idx.size());
        filter_raw_ranges.resize(idx.size());
        for (int i = 0; i < idx.size(); i++) {
            filter_raw_ranges[i] = raw_ranges[idx[i]];
            filter_pred_ranges.col(i) = pred_ranges.col(idx[i]); // Row is per sigma point
        }
        
        return idx;
    }


//-------------

void NDTUKF3D::predict(Eigen::Affine3d Tmotion) {

    // Use the motion model to compute the covariance of the increment in a local frame...
    Eigen::Vector3d tr = Tmotion.translation();
    Eigen::Vector3d rot = Tmotion.rotation().eulerAngles(0,1,2);
    
    Eigen::Matrix<double, 6,6> Q;
    Q.setIdentity();

    Eigen::Matrix<double, 6,6> motion_model_m(motion_model.data());
    Eigen::Matrix<double,6,1> incr;
    incr << fabs(tr[0]),fabs(tr[1]),fabs(tr[2]), fabs(rot[0]), fabs(rot[1]), fabs(rot[2]);
    Eigen::Matrix<double,6,1> m = motion_model_m*incr; 
    for (int i = 0; i < 6; i++) {
        Q(i,i) = m(i) + motion_model_offset[i];
    }
       
    Q.setZero();

    ukf_.predict(Tmotion, Q);
} 


void NDTUKF3D::updateAndPredict(Eigen::Affine3d Tmotion, pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tsensor){

    ukf_.assignMinDiagVariances();
    this->predict(Tmotion);

    Eigen::MatrixXd pred_ranges, filter_pred_ranges;
    Eigen::VectorXd raw_ranges, filter_raw_ranges;

    this->computeMeasurements(cloud, Tsensor, pred_ranges, raw_ranges);

    ukf_.filterMeasurements(pred_ranges, raw_ranges,
                            filter_pred_ranges, filter_raw_ranges);

    //    ukf_.update(pred_ranges, raw_ranges);
    ukf_.update(filter_pred_ranges, filter_raw_ranges);
}


// The cloud should be given in the vehicle's frame
void NDTUKF3D::computeMeasurements(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tsensor, Eigen::MatrixXd &measurements, Eigen::VectorXd &raw_ranges)
{
    std::vector<Eigen::Vector3d> dirs;
    std::vector<double> r;
    ndt_generic::computeDirectionsAndRangesFromPointCloud(cloud, Tsensor.translation(), dirs, r);

    raw_ranges = Eigen::VectorXd::Map(r.data(), r.size());

    // Cloud is given in the vehicle frame. Hence the cloud should / cloud be tranformed around using the sigma points. One option is instead of rotating the points around simply update the directional vectors (the translation is instead only added to the pose to be computed).

    std::vector<Eigen::Affine3d> sigmas = ukf_.getSigmasAsAffine3d(); // These are in vehicle frames...

    measurements.resize(sigmas.size(), raw_ranges.size());

    std::vector<Eigen::Vector3d> d(dirs.size());
    
    for (int i = 0; i < sigmas.size(); i++) {
        Eigen::Matrix3d rot = sigmas[i].linear();
     
        for (int j = 0; j < d.size(); j++) {
            d[j] = rot * dirs[j];
        }
        Eigen::Vector3d origin = (sigmas[i]*Tsensor).translation();
        Eigen::VectorXd ranges;

        map.computeMaximumLikelihoodRanges(origin, raw_ranges, d, ranges);
        measurements.row(i) = ranges;
    }

}


