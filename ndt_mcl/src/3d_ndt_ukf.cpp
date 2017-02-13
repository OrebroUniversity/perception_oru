#include <ndt_mcl/3d_ndt_ukf.h>

// Point cloud processing utils... 
void computeDirectionsAndRangesFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Vector3d &origin, std::vector<Eigen::Vector3d> &dirs, std::vector<double> &ranges) {
    ranges.reserve(cloud.size());
    dirs.reserve(cloud.size());
    pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud.points.begin();

    while (it != cloud.points.end()) {
        if(std::isnan(it->x) ||std::isnan(it->y) ||std::isnan(it->z))
        {
            it++;
            continue;
        }
        
        Eigen::Vector3d diff;
        diff << it->x-origin(0), it->y-origin(1), it->z-origin(2);
        ranges.push_back(diff.norm());

        diff.normalize();
        dirs.push_back(diff);

        it++;
    }
}


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

    this->predict(Tmotion);


#if 0
   
    Eigen::MatrixXd pred_ranges;
    Eigen::VectorXd raw_ranges;


    ROS_INFO("updateAndPredict : compute measurments");

    this->computeMeasurements(cloud, Tsensor, pred_ranges, raw_ranges);

    ROS_INFO("updateAndPredict : compute measurments - completed");

    ROS_INFO("udpateAndPredict : update");

    ukf_.update(pred_ranges, raw_ranges);

    ROS_INFO("udpateAndPredict : update- completed");
#endif
}


// The cloud should be given in the vehicle's frame
void NDTUKF3D::computeMeasurements(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tsensor, Eigen::MatrixXd &measurements, Eigen::VectorXd &raw_ranges)
{
    std::vector<Eigen::Vector3d> dirs;
    std::vector<double> r;
    computeDirectionsAndRangesFromPointCloud(cloud, Tsensor.translation(), dirs, r);

    ROS_INFO_STREAM("r.size() : " << r.size());
    ROS_INFO_STREAM("dirs.size() : " << dirs.size());

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
