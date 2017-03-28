#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ndt_mcl/3d_ndt_ukf.h>

#include <iostream>
#include <iomanip>

using namespace std;

int main()
{
    {
        UKF3D ukf;

        std::cout << "ukf.getLambda() : " << ukf.getLambda() << std::endl;
        std::cout << "ukf.getXsi() : " << ukf.getXsi() << std::endl;

        for(int i = 0; i < ukf.getNbSigmaPoints(); i++) {
            std::cout << "mean weight [" << i << "]:" << ukf.getWeightMean(i) << std::endl;
        }

        std::cout << "####################################################" << std::endl;

        UKF3D::Params params;
        params.alpha = 0.4;
        params.beta = 2.;
        params.kappa = 3.;
        ukf.setParams(params);

        std::cout << "ukf.getLambda() : " << ukf.getLambda() << std::endl;
        std::cout << "ukf.getXsi() : " << ukf.getXsi() << std::endl;

        for(int i = 0; i < ukf.getNbSigmaPoints(); i++) {
            std::cout << "mean weight [" << i << "]:" << ukf.getWeightMean(i) << std::endl;
        }

        Eigen::VectorXd x(6);
        x << 1, 2, 3, 0.0, 0.0, 0.0;
        std::cout << "x: " << x << std::endl;
        Eigen::Affine3d T = ndt_generic::vectorToAffine3d(x);

        Eigen::MatrixXd cov(6,6);
        cov.setZero();
        cov(0,0) = 0.1; cov(1,1) = 0.1; cov(2,2) = 0.1;
        cov(3,3) = 0.02; cov(4,4) = 0.02; cov(5,5) = 0.02;
    
        //ukf.initializeFilter(T, cov);
        ukf.assignSigmas(x, cov);

        std::cout << ukf.getDebugString() << std::endl;


        // Eigen::VectorXd mean = ukf.computePoseMean();
        // std::cout << "mean : " <<  ukf.computePoseMean().transpose() << std::endl;

        Eigen::VectorXd mean(6);
        std::vector<Eigen::Affine3d> T_sigmas = ukf.getSigmasAsAffine3d();
        std::vector<double> weights = ukf.getMeanWeights();
        Eigen::Affine3d T2 = ndt_generic::getAffine3dMean(T_sigmas);
        Eigen::Affine3d T3 = ndt_generic::getAffine3dMeanWeights(T_sigmas, weights);
        Eigen::Affine3d T4 = ndt_generic::getAffine3dMeanWeightsUsingQuat(T_sigmas, weights);
        Eigen::Affine3d T5 = ndt_generic::getAffine3dMeanWeightsUsingQuatNaive(T_sigmas, weights);

        std::cout << "T  -> vec : " << ndt_generic::affine3dToStringRPY(T) << std::endl;
        std::cout << "T2 -> vec : " << ndt_generic::affine3dToStringRPY(T2) << std::endl;
        std::cout << "T3 -> vec : " << ndt_generic::affine3dToStringRPY(T3) << std::endl;
        std::cout << "T4 -> vec : " << ndt_generic::affine3dToStringRPY(T4) << std::endl;
        std::cout << "T5 -> vec : " << ndt_generic::affine3dToStringRPY(T5) << std::endl;
    
        std::cout << " T : " << ndt_generic::affine3dToStringRotMat(T) << std::endl;
        std::cout << " T2: " << ndt_generic::affine3dToStringRotMat(T2) << std::endl;
        std::cout << " T3: " << ndt_generic::affine3dToStringRotMat(T3) << std::endl;
        std::cout << " T4: " << ndt_generic::affine3dToStringRotMat(T4) << std::endl;
        std::cout << " T5: " << ndt_generic::affine3dToStringRotMat(T5) << std::endl;
    
        std::cout << ukf.getDebugString() << std::endl;

        Eigen::VectorXd incr(6);
        incr << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        Eigen::Affine3d T_incr = ndt_generic::vectorToAffine3d(incr);

        std::cout << "T_incr : " << ndt_generic::affine3dToStringRotMat(T_incr) << std::endl;

        for (int i = 0; i< 5; i++) {
            std::cout << "------------------------------" << std::endl;
            std::cout << ukf.getDebugString(); 
            ukf.predict(T_incr, cov);
            mean = ukf.computePoseMean(); 
        
            std::cout << "mean[" << i << "]: " <<  mean.transpose() << std::endl;
        }
    }

    // {
    //     UKF3D ukf;

    //     ukf.setParams(0.4, 2., 3);
        
    //     Eigen::VectorXd x(6), x2(6);
    //     x << 1, 2, 3, 0.0, 0.0, 0.0;

    //     Eigen::MatrixXd cov(6,6), cov2(6,6);
    //     cov.setZero();
    //     cov(0,0) = 0.1; cov(1,1) = 0.1; cov(2,2) = 0.1;
    //     cov(3,3) = 0.02; cov(4,4) = 0.02; cov(5,5) = 0.02;
        
    //     for (int i = 1; i < 10; i++) {
    //         double alpha = i*0.01;
    //         double kappa = i;
    //         ukf.setParams(alpha, 2., kappa);
    //         std::cout << "x: " << x << std::endl;
    //         std::cout << "cov : " << cov << std::endl;
    //         ukf.assignSigmas(x, cov);
            
    //         x2 = ukf.computePoseMean();
    //         cov2 = ukf.computePoseCov(x2);

    //         std::cout << "----------------------------" << std::endl;
    //         std::cout << "x2: " << x2 << std::endl;
    //         std::cout << "cov2 : " << cov2 << std::endl;
    //     }
    // }
    {
        UKF3D ukf;

        UKF3D::Params params;
        params.alpha = 0.1;
        params.beta = 2.;
        params.kappa = 3.;
        ukf.setParams(params);
        
        Eigen::VectorXd x(6), x2(6);
        x << 1, 2, 3, 0.0, 0.0, 0.0;

        Eigen::MatrixXd cov(6,6), cov2(6,6);
        cov.setZero();
        cov(0,0) = 10; cov(1,1) = 1; cov(2,2) = 0.1;
        cov(3,3) = 0.02; cov(4,4) = 0.02; cov(5,5) = 0.02;
        
        std::cout << "x: " << x << std::endl;
        std::cout << "cov : " << cov << std::endl;
        ukf.assignSigmas(x, cov);
        
        x2 = ukf.computePoseMean();
        cov2 = ukf.computePoseCov(x2);
        
        std::cout << "----------------------------" << std::endl;
        std::cout << "x2: " << x2 << std::endl;
        std::cout << "cov2 : " << cov2 << std::endl;
    }
}
