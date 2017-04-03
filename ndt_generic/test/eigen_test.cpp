#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ndt_generic/eigen_utils.h>

#include <iostream>
#include <iomanip>

using namespace std;

int main()
{
    // Simple check of the normalization
    {
        Eigen::VectorXd x(6);
        x << 1, 2, 3, 0.1, 0.2, 2.7;
        
        std::cout << "x: " << x << std::endl;
        
        Eigen::Affine3d T = ndt_generic::vectorToAffine3d(x);
        
        std::cout << "T : " << ndt_generic::affine3dToStringRPY(T) << std::endl;
        std::cout << "T.rotation() : " << T.rotation() << std::endl;
        
        x = ndt_generic::affine3dToVector(T);
        
        std::cout << "x: " << x << std::endl;
        
        std::cout << "------------------------- The two T matrices should be the same below ------------------" << std::endl;
        
        x(3) += M_PI;
        
        std::cout << "x : " << x << std::endl;
        T = ndt_generic::vectorToAffine3d(x);
        std::cout << "T : " << ndt_generic::affine3dToStringRPY(T) << std::endl;
        std::cout << "T.rotation() : " << T.rotation() << std::endl;
        
        
        ndt_generic::normalizeEulerAngles6dVec(x);
        
        std::cout << "x : " << x << std::endl;
        T = ndt_generic::vectorToAffine3d(x);
        std::cout << "T : " << ndt_generic::affine3dToStringRPY(T) << std::endl;
        std::cout << "T.rotation() : " << T.rotation() << std::endl;
    }
    
    // Test the fusion
    {
        std::cout << "---------------------------------------------" << std::endl;
        Eigen::Matrix3d covA;
        covA.setIdentity();

        Eigen::Matrix3d covB;
        covB.setIdentity();

        Eigen::Vector3d a(1,2,3);
        Eigen::Vector3d b(0,1,0);

        Eigen::Vector3d weighted = ndt_generic::getWeightedPoint(a,covA,b,covB);

        std::cout << "weighted : " << ndt_generic::getWeightedPoint(a,covA,b,covB);
        std::cout << "should be : 0.5, 1.5, 1.5" << std::endl;
        covB *= 10.;
        std::cout << "weighted : " << ndt_generic::getWeightedPoint(a,covA,b,covB);
        std::cout << "should be : 0.9091, 1.9091, 2.7273" << std::endl;
    }
    
    {
        std::cout << "---------------------------------------------" << std::endl;
        Eigen::Affine3d a = Eigen::Translation<double,3>(1,2,3)*
        Eigen::AngleAxis<double>(0.1,Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxis<double>(0.2,Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxis<double>(0.3,Eigen::Vector3d::UnitZ()) ;

        Eigen::Affine3d b = Eigen::Translation<double,3>(0,1,0)*
        Eigen::AngleAxis<double>(0.0,Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxis<double>(0.1,Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxis<double>(0.0,Eigen::Vector3d::UnitZ()) ;
        
        Eigen::MatrixXd covA(6,6);
        covA.setIdentity();
        
        Eigen::MatrixXd covB(6,6);
        covB.setIdentity();
        
        std::cout << "weighted : " << ndt_generic::affine3dToStringRPY(ndt_generic::getWeightedPose(a, covA, b, covB)) << std::endl;        
    }


}
