#include "mcl_ndt/mcl_ndt.h"
namespace GraphMapLocalisation{
MCLNDTType::~MCLNDTType(){}
MCLNDTType::MCLNDTType(LocalisationParamPtr param):LocalisationType(param){


}


void GraphMapLocalisation::UpdateAndPredict(pcl::PointCloud<pcl::PointXYZ> &cloud, const Eigen::Affine3d &Tmotion){

}


MCLNDTParam::~MCLNDTParam(){}
MCLNDTParam::MCLNDTParam(){}
}
