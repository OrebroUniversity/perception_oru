#include "ndt/ndt_map_type.h"
#include <boost/serialization/export.hpp>
BOOST_CLASS_EXPORT(libgraphMap::NDTMapType)
namespace libgraphMap{
  using namespace std;

  using namespace lslgeneric;

  NDTMapType::NDTMapType( MapParamPtr paramptr) : MapType(paramptr){
    NDTMapParamPtr param = boost::dynamic_pointer_cast< NDTMapParam >(paramptr);//Should not be NULL
    if(param!=NULL){
      resolution_=param ->resolution_;
      map_ = new lslgeneric::NDTMap(new lslgeneric::LazyGrid(resolution_));
      map_->initialize(0.0,0.0,0.0,param->sizex_,param->sizey_,param->sizez_);
      cout<<"created ndt2dmap"<<endl;
    }
    else
      cerr<<"Cannot create instance of NDTmapHMT"<<std::endl;
  }
  NDTMapType::~NDTMapType(){}

  void NDTMapType::update(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud){//update map, cloud is the scan, Tsensor is the pose where the scan was aquired.

    if(initialized_ && enable_mapping_){
      Eigen::Vector3d localMapSize(max_range_,max_range_,sizez_);
      map_->addPointCloudMeanUpdate(Tsensor.translation(),cloud,localMapSize, 1e5, 25, 2*sizez_, 0.06);
    }
    else if(!initialized_){
      InitializeMap(Tsensor,cloud);
      initialized_ = true;
    }
  }
  void NDTMapType::InitializeMap(const Eigen::Affine3d &Tsensor,pcl::PointCloud<pcl::PointXYZ> &cloud){
    cout<<"initialize map"<<endl;
    map_->addPointCloud(Tsensor.translation(),cloud, 0.1, 100.0, 0.1);
    map_->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE, 1e5, 255, Tsensor.translation(), 0.1);
  }
  bool NDTMapType::CompoundMapsByRadius(MapTypePtr target,const Affine3d &T_source,const Affine3d &T_target, double radius){

    Affine3d Tdiff=Affine3d::Identity();
    Tdiff=T_source.inverse()*T_target;
    pcl::PointXYZ center_pcl(Tdiff.translation()(0),Tdiff.translation()(1),Tdiff.translation()(2));
    if( NDTMapPtr targetPtr=boost::dynamic_pointer_cast<NDTMapType>(target) ){
      cout<<"dynamic casted pointer"<<endl;
      if(resolution_!=targetPtr->resolution_)//checking if source and target have same resolution, they shoould have.
        return false;

      if(radius==-1)//if radius is not defined, match rcenter_pcladius to size of new map
        radius=targetPtr->sizex_<targetPtr->sizey_? targetPtr->sizex_/2:targetPtr->sizey_/2;

      int neighboors=radius/resolution_;
      cout<<"neighboors cells to search through="<<neighboors<<endl;
      std::vector<NDTCell*>cells= map_->getCellsForPoint(center_pcl,neighboors,true);
      cout<<"cells to transfer:"<<cells.size()<<endl;
      Tdiff=T_source.inverse()*T_target;
      cout<<"centerpoint in prev map frame=\n"<<Tdiff.translation()<<endl;

      for(int i=0;i<cells.size();i++){
        Eigen::Matrix3d cov=Tdiff.inverse().linear()*cells[i]->getCov()*Tdiff.linear();
        Eigen::Vector3d mean=Tdiff.inverse()*cells[i]->getMean();
        targetPtr->GetMap()->addDistributionToCell(cov,mean,cells[i]->getN());
      }

    }

  }
  std::string NDTMapType::ToString(){
    stringstream ss;
    ss<<MapType::ToString()<<"NDT Map Type:"<<endl;
    ss<<"resolution:"<<resolution_<<endl;
    ss<<"resolution local factor:"<<resolution_local_factor_<<endl;
    ss<<"maximum sensor range:"<<sensor_range_<<endl;
    return ss.str();
  }

}
