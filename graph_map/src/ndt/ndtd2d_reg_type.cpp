#include "ndt/ndtd2d_reg_type.h"
namespace libgraphMap{


NDTD2DRegType::NDTD2DRegType(const Affine3d &sensor_pose, RegParamPtr paramptr):registrationType(sensor_pose,paramptr){

  NDTD2DRegParamPtr param_ptr = boost::dynamic_pointer_cast< NDTD2DRegParam >(paramptr);//Should not be NULL
  if(param_ptr!=NULL){
    resolution_=param_ptr->resolution_;
    resolutionLocalFactor_=param_ptr->resolutionLocalFactor_;
    //NDTMatcherD2D_2D parameters
    matcher2D_.ITR_MAX = param_ptr->matcher2D_ITR_MAX;
    matcher2D_.step_control=param_ptr->matcher2D_step_control;
    matcher2D_.n_neighbours=param_ptr->matcher2D_n_neighbours;
    cout<<"succesfully applied ndt d2d registration parameters"<<endl;
  }
  else
    cerr<<"ndtd2d registrator has NULL parameters"<<endl;
}

NDTD2DRegType::~NDTD2DRegType(){}


bool NDTD2DRegType::RegisterMap2Map(MapTypePtr map_prev,MapTypePtr map_next, Eigen::Affine3d &Tdiff,double match_score){

  NDTMap *prev,*next;
  bool match_succesfull;
  prev=(boost::dynamic_pointer_cast<NDTMapType>(map_prev))->GetNDTMap();
  next=(boost::dynamic_pointer_cast<NDTMapType>(map_next))->GetNDTMap();
  if(registration2d_){
    match_succesfull=matcher2D_.match(*prev,*next,Tdiff,true);
  }
  else if(!registration2d_){
    match_succesfull=matcher3D_.match( *prev,*next,Tdiff,true);
  }
  cout<<"diff after reg-trans=\n"<<Tdiff.translation()<<endl;
  cout<<"diff after reg-rot=\n="<<Tdiff.linear()<<endl;
  return match_succesfull;
}
std::string NDTD2DRegType::ToString(){
  std::stringstream ss;
  ss<<registrationType::ToString();
  if(enableRegistration_){
    ss<<"NDT d2d registration type:"<<endl;
    ss<<"resolution :"<< resolution_<<endl;
    ss<<"resolutionLocalFactor :"<< resolutionLocalFactor_<<endl;
  }
  return ss.str();
}

/* ----------- Parameters ------------*/
NDTD2DRegParam::~NDTD2DRegParam(){}
NDTD2DRegParam::NDTD2DRegParam():registrationParameters(){}
void NDTD2DRegParam::GetParametersFromRos(){
  registrationParameters::GetParametersFromRos();
  cout<<"derived class read from ros"<<endl;
  ros::NodeHandle nh("~");//base class parameters
  nh.param("resolution",resolution_,0.4);
  nh.param("resolutionLocalFactor",resolutionLocalFactor_,1.0);
}



}//end namespace

