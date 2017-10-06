#include "graph_map/template/template_reg_type.h"
namespace perception_oru{
namespace libgraphMap{


TemplateRegType::TemplateRegType(const Affine3d &sensor_pose,RegParamPtr paramptr):registrationType(sensor_pose,paramptr){

  TemplateRegTypeParamPtr param = boost::dynamic_pointer_cast< TemplateRegTypeParam >(paramptr);//Should not be NULL
  if(param!=NULL){
    //Transfer all parameters from param to this class
    cout<<"Created registration type for template"<<endl;
  }
  else
    cerr<<"ndtd2d registrator has NULL parameters"<<endl;
}

TemplateRegType::~TemplateRegType(){}

bool TemplateRegType::Register(MapTypePtr maptype, Eigen::Affine3d &Tnow, pcl::PointCloud<pcl::PointXYZ> &cloud, Matrix6d cov) {

  if(!enableRegistration_||!maptype->Initialized()){
    cout<<"Registration disabled - motion based on odometry"<<endl;

    return false;
  }
  else{
    TemplateMapTypePtr MapPtr = boost::dynamic_pointer_cast< TemplateMapType >(maptype);
    //Perform registration based on prediction "Tinit", your map "MapPtr" and the "cloud"
    cout<<"please fill in code for registration- uintil then, registration is disabled"<<endl;
  }
  return false;//remove this when registration code has been implemented

}






/* ----------- Parameters ------------*/
TemplateRegTypeParam::~TemplateRegTypeParam(){}
TemplateRegTypeParam::TemplateRegTypeParam():registrationParameters(){
}
void TemplateRegTypeParam::GetParametersFromRos(){
  registrationParameters::GetParametersFromRos();
  ros::NodeHandle nh("~");//base class parameters
  nh.param<std::string>("super_important_parameter",super_important_parameter_,"default string");

}

}

}//end namespace

