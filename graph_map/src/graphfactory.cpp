#include "graphfactory.h"
#include "graph_map/factor.h"
#include "graph_map/map_type.h"
#include "ndt/ndt_map_param.h"
#include "ndt/ndt_map_type.h"
#include "graph_map/map_node.h"
#include "graph_map/graph_map.h"
#include "graph_map/graph_map_navigator.h"
#include "graph_map/reg_type.h"
#include "ndt_dl/ndtdl_reg_type.h"
#include "ndt/ndtd2d_reg_type.h"
#include "Eigen/Geometry"
#include "template/template_map_type.h"
#include "template/template_reg_type.h"
#include "ndt_dl/ndtdl_map_type.h"



namespace libgraphMap{



MapParamPtr GraphFactory::CreateMapParam(string mapname){
  if(mapname.compare(ndt_map_type_name)==0){
    cout<<"Graphfactory: Created parameters for map type: \""<<ndt_map_type_name<<"\""<<endl;
    NDTMapParamPtr paramPtr(new NDTMapParam());
    return paramPtr;
  }
  else if(mapname.compare(ndtdl_map_type_name)==0){
    cout<<"Graphfactory: Created parameters for map type: \""<<ndtdl_map_type_name<<"\""<<endl;
    NDTDLMapParamPtr paramPtr(new NDTDLMapParam());
    return paramPtr;
  }
  else if(mapname.compare(template_map_name)==0){
    TemplateMapParamPtr templatePar(new TemplateMapParam());
    cerr<<"Graphfactory:  Template, no map parameter instance created"<<endl;
    return templatePar;
  }
  else{
    cerr<<"No map type exists with name: \""<<mapname<<"\""<<endl;
    return NULL;
  }
}

MapTypePtr GraphFactory::CreateMapType(MapParamPtr mapparam){
  if(  NDTMapParamPtr ndt2MapParam = boost::dynamic_pointer_cast< NDTMapParam >(mapparam) ){ //perform typecast and check if not null conversion
    cout<<"Graphfactory: Created map of type: \""<<ndt_map_type_name<<"\""<<endl;
    return  MapTypePtr(new NDTMapType(ndt2MapParam));
  }
  else if(  NDTDLMapParamPtr ndtdlMapParam = boost::dynamic_pointer_cast< NDTDLMapParam >(mapparam) ){ //perform typecast and check if not null conversion
    cout<<"Graphfactory: Created map of type: \""<<ndtdl_map_type_name<<"\""<<endl;
    return  MapTypePtr(new NDTDL(ndtdlMapParam));
  }
  else if(TemplateMapParamPtr template_par=boost::dynamic_pointer_cast<TemplateMapParam>(mapparam)){
    cerr<<"Graphfactory: no map exists for \"template\""<<endl;
    return NULL;
  }
  else{
    cerr<<"Graphfactory: No map type exists for map parameters"<<endl;
    return NULL;
  }

}

GraphMapPtr GraphFactory::CreateGraph(const Eigen::Affine3d &nodepose, MapParamPtr &mapparam, GraphParamPtr graphparam){

  if(mapparam!=NULL){
    cout<<"Graphfactory: Creating graph"<<endl;
    GraphMapPtr graphPtr=GraphMapPtr(new GraphMapNavigator(nodepose,mapparam,graphparam));
    return graphPtr;
  }
  else{
    cout<<"Graphfactory: parameter NULL to graph"<<endl;
    return NULL;
  }
}
GraphMapNavigatorPtr GraphFactory::CreateGraphNavigator(const Eigen::Affine3d &nodepose, MapParamPtr &mapparam, GraphParamPtr graphparam){
  if(mapparam!=NULL){
    cout<<"Graphfactory: Creating graph"<<endl;
    return GraphMapNavigatorPtr(new GraphMapNavigator(nodepose,mapparam,graphparam));
  }
  else{
    cout<<"Graphfactory: parameter NULL to graph"<<endl;
    return NULL;
  }
}
GraphParamPtr GraphFactory::CreateGraphParam(){
  cout<<"Graphfactory: Creating graph parameters"<<endl;
  return boost::shared_ptr<GraphParam>(new GraphParam());
}

MapNodePtr GraphFactory::CreateMapNode(const Eigen::Affine3d &pose,const MapParamPtr &mapparam){//Create a node
  cout<<"Graphfactory: Creating map node"<<endl;
  return boost::shared_ptr<MapNode>(new MapNode(pose,mapparam));
}


RegTypePtr GraphFactory::CreateRegistrationType(const Affine3d &sensor_pose,RegParamPtr regparam){


  if(NDTD2DRegParamPtr ndt_reg_ptr=boost::dynamic_pointer_cast<NDTD2DRegParam>(regparam)){
    cout<<"Graphfactory: created registration type:"<<ndt_d2d_reg_type_name<<endl;
    return NDTD2DRegTypePtr(new NDTD2DRegType(sensor_pose,ndt_reg_ptr));
  }

  else if(  NDTDLRegTypeParamPtr ndt_reg_ptr=boost::dynamic_pointer_cast<NDTDLRegTypeParam>(regparam) ){ //perform typecast and check if not null conversion
    cout<<"Graphfactory: created registration type:"<<ndt_dl_reg_type_name<<endl;
    return NDTDLRegTypePtr(new NDTDLRegType(sensor_pose,ndt_reg_ptr));
  }



  cerr<<"Failed to create object of registration type"<<endl;
  return NULL;
}
RegParamPtr GraphFactory::CreateRegParam(string regType){
  if(regType.compare(ndt_d2d_reg_type_name)==0){
    cout<<"Graphfactory: Creating parameters for registration type: \""<<ndt_d2d_reg_type_name<<"\""<<endl;
    return NDTD2DRegParamPtr(new NDTD2DRegParam());
  }
  else if(regType.compare(ndt_dl_reg_type_name)==0){
    cout<<"Graphfactory: Creating parameters for registration type: \""<<ndtdl_map_type_name<<"\""<<endl;
    return NDTDLRegTypeParamPtr(new NDTDLRegTypeParam());
  }
  else{
    cerr<<"No registration type with name: \""<<regType<<"\""<<endl;
    return NULL;
  }
}

FactorPtr GraphFactory::CreateMapNodeFactor(MapNodePtr prevMapPose, MapNodePtr nextMapPose, const Eigen::Affine3d &diff, const Matrix6d &covar){
  FactorPtr factorptr=boost::shared_ptr<factor>(new factor(prevMapPose,nextMapPose,diff,covar));
  return factorptr;
}

}
