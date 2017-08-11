#include "graph_localisation/localisation_factory.h"//must be included first
#include "graph_localisation/localisation_type.h"
#include "mcl_ndt/mcl_ndt.h"
string ndt_mcl_name="mcl_ndt";
using namespace std;
namespace GraphMapLocalisation{
LocalisationParamPtr LocalisationFactory::CreateLocalisationParam(string localisationType){

    if(localisationType.compare(ndt_mcl_name)==0){
      cout<<"LocalisationFactory: Created parameters for localisation type: \""<<ndt_mcl_name<<"\""<<endl;
      return MCLNDTParamPtr (new MCLNDTParam());
    }  else{
      std::cerr<<"No localisation parameter type exists with name: \""<<localisationType<<"\""<<endl;
      return NULL;
    }
}

LocalisationTypePtr  LocalisationFactory::CreateLocalisationType(LocalisationParamPtr param){
  if(MCLNDTParamPtr mcl_ndt_par=boost::dynamic_pointer_cast<MCLNDTParam>(param)){
    cout<<"LocalisationFactory: Created object of type: \""<<ndt_mcl_name<<"\""<<endl;
     return  MCLNDTTypePtr(new MCLNDTType(mcl_ndt_par));
  }
  else{
    std::cerr<<"LocalisationFactory: No localisation type exists for parameters"<<endl;
    return NULL;
  }
}
}
