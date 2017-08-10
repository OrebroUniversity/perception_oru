#include "graph_localisation/localisation_factory.h"
#include "graph_localisation/localisation_type.h"
#include "mcl_ndt/mcl_ndt.h"
string ndt_mcl_name="ndt_mcl";
using namespace std;
namespace GraphMapLocalisation{
LocalisationParamPtr LocalisationFactory::CreateLocalisationParam(string localisationType){

    if(localisationType.compare(ndt_mcl_name)==0){
      cout<<"Localisation: Created parameters for localisation type: \""<<ndt_mcl_name<<"\""<<endl;
      return MCLNDTParamPtr (new MCLNDTParam());
    }  else{
      std::cerr<<"No localisation type exists with name: \""<<localisationType<<"\""<<endl;
      return NULL;
    }
}

LocalisationTypePtr  LocalisationFactory::CreateLocalisationType(LocalisationParamPtr param){
  if(MCLNDTParamPtr mcl_ndt_par=boost::dynamic_pointer_cast<MCLNDTParam>(param)){
    cerr<<"Graphfactory: no map exists for \"template\""<<endl;
     return  MCLNDTTypePtr(new MCLNDTType(mcl_ndt_par));
    return NULL;
  }
  else{
    std::cerr<<"Graphfactory: No map type exists for map parameters"<<endl;
    return NULL;
  }
}
}
