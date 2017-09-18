#include "graph_localisation/localisation_type.h"
namespace  GraphMapLocalisation {
LocalisationType::~LocalisationType(){}
 std::string LocalisationType::ToString(){
  std::stringstream ss;
  ss<<"Localisation Type:"<<endl;
  ss<<"Initialized: "<<std::boolalpha<<initialized_<<endl;
}

LocalisationParam::~LocalisationParam(){}
LocalisationParam::LocalisationParam(){


}

}

