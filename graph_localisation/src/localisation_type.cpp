#include "graph_localisation/localisation_type.h"
namespace perception_oru{
namespace  GraphMapLocalisation {
LocalisationType::~LocalisationType(){}
std::string LocalisationType::ToString(){
  std::stringstream ss;
  ss<<"Localisation Type:"<<endl;
  ss<<"Initialized: "<<std::boolalpha<<initialized_<<endl;
  return ss.str();
}

LocalisationParam::~LocalisationParam(){}
LocalisationParam::LocalisationParam(){


}

}
}

