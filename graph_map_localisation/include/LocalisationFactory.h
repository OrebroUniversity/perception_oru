#ifndef LOCALISATIONFACTORY_H
#define LOCALISATIONFACTORY_H
#include <stdio.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include "Eigen/Dense"
#include "boost/shared_ptr.hpp"
using namespace  std;
namespace GraphMapLocalisation{



/*!
 * \brief templateMapType implements the map or acts as a wrapper to your existing map classes
 * \brief templateMapTypePtr is the general way the map type is passed around, based om shared pointers to ensure memory no memory losses
 */
class TemplateLocalisationType;
typedef boost::shared_ptr<TemplateLocalisationType> TemplateLocalisationTypePtr;

/*!
 * \brief templateMapParam implements the parameters for the map type
 * \brief templateMapParamPtr
 */
class TemplateLocalisationParam;
typedef boost::shared_ptr<TemplateLocalisationParam> TemplateLocalisationParamPtr;



class NDTMCLType;
typedef boost::shared_ptr<NDTMCLType> NDTMCLTypePtr;

class NDTMCLParam;
typedef boost::shared_ptr<NDTMCLParam> NDTMCLParamPtr;


class LocalisationType;
typedef boost::shared_ptr<LocalisationType> LocalisationTypePtr;

class LocalisationParam;
typedef boost::shared_ptr<LocalisationParam> LocalisationParamPtr;

}
#endif // LOCALISATIONFACTORY_H
