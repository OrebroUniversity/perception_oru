#ifndef GRAPHFACTORY_H
#define GRAPHFACTORY_H
#include <stdio.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include "Eigen/Dense"
#include "boost/shared_ptr.hpp"
using namespace  std;
namespace perception_oru{
namespace libgraphMap{

using Eigen::Vector3d;
using Eigen::Affine3d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
const Matrix6d unit_covar = (Eigen::Matrix<double, 6, 6>() << 0.1,0.0,0.0,0.0,0.0,0.0,
                                                              0.0,0.1,0.0,0.0,0.0,0.0,
                                                              0.0,0.0,0.1,0.0,0.0,0.0,
                                                              0.0,0.0,0.0,0.01,0.0,0.0,
                                                              0.0,0.0,0.0,0.0,0.01,0.0,
                                                              0.0,0.0,0.0,0.0,0.0,0.01).finished();

/*--------------------------- TEMPLATE FOR USAGE OF THE GRAPH LIBRARY ---------------------------------*/
/*!
 * The following types with name template_map_type are presented here as an example for how to create your own map and registration types
 */

/*!
 * \brief templateMapType implements the map or acts as a wrapper to your existing map classes
 * \brief templateMapTypePtr is the general way the map type is passed around, based om shared pointers to ensure memory no memory losses
 */
class TemplateMapType;
typedef boost::shared_ptr<TemplateMapType> TemplateMapTypePtr;

/*!
 * \brief templateMapParam implements the parameters for the map type
 * \brief templateMapParamPtr
 */
class TemplateMapParam;
typedef boost::shared_ptr<TemplateMapParam> TemplateMapParamPtr;

/*!
 * \brief templateRegType implements the registration or act as a wrapper to your existing registration types.
 * \brief templateRegTypePtr
 */
class TemplateRegType;
typedef boost::shared_ptr<TemplateRegType> TemplateRegTypePtr;

/*!
 * \brief templateRegTypeParam implements the parameters for the registration type <templateRegType>
 * \brief regParamPtr
 */
class TemplateRegTypeParam;
typedef boost::shared_ptr<TemplateRegTypeParam> TemplateRegTypeParamPtr;

/*--------------------------------END OF TEMPLATE -------------------------------------------------------*/





class NDTMapType;
typedef boost::shared_ptr<NDTMapType> NDTMapPtr;

class NDTMapParam;
typedef boost::shared_ptr<NDTMapParam> NDTMapParamPtr;

class NDTD2DRegParam;
typedef boost::shared_ptr<NDTD2DRegParam> NDTD2DRegParamPtr;

class NDTD2DRegType;
typedef boost::shared_ptr<NDTD2DRegType> NDTD2DRegTypePtr;


class NDTDLMapType;
typedef boost::shared_ptr<NDTDLMapType> NDTDLMapPtr;

class NDTDLMapParam;
typedef boost::shared_ptr<NDTDLMapParam> NDTDLMapParamPtr;

class NDTDLRegTypeParam;
typedef boost::shared_ptr<NDTDLRegTypeParam> NDTDLRegTypeParamPtr;


class NDTDLRegType;
typedef boost::shared_ptr<NDTDLRegType> NDTDLRegTypePtr;



class factor;
typedef boost::shared_ptr<factor> FactorPtr;

/*!
 *\brief registrationType is an abstract class for registration
 *\brief registrationParameters provides paramerters to the registration
 */
class registrationType;
typedef boost::shared_ptr<registrationType> RegTypePtr;

class registrationParameters;
typedef boost::shared_ptr<registrationParameters> RegParamPtr;

class MapType;
typedef boost::shared_ptr<MapType> MapTypePtr;

class MapParam;
typedef boost::shared_ptr<MapParam> MapParamPtr;


class Node;
typedef boost::shared_ptr<Node> NodePtr;

class MapNode;
typedef boost::shared_ptr<MapNode> MapNodePtr;

class GraphMap;
typedef boost::shared_ptr<GraphMap> GraphMapPtr;

class GraphMapNavigator;
typedef boost::shared_ptr<GraphMapNavigator> GraphMapNavigatorPtr;


class GraphParam;
typedef boost::shared_ptr<GraphParam> GraphParamPtr;



/*!
 * ... Abstract class to implement map parameters.  ...
 */
class GraphFactory{
public:
  static MapParamPtr          CreateMapParam(string MapType);
  static MapTypePtr           CreateMapType(MapParamPtr mapparam);
  static GraphMapNavigatorPtr CreateGraphNavigator(const Eigen::Affine3d &nodepose, MapParamPtr &mapparam, GraphParamPtr graphparam);

  static GraphParamPtr        CreateGraphParam();
  static GraphMapPtr          CreateGraph(const Eigen::Affine3d &nodepose, MapParamPtr &mapparam,GraphParamPtr graphparam);
  static MapNodePtr           CreateMapNode(const Eigen::Affine3d &pose,const MapParamPtr &mapparam);
  static FactorPtr            CreateObservationFactor(MapNodePtr mapPose, NodePtr observationPose,const Eigen::Affine3d &diff,const Matrix6d &covar);
  static FactorPtr            CreateMapNodeFactor(MapNodePtr prevMapPose, MapNodePtr nextMapPose, const Eigen::Affine3d &diff, const Matrix6d &covar);

  static RegTypePtr           CreateRegistrationType(const Eigen::Affine3d &sensor_pose, RegParamPtr regparam);
  static RegParamPtr          CreateRegParam(string regType);



private:
  GraphFactory(){}
};



}

}
#endif // GRAPHFACTORY_H
