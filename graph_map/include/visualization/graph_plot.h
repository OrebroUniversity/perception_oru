#ifndef GRAPH_PLOT_H
#define GRAPH_PLOT_H
#include "graph_map/graphfactory.h"
#include "ros/ros.h"
#include "ros/publisher.h"
#include "Eigen/Dense"
#include "visualization_msgs/MarkerArray.h"
#include "ndt_map/ndt_map.h"
#include "geometry_msgs/PoseArray.h"
#include "eigen_conversions/eigen_msg.h"
#include "graph_map/graph_map.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ndt_mcl/3d_ndt_mcl.h"
#include "graph_map/ndt_dl/ndtdl_map_type.h"

#define NDT_GLOBAL_MAP_TOPIC "NDTglobalMap"
#define NDT_GLOBAL2_MAP_TOPIC "NDTglobal2Map"
#define NDT_LOCAL_MAP_TOPIC  "NDTlocalMap"
#define GRAPH_POSE_TOPIC "graphMap"
#define GRAPH_INFO_TOPIC "graphInfo"
#define PARTICLES_TOPIC "ParticleCloud"
namespace perception_oru{
namespace libgraphMap{
using namespace std;
using Eigen::Affine3d;
typedef std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d> > cov_vector;
typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > mean_vector;
typedef enum plotmarker{sphere=0,cross=1,point=2}PlotMarker;
class GraphPlot{

public:

  static void sendMapToRviz(mean_vector &mean, cov_vector &cov, ros::Publisher *mapPublisher, string frame, int color, const Affine3d &offset=Affine3d::Identity(), string ns="ndt", int markerType=visualization_msgs::Marker::SPHERE);
  static void SendLocalMapToRviz(perception_oru::NDTMap *mapPtr, int color,const Affine3d &offset=Affine3d::Identity());
  static void SendGlobalMapToRviz(perception_oru::NDTMap *mapPtr, int color,const Affine3d &offset=Affine3d::Identity());
  static void SendGlobal2MapToRviz(perception_oru::NDTMap *mapPtr, int color,const Affine3d &offset=Affine3d::Identity());
  static void SendGlobal2MapToRviz(std::vector<perception_oru::NDTCell*>cells, int color,const Affine3d &offset=Affine3d::Identity());
  static void plotParticleCloud( const Eigen::Affine3d &offset,std::vector<PoseParticle> pcloud);
  static void PlotPoseGraph(GraphMapPtr graph);
  static void PlotMap(MapTypePtr map,int color, const Affine3d &offset=Affine3d::Identity(),PlotMarker marker=sphere, std::string ns="");
  static void PlotMap(NDTMap &map, int color, const Affine3d &offset=Affine3d::Identity(),PlotMarker marker=sphere, std::string ns="ndtmap", double point_size=0.1);
private:
  static void PublishMapAsPoints(mean_vector &mean, int color,double scale,const Eigen::Affine3d &offset, std::string ns = "pts", int id = 0);
  static void CovarToMarker(const Eigen::Matrix3d &cov,const Eigen::Vector3d &mean,visualization_msgs::Marker &marker);
  static void GetAllCellsMeanCov(const perception_oru::NDTMap *mapPtr, cov_vector &cov, mean_vector &mean);
  static void GetAllCellsMeanCov( std::vector<perception_oru::NDTCell*>cells,cov_vector &cov, mean_vector &mean);
  static void makeRightHanded( Eigen::Matrix3d& eigenvectors, Eigen::Vector3d& eigenvalues);
  static void computeShapeScaleAndOrientation3D(const Eigen::Matrix3d& covariance, Eigen::Vector3d& scale, Eigen::Quaterniond& orientation);
  static void Initialize();
  static bool initialized_;
  static ros::NodeHandle *nh_;
  static ros::Publisher *localMapPublisher_;
  static ros::Publisher *globalMapPublisher_,*global2MapPublisher_;
  static ros::Publisher *graphPosePublisher_, *graphInfoPublisher_,*particlaCloudPublisher_;
private:
  GraphPlot();
  class ColorGradient
  {
  private:
    struct ColorPoint  // Internal class used to store colors at different points in the gradient.
    {
      float r,g,b;      // Red, green and blue values of our color.
      float val;        // Position of our color along the gradient (between 0 and 1).
      ColorPoint(float red, float green, float blue, float value)
        : r(red), g(green), b(blue), val(value) {}
    };
    vector<ColorPoint> color;      // An array of color points in ascending value.

  public:
    //-- Default constructor:
    ColorGradient()  {  createDefaultHeatMapGradient();  }

    //-- Inserts a new color point into its correct position:
    void addColorPoint(float red, float green, float blue, float value)
    {
      for(int i=0; i<color.size(); i++)  {
        if(value < color[i].val) {
          color.insert(color.begin()+i, ColorPoint(red,green,blue, value));
          return;  }}
      color.push_back(ColorPoint(red,green,blue, value));
    }

    //-- Inserts a new color point into its correct position:
    void clearGradient() { color.clear(); }

    //-- Places a 5 color heapmap gradient into the "color" vector:
    void createDefaultHeatMapGradient()
    {
      color.clear();
      color.push_back(ColorPoint(0, 0, 1,   0.0f));      // Blue.
      color.push_back(ColorPoint(0, 1, 1,   0.25f));     // Cyan.
      color.push_back(ColorPoint(0, 1, 0,   0.5f));      // Green.
      color.push_back(ColorPoint(1, 1, 0,   0.75f));     // Yellow.
      color.push_back(ColorPoint(1, 0, 0,   1.0f));      // Red.
    }

    //-- Inputs a (value) between 0 and 1 and outputs the (red), (green) and (blue)
    //-- values representing that position in the gradient.
    void getColorAtValue(const float value, float &red, float &green, float &blue)
    {
      if(color.size()==0)
        return;

      for(int i=0; i<color.size(); i++)
      {
        ColorPoint &currC = color[i];
        if(value < currC.val)
        {
          ColorPoint &prevC  = color[ max(0,i-1) ];
          float valueDiff    = (prevC.val - currC.val);
          float fractBetween = (valueDiff==0) ? 0 : (value - currC.val) / valueDiff;
          red   = (prevC.r - currC.r)*fractBetween + currC.r;
          green = (prevC.g - currC.g)*fractBetween + currC.g;
          blue  = (prevC.b - currC.b)*fractBetween + currC.b;
          return;
        }
      }
      red   = color.back().r;
      green = color.back().g;
      blue  = color.back().b;
      return;
    }
  };

};

}
}
#endif // GRAPH_PLOT_H
