#include "stdio.h"
#include "iostream"
#include "graph_map/graph_map.h"
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include "graph_map/graphfactory.h"
#include "ros/ros.h"
#include "Eigen/Geometry"
#include "visualization/graph_plot.h"

using namespace std;
using namespace Eigen;

using namespace perception_oru;
using namespace libgraphMap;

int main(int argc, char **argv){
  ros::init(argc, argv, "testGraphLib");
  ros::NodeHandle n;
  string maptype;

  n.param<std::string>("map_type",maptype,"ndt_2d_map");
  cout<<"Starting graph node with maptype: "<<maptype<<endl;

  Eigen::Affine3d initPose=Affine3d::Identity();
  initPose.translation()<<2.5,2.5,0.01;//Create initiali pose of graph

  Eigen::Affine3d diff=Affine3d::Identity();

  diff= AngleAxisd(0.0*M_PI, Vector3d::UnitX())
      * AngleAxisd(0.0*M_PI, Vector3d::UnitY())
      * AngleAxisd(0.2*M_PI, Vector3d::UnitZ())*Translation3d(1,1,0);//Transformation between subsequent map nodes

  Matrix6d cov;
  Eigen::DiagonalMatrix<double,6> diag1;
  diag1.diagonal()<<0.15,0.15,0.15,0.01,0.01,0.01;
  cov=diag1; //Create covariance to represent uncertainty betweem mpde

  MapParamPtr param=GraphFactory::CreateMapParam(maptype);
  GraphParamPtr graphparam=GraphFactory::CreateGraphParam();
  GraphMapPtr graph=GraphFactory::CreateGraph(initPose,param,graphparam);

  cout<<"size of graph :"<<graph->GraphSize()<<endl;

  graph->AddMapNode(param,diff,cov);
  graph->AddMapNode(param,diff,cov);
  graph->AddMapNode(param,diff,cov);
  cout<<"size of graph :"<<graph->GraphSize()<<endl;
  GraphPlot::PlotPoseGraph(graph);
  cout<<graph->ToString()<<endl;
  pcl::PointCloud<pcl::PointXYZ> cloud;

}



