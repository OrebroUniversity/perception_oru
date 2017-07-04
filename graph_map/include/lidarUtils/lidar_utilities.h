#ifndef LIDAR_UTILITIES_H
#define LIDAR_UTILITIES_H

#include <laser_geometry/laser_geometry.h>
//#include "gnuplot-iostream.h"
#include "pcl/io/pcd_io.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "strings.h"

using namespace std;
namespace libgraphMap {

typedef std::vector<std::pair<double, double> > pair_vector_double;

class ScanPlot{
public:
  typedef enum scanRepresentation{
                        RangeAngle=0,
                        XY     =1,
                       RangeAngleDerivative=3,
                       XYDerivative=4}scanRepresentation;
  void expSmoothO1(pair_vector_double &data,double alpha);
  void scanToPairVector(const sensor_msgs::LaserScan &scan,const scanRepresentation rep,pair_vector_double &data);
  void plot(const sensor_msgs::LaserScan &scan,const scanRepresentation plot_type,double alpha);
  void plot(const sensor_msgs::LaserScan &scan,const scanRepresentation plot_type);
private:
  void plot(const sensor_msgs::LaserScan &scan, const scanRepresentation plot_type, const string PlotOptsX, const string PlotOptsY, double alpha);
  void derivative(pair_vector_double &data_in, pair_vector_double &derivative);
 // Gnuplot gp;


};
}
#endif // LIDAR_UTILITIES_H
