#include "lidarUtils/lidar_utilities.h"

namespace libgraphMap {

/*

  */
void ScanPlot::plot(const sensor_msgs::LaserScan &scan,const scanRepresentation plot_type){
  if(plot_type==XY||plot_type==XYDerivative){
    plot(scan,plot_type,"[-20:20]","[-20:20]",1.0);
  }
  else if(plot_type==RangeAngle||plot_type==RangeAngleDerivative){
    plot(scan,plot_type,"[-3.14:3.14]","[-20:20]",1.0);
  }

}
void ScanPlot::plot(const sensor_msgs::LaserScan &scan,const scanRepresentation plot_type,double alpha){
  if(plot_type==XY||plot_type==XYDerivative){
    plot(scan,plot_type,"[-20:20]","[-20:20]",alpha);
  }
  else if(plot_type==RangeAngle||plot_type==RangeAngleDerivative){
    plot(scan,plot_type,"[-3.14:3.14]","[-20:20]",alpha);
  }


}
void ScanPlot::expSmoothO1(pair_vector_double &data,double alpha){
  for(int i=1;i<data.size();i++){
    data[i].second=data[i].second*alpha+data[i-1].second*(1-alpha);
  }

}

void ScanPlot::plot(const sensor_msgs::LaserScan &scan,const scanRepresentation plot_type,const string PlotOptsX,const string PlotOptsY,double alpha){
  pair_vector_double data_set,der;
  scanToPairVector(scan,plot_type,data_set);
  expSmoothO1(data_set,alpha);
  if(plot_type==RangeAngleDerivative||plot_type==XYDerivative){
    derivative(data_set,der);
    data_set=der;
  }

  cout<<"derivative size:="<<der.size()<<", data size="<<data_set.size()<<endl;
 // gp <<"unset autoscale"<<endl;
 // gp << "set xrange "<<PlotOptsX<<"\nset yrange "<<PlotOptsY<<"\n"<<endl;;
 // gp << "plot" << gp.file1d(data_set) << "with points title 'range angle'"<<endl;

  /*
    gp<<"set multiplot layout 2,2 columnsfirst margins 0.1,0.9,0.1,0.9 spacing 0.1"<<endl;
    plotScanRangeAngle(scan,PlotOptsX,PlotOptsY);
    plotScanXY(scan,PlotOptsX,PlotOptsY);
    gp<<"unset multiplot"<<endl;
    break;
  */

}

void ScanPlot::scanToPairVector(const sensor_msgs::LaserScan &scan,const scanRepresentation rep,pair_vector_double &data){
  data.clear();
  double angle;
  double range;
  switch(rep){
  case RangeAngle:
  case RangeAngleDerivative:
    for(int i=0;i<scan.ranges.size();i++){
      angle=-(scan.angle_min+(double)i/(double)(scan.ranges.size())*(double)(scan.angle_max-scan.angle_min));
      range=scan.ranges[i];
      data.push_back(std::make_pair(angle,range ));
    }
    break;
  case XY:
  case XYDerivative:
  default:
    for(int i=0;i<scan.ranges.size();i++){
      angle=-(scan.angle_min+(double)i/(double)(scan.ranges.size())*(double)(scan.angle_max-scan.angle_min));
      range=scan.ranges[i];
      data.push_back(std::make_pair(range*cos(angle+M_PI/2),range*sin(angle+M_PI/2)));
    }
  }
}


void ScanPlot::derivative(pair_vector_double &data_in, pair_vector_double &derivative){
  std::pair<double, double> sample;

  for(int i=1;i<data_in.size()-1;i++){
    double dR=((double)data_in[i+1].second-(double)data_in[i-1].second)/((double)data_in[i+1].first-(double)data_in[i-1].first);
    derivative.push_back(std::make_pair(data_in[i].first ,dR));
    cout<<data_in[i].first<<endl;
  }
  //derivative.push_back(std::make_pair(data_in[data_in.size()-1].first, 0));
  sample=std::make_pair(data_in[0].first,0);
  derivative.insert(derivative.begin(),sample);
}

}
