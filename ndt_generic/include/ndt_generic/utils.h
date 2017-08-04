#pragma once

#include <string>
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <sstream>	
#include <sys/time.h>
#include <time.h>
namespace ndt_generic
{

void getVectorMeanStdev(const std::vector<double> &v, double &mean, double &stdev) {

    mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();

    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
		   std::bind2nd(std::minus<double>(), mean));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    stdev = std::sqrt(sq_sum / v.size());
}

void getVectorMinMax(const std::vector<double> &v, double &min, double &max) {

    min = *std::min_element(v.begin(), v.end());
    max = *std::max_element(v.begin(), v.end());
}

void getVectorQuartiles(const std::vector<double> &vec, double &q1, double &median, double &q3) {
    std::vector<double> v(vec);
    std::sort(v.begin(), v.end());
    q1 = v[v.size()*1/4];
    median = v[v.size()*2/4];
    q3 = v[v.size()*3/4];
}

void normalizeVector(std::vector<double> &v) {
    double weight_factor = std::accumulate(v.begin(), v.end(), 0.);
    std::vector<double>::iterator it;
    for(it = v.begin(); it != v.end(); it++)
        *it = *it * weight_factor;
}

template<class T> std::string toString (const T& x)
{
    std::ostringstream o;

    if (!(o << x))
        throw std::runtime_error ("::toString()");

    return o.str ();
}

template <class T> T 
fromString(const std::string &s) {
    T t;
    std::istringstream iss(s);
    iss >> t;
    return t;
}

std::string getVectorStatisticStr(const std::vector<double> &data) {
    double mean, stdev, min, max, q1, median, q3;
    ndt_generic::getVectorMeanStdev(data, mean, stdev);
    ndt_generic::getVectorMinMax(data, min, max);
    ndt_generic::getVectorQuartiles(data, q1, median, q3);

    std::string ret("[mean]: " + toString(mean) + "\n[stdev]:" + toString(stdev) + "\n[min]: " + toString(min) + "\n[max] :" + toString(max) + "\n[q1]: " + toString(q1) + "\n[median]: " + toString(median) + "\n[q3]: " + toString(q3));
    return ret;
}

std::string getVectorStatisticGnuplotStr(const std::vector<double> &data) {
    double mean, stdev, min, max, q1, median, q3;
    ndt_generic::getVectorMeanStdev(data, mean, stdev);
    ndt_generic::getVectorMinMax(data, min, max);
    ndt_generic::getVectorQuartiles(data, q1, median, q3);
    
    std::string ret(toString(mean) + "\t" + toString(stdev) + "\t" + toString(min) + "\t" + toString(max) + "\t" + toString(q1) + "\t" + toString(median) + "\t" + toString(q3));
    return ret;
}

double getDoubleTime()
{
    struct timeval time;
    gettimeofday(&time,NULL);
    return time.tv_sec + time.tv_usec * 1e-6;
}
//!
//! \brief currentDateTimeString return the current time formated as a file-name compatible string
//! \return not used
//!
const std::string currentDateTimeString() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "d%Y_%m_%d_time%H_%M_%S", &tstruct);

  return buf;
}



} // namespace
