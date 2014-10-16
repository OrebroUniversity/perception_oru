#include <ndt_registration/ndt_matcher_p2d.h>
#include <ndt_registration/ndt_matcher_d2d.h>
#include <ndt_map/ndt_histogram.h>

namespace lslgeneric
{

  class MapVertex{
  public:
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> pose;
    pcl::PointCloud<pcl::PointXYZ> scan;
    int id;
    NDTHistogram hist;
    double timeRegistration;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };

  class MapEdge{
  public:
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> relative_pose;
    Eigen::Matrix<double,6,6> covariance;
    int idFirst, idSecond;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };


  class NDTMapBuilder{
  public:
  NDTMapBuilder(double res, bool _doHistogram = false):tr(res){
      isP2F = false;
      isF2F=false;
      doHistogram = _doHistogram;
    }
    /* bool setICP(){ */
    /*       isP2F = false; */
    /*       isF2F=false; */
    /*       return true; */
    /*   } */
    bool setMatcherP2F(NDTMatcherP2D *_matcherP2F){
      matcherP2F  = _matcherP2F;
      isP2F = true;
      isF2F=false;
      return true;
    }

    bool setMatcherF2F(NDTMatcherD2D *_matcherF2F){
      matcherF2F  = _matcherF2F;
      isP2F = false;
      isF2F=true;
      return true;
    }

    bool addScan(pcl::PointCloud<pcl::PointXYZ> scan, int id=-1);
    void saveG2OlogFile(const char* fname);
    void saveDatlogFile(const char* fname);
    void printNodePositions();

    lslgeneric::LazyGrid tr;

  private:
    NDTMatcherP2D *matcherP2F;
    NDTMatcherD2D *matcherF2F;
    bool isP2F, isF2F, doHistogram;
    double resolution;
    std::vector<MapVertex> vertices;
    std::vector<MapEdge> edges;


    /* bool matchICP(pcl::PointCloud<pcl::PointXYZ> &target,  pcl::PointCloud<pcl::PointXYZ> &source, */
    /*               Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &Tout, double &finalscore); */
  };
}


