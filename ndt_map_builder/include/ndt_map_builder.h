#include <ndt_matcher_p2d.h>
#include <ndt_matcher_d2d.h>
#include <ndt_histogram.h>

namespace lslgeneric
{

template<typename PointT>
class MapVertex
{
public:
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> pose;
    pcl::PointCloud<PointT> scan;
    int id;
    NDTHistogram<PointT> hist;
    double timeRegistration;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class MapEdge
{

public:
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> relative_pose;
    Eigen::Matrix<double,6,6> covariance;
    int idFirst, idSecond;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


template<typename PointT>
class NDTMapBuilder
{

public:
    NDTMapBuilder(bool _doHistogram = false)
    {
        isP2F = false;
        isF2F=false;
        doHistogram = _doHistogram;
    }
    bool setICP()
    {
        isP2F = false;
        isF2F=false;
        return true;
    }
    bool setMatcherP2F(NDTMatcherP2D<PointT,PointT> *_matcherP2F   )
    {
        matcherP2F  = _matcherP2F;
        isP2F = true;
        isF2F=false;
        return true;
    }

    bool setMatcherF2F(NDTMatcherD2D<PointT,PointT> *_matcherF2F)
    {
        matcherF2F  = _matcherF2F;
        isP2F = false;
        isF2F=true;
        return true;
    }

    bool addScan(pcl::PointCloud<PointT> scan, int id=-1);
    void saveG2OlogFile(const char* fname);
    void saveDatlogFile(const char* fname);
    void printNodePositions();
    void theMotherOfAllPointClouds(const char* fname);

    lslgeneric::OctTree<PointT> tr;

private:
    NDTMatcherP2D<PointT,PointT> *matcherP2F;
    NDTMatcherD2D<PointT,PointT> *matcherF2F;
    bool isP2F, isF2F, doHistogram;
    std::vector<MapVertex<PointT>, Eigen::aligned_allocator<MapVertex<PointT> > > vertices;
    std::vector<MapEdge, Eigen::aligned_allocator<MapEdge> > edges;

    bool matchICP(pcl::PointCloud<PointT> &target,  pcl::PointCloud<PointT> &source,
                  Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &Tout, double &finalscore);
};
};

#include <impl/ndt_map_builder.hpp>
