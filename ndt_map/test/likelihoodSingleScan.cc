#include <ndt_map.h>
#include <oc_tree.h>
#include <pointcloud_utils.h>

#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>

#include <pcl/filters/radius_outlier_removal.h>
#include<iostream>

#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

class OneTestResult
{
public:
    Eigen::VectorXi tp, fp, tn, fn;
};

class Tester
{
private:
    pcl::PointCloud<pcl::PointXYZ> filterRange(pcl::PointCloud<pcl::PointXYZ> &rawCloud);
    pcl::PointCloud<pcl::PointXYZ> filterDensity(pcl::PointCloud<pcl::PointXYZ> &rawCloud);
    void computePartition(pcl::PointCloud<pcl::PointXYZ> &rawCloud);
    pcl::PointCloud<pcl::PointXYZ> generateNegatives(pcl::PointCloud<pcl::PointXYZ> &rawCloud);

    void countResults(OneTestResult &res, std::vector<double> &scoresPositives, std::vector<double> &scoresNegatives);

    int nPartitions, nThresholdDiscretisations;
    double fixed_threshold;
    pcl::PointXYZ origin;
    pcl::PointCloud<pcl::PointXYZ> negCloud;
    double min_offset, max_offset, maxScannerRange;

public:
    Tester() : nPartitions(10),fixed_threshold(0.05),min_offset(0.1),max_offset(2),nThresholdDiscretisations(100)
    {
        maxScannerRange = 100;
    }
    Tester(int _nPartitions, double _min_offset, double _max_offset, int _threshold, double _maxScannerRange, int ndiscr = 100) : nPartitions(_nPartitions),
        fixed_threshold(_threshold),min_offset(_min_offset),max_offset(_max_offset),maxScannerRange(_maxScannerRange),nThresholdDiscretisations(ndiscr)
    {
    }

    ~Tester()
    {
    }

    void runTestsNDTTree(pcl::PointCloud<pcl::PointXYZ> &modelCloud, pcl::PointCloud<pcl::PointXYZ> &testCloud,
                         double resolutionMin, double resolutionMax, pcl::PointXYZ _origin, OneTestResult &res);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

pcl::PointCloud<pcl::PointXYZ> Tester::filterRange(pcl::PointCloud<pcl::PointXYZ> &rawCloud)
{
    pcl::PointCloud<pcl::PointXYZ> pc;
    for(unsigned int i=0; i<rawCloud.points.size(); i++)
    {
        pcl::PointXYZ pt = rawCloud.points[i];
        double d = sqrt(pt.x*pt.x+pt.y*pt.y+pt.z*pt.z);
        if(d<this->maxScannerRange)
        {
            pc.points.push_back(pt);
        }
    }
    return pc;
}


pcl::PointCloud<pcl::PointXYZ> Tester::filterDensity(pcl::PointCloud<pcl::PointXYZ> &rawCloud)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> pointsTree;
    pcl::PointCloud<pcl::PointXYZ> pc;
    double radius = 1;
    int n_neigh = 3;
    std::vector<int> id;
    std::vector<float> dist;
    id.reserve(n_neigh);
    dist.reserve(n_neigh);

    //pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
    //filter.setRadiusSearch(1);
    //filter.setMinNeighborsInRadius(3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcptr(new pcl::PointCloud<pcl::PointXYZ>());
    *pcptr = rawCloud;
    //filter.setInputCloud(pcptr);
    pointsTree.setInputCloud(pcptr);

    std::cout<<"filtering on density....\n before: "<<pc.points.size()<<" "<<pcptr->points.size()<<std::endl;
    for (int i=0; i<pcptr->points.size(); ++i) 
    {
	if(!pointsTree.nearestKSearch(pcptr->points[i],n_neigh,id,dist)) continue;
	if(dist.back() > radius) continue;
	pc.push_back(pcptr->points[i]);
    }
    //filter.filter(pc);
    std::cout<<"after: "<<pc.points.size()<<std::endl;

    return pc;
}


pcl::PointCloud<pcl::PointXYZ> Tester::generateNegatives(pcl::PointCloud<pcl::PointXYZ> &rawCloud)
{

    pcl::PointCloud<pcl::PointXYZ> res;
    res.points.clear();
    pcl::PointXYZ neg;
    for(unsigned int i=0; i<rawCloud.points.size(); i++)
    {
        //generate a random false point on the ray
        double rand_offset = min_offset + (max_offset-min_offset)*
                             (double)rand()/(double)RAND_MAX;

        neg.x = (rawCloud.points[i].x-origin.x);
        neg.y = (rawCloud.points[i].y-origin.y);
        neg.z = (rawCloud.points[i].z-origin.z);
        double len = sqrt(neg.x*neg.x + neg.y*neg.y + neg.z*neg.z);
        double factor = (len - rand_offset) / len;
        factor = (factor < 0) ? 0 : factor;

        neg.x = factor*(rawCloud.points[i].x-origin.x) + origin.x;
        neg.y = factor*(rawCloud.points[i].y-origin.y) + origin.y;
        neg.z = factor*(rawCloud.points[i].z-origin.z) + origin.z;

        res.points.push_back(neg);
    }
    return res;
}

void Tester::countResults(OneTestResult &res, std::vector<double> &scoresPositives, std::vector<double> &scoresNegatives)
{

    res.tp = Eigen::VectorXi (nThresholdDiscretisations,1);
    res.fp = Eigen::VectorXi (nThresholdDiscretisations,1);
    res.tn = Eigen::VectorXi (nThresholdDiscretisations,1);
    res.fn = Eigen::VectorXi (nThresholdDiscretisations,1);

    res.tp.setZero();
    res.fp.setZero();
    res.tn.setZero();
    res.fn.setZero();

    Eigen::VectorXi tmpVec (nThresholdDiscretisations,1);
    Eigen::Block<Eigen::VectorXi> *bl;

    //book-keeping: count TP, FP etc. for each threshold value
    for(unsigned int i=0; i<scoresPositives.size(); i++)
    {
        if(scoresPositives[i] <0 || scoresPositives[i]>1)
        {
            std::cout<<"The horror! "<<scoresPositives[i]<<std::endl;
        }
        int idx = floor(scoresPositives[i]*nThresholdDiscretisations);
        //using a threshold below idx => point is a positive
        tmpVec.setZero();
        bl = new Eigen::Block<Eigen::VectorXi>(tmpVec,0,0,idx,1);
        bl->setOnes();
        res.tp += tmpVec;
        delete bl;

        //above idx => point is a negative
        tmpVec.setZero();
        bl = new Eigen::Block<Eigen::VectorXi>(tmpVec,idx,0,nThresholdDiscretisations-idx,1);
        bl->setOnes();
        res.fn += tmpVec;
        delete bl;
    }

    for(unsigned int i=0; i<scoresNegatives.size(); i++)
    {
        if(scoresNegatives[i] <0 || scoresNegatives[i]>1)
        {
            std::cout<<"The horror! "<<scoresNegatives[i]<<std::endl;
        }
        int idx = floor(scoresNegatives[i]*nThresholdDiscretisations);
        //using a threshold below idx => point is a positive
        tmpVec.setZero();
        bl = new Eigen::Block<Eigen::VectorXi>(tmpVec,0,0,idx,1);
        bl->setOnes();
        res.fp += tmpVec;
        delete bl;

        //above idx => point is a negative
        tmpVec.setZero();
        bl = new Eigen::Block<Eigen::VectorXi>(tmpVec,idx,0,nThresholdDiscretisations-idx,1);
        bl->setOnes();
        res.tn += tmpVec;
        delete bl;
    }
}

void Tester::runTestsNDTTree(pcl::PointCloud<pcl::PointXYZ> &rawCloud, pcl::PointCloud<pcl::PointXYZ> &gtCloud,
                             double resolutionMin, double resolutionMax, pcl::PointXYZ _origin, OneTestResult &res)
{

    std::vector<double> scoresPositives, scoresNegatives;

    gtCloud = filterRange(gtCloud);
    gtCloud = filterDensity(gtCloud);
    negCloud = generateNegatives(gtCloud);

    rawCloud = filterRange(rawCloud);
    //we construct one model from rawcloud and test on gt and negatives
    lslgeneric::OctTree<pcl::PointXYZ> prototype;
    prototype.setParameters(resolutionMax,resolutionMin,10);
    /*
    lslgeneric::OctTree<pcl::PointXYZ>::SMALL_CELL_SIZE = resolutionMin;
    lslgeneric::OctTree<pcl::PointXYZ>::BIG_CELL_SIZE = resolutionMax;
    lslgeneric::OctTree<pcl::PointXYZ>::MAX_POINTS = 10;
    */
    lslgeneric::NDTMap<pcl::PointXYZ> map(&prototype);
    map.loadPointCloud(rawCloud);
    map.computeNDTCells();

    for(unsigned int i =0; i<gtCloud.points.size(); i++)
    {
        scoresPositives.push_back(map.getLikelihoodForPoint(gtCloud.points[i]));
    }
    for(unsigned int i =0; i<negCloud.points.size(); i++)
    {
        scoresNegatives.push_back(map.getLikelihoodForPoint(negCloud.points[i]));
    }

    this->countResults(res,scoresPositives,scoresNegatives);

}

static int ctr = 0;
int
main (int argc, char** argv)
{

    if(argc!=3)
    {
        cout<<"usage: ltest point_cloud1 point_cloud2\n";
        return(-1);
    }
    pcl::PointCloud<pcl::PointXYZ> cloud, cloud2;
    pcl::PointCloud<pcl::PointXYZI> outCloud;

    cloud = lslgeneric::readVRML<pcl::PointXYZ>(argv[1]);
    cloud2 = lslgeneric::readVRML<pcl::PointXYZ>(argv[2]);

    OneTestResult res;
    int ndiscr = 100;
    double threshold = 0.05;
    Tester tester(10,0.3,1,0.05,40,ndiscr);
    pcl::PointXYZ scannerOriginModel;
    scannerOriginModel.x = 0;
    scannerOriginModel.y = 0;
    scannerOriginModel.z = 0;
    tester.runTestsNDTTree(cloud,cloud2,0.5,4,scannerOriginModel,res);

    int index = threshold*ndiscr;

    //full ROC curve
    cout<<"tp = ["<<res.tp.transpose()<<"];\n fp = ["<<res.fp.transpose()<<"];\n tn = ["<<res.tn.transpose()<<"];\n fn = ["<<res.fn.transpose()<<"];\n";
    //just the fixed threshold
    cout<<"tp = ["<<res.tp(index)<<"];\n fp = ["<<res.fp(index)<<"];\n tn = ["<<res.tn(index)<<"];\n fn = ["<<res.fn(index)<<"];\n";


    /*
        //loop through points and compute likelihoods LASER
        for(int i=0; i<cloud.points.size(); i++) {
    	pcl::PointXYZ thisPt = cloud.points[i];
    	//double likelihood = nd.getLikelihoodForPointWithInterpolation(thisPt);
    	double likelihood = nd.getLikelihoodForPoint(thisPt);
    	pcl::PointXYZI outPt;
    	outPt.x = thisPt.x;
    	outPt.y = thisPt.y;
    	outPt.z = thisPt.z;
    	outPt.intensity = likelihood;
    	sumLikelihoods += likelihood;
    	maxLikelihood = (likelihood > maxLikelihood) ?
    			    likelihood : maxLikelihood;
    	outCloud.points[i] = outPt;
        }
        cout<<endl;
        cout<<"max likelihood "<<maxLikelihood<<endl;
        cout<<"sum likelihoods "<<sumLikelihoods<<endl;
        cout<<"average likelihood "<<sumLikelihoods/cloud.points.size()<<endl;
        //normalize for display
        //compute standart deviation
        for(int i=0; i<outCloud.points.size(); i++) {
    	outCloud.points[i].intensity /= (maxLikelihood);
        }
        snprintf(fname,49,"/home/tsv/ndt_tmp/likelihood.wrl");
        lslgeneric::writeToVRML(fname,outCloud);
        */
    return (0);
}



