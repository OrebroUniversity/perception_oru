#include <NDTMap.hh>
#include <NDTHistogram.hh>
#include <OctTree.hh>
#include <AdaptiveOctTree.hh>
#include <PointCloudUtils.hh>

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include "pcl/registration/ia_ransac.h"
#include <cstdio>

#include <LazzyGrid.hh>
#include <fstream>

using namespace std;

//CLASSES FOR FPFH MACHING

class FeatureCloud
{
public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::KdTreeFLANN<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
        search_method_xyz_ (new SearchMethod),
        normal_radius_ (0.05),
        feature_radius_ (0.05)
    {
    }

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
        xyz_ = xyz;
        processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
        return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
        return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
        return (features_);
    }

protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
        computeSurfaceNormals ();
        computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
        normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
        norm_est.setInputCloud (xyz_);
        norm_est.setSearchMethod (search_method_xyz_);
        norm_est.setRadiusSearch (normal_radius_);
        norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
        features_ = LocalFeatures::Ptr (new LocalFeatures);

        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud (xyz_);
        fpfh_est.setInputNormals (normals_);
        fpfh_est.setSearchMethod (search_method_xyz_);
        fpfh_est.setRadiusSearch (feature_radius_);
        fpfh_est.compute (*features_);
    }

private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateRegistration
{
public:

    // A struct for storing alignment results

    typedef Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Result;

    TemplateRegistration () :
        min_sample_distance_ (0.05),
        max_correspondence_distance_ (0.01*0.01),
        nr_iterations_ (500)
    {
        // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
        sac_ia_.setMinSampleDistance (min_sample_distance_);
        sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
        sac_ia_.setMaximumIterations (nr_iterations_);
        sac_ia_.setNumberOfSamples (10);
    }

    ~TemplateRegistration () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
        target_ = target_cloud;
        sac_ia_.setInputTarget (target_cloud.getPointCloud ());
        sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Align the moving cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &moving_cloud, TemplateRegistration::Result &result)
    {
        sac_ia_.setInputCloud (moving_cloud.getPointCloud ());
        sac_ia_.setSourceFeatures (moving_cloud.getLocalFeatures ());

        pcl::PointCloud<pcl::PointXYZ> registration_output;
        sac_ia_.align (registration_output);

        result = sac_ia_.getFinalTransformation ().cast<double>();
    }

private:
    // A list of template clouds and the target to which they will be aligned
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    float nr_iterations_;
};


// Align two point clouds based on the features
bool matchFPFH(pcl::PointCloud<pcl::PointXYZ> &fixed,  pcl::PointCloud<pcl::PointXYZ> &moving,
               Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &Tout)
{

    Tout.setIdentity();

    // Load the target cloud PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudM (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudF (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> Final, Final0, Final1;

    *cloudM = moving;
    *cloudF = fixed;

    pcl::VoxelGrid<pcl::PointXYZ> gr1,gr2;
    gr1.setLeafSize(0.05,0.05,0.05);
    gr2.setLeafSize(0.05,0.05,0.05);

    gr1.setInputCloud(cloudM);
    gr2.setInputCloud(cloudF);

    gr1.filter(*cloudM);
    gr2.filter(*cloudF);

    cloudM->height = 1;
    cloudM->width = cloudM->points.size();
    cloudF->height = 1;
    cloudF->width = cloudF->points.size();
    cloudM->is_dense = false;
    cloudF->is_dense = false;

    // Assign to the target FeatureCloud
    FeatureCloud target_cloud, moving_cloud;
    target_cloud.setInputCloud (cloudF);
    moving_cloud.setInputCloud (cloudM);

    TemplateRegistration templateReg;
    templateReg.setTargetCloud (target_cloud);

    // Find the best template alignment
    templateReg.align(moving_cloud,Tout);

    return true;
}

int
main (int argc, char** argv)
{

    if(argc!=4)
    {
        cout<<"usage: "<<argv[0]<<" numberOfUniquePositions pathTotestDirectory outputFileName\n Data has to be in the \
	    proper format!!!\n Assumptions: scans are 15 degrees apart, in groups of 10\n";
        return(-1);
    }
    int nPos = atoi(argv[1]);
    std::string prefix = argv[2];
    lslgeneric::OctTree tr;
    //lslgeneric::LazzyGrid tr(0.5);
    lslgeneric::OctTree::BIG_CELL_SIZE = 0.5;
    lslgeneric::OctTree::SMALL_CELL_SIZE = 0.2;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T_HIST, T_FPFH, Tinverse;
    Eigen::Quaternion<double> id, errorQ;
    id.setIdentity();
    double msec_HIST=0, msec_FPFH=0;
    struct timeval tv_start, tv_end;

    Tinverse = Eigen::AngleAxis<double>(-30*M_PI/180,Eigen::Vector3d::UnitZ());
    double angle_threshold = 5*M_PI/180;
    int N_TRUE_HIST = 0, N_TRUE_FPFH = 0;
    int N_FALSE_HIST= 0, N_FALSE_FPFH= 0;
    int N_PASS1=0, N_PASS2 = 0;

    std::vector<double> angleErrorsHIST, angleErrorsFPFH;
    std::vector<double> timesHIST, timesFPFH;
    for(int i=0; i<nPos; i++)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud[10], cloud2, cloud3;
        for(int j=0; j<10; j++)
        {
            char fname[500];
            snprintf(fname,499,"%s%03d.wrl",prefix.c_str(),10*i+j);
            cloud[j] = lslgeneric::readVRML(fname);
        }

        for(int j=2; j<10; j++)
        {

            //HISTOGRAM
            //start timing
            gettimeofday(&tv_start,NULL);
            //compare clouds of j and j-2
            //lslgeneric::NDTMap nd(new lslgeneric::LazzyGrid(0.5));
            lslgeneric::NDTMap nd1(&tr);
            nd1.loadPointCloud(cloud[j-2]);
            //lslgeneric::NDTMap nd2(new lslgeneric::LazzyGrid(0.5));
            lslgeneric::NDTMap nd2(&tr);
            nd2.loadPointCloud(cloud[j]);

            nd1.computeNDTCells();
            nd2.computeNDTCells();

            lslgeneric::NDTHistogram nh1(nd1);
            lslgeneric::NDTHistogram nh2(nd2);

            //find the distance from nh1 to nh2
            nh2.bestFitToHistogram(nh1,T_HIST);
            //stop timing1
            gettimeofday(&tv_end,NULL);
            msec_HIST += (tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.;

            T_HIST = T_HIST*Tinverse;
            errorQ = T_HIST.rotation();
            double angle = acos(id.dot(errorQ))/2;
            angleErrorsHIST.push_back(angle);
            timesHIST.push_back((tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.);

            std::cout<<"h: "<<T_HIST.rotation().eulerAngles(0,1,2).transpose()<<" --> "<<angle<<endl;
            if(fabsf(angle) < angle_threshold)
            {
                cout<< "PASS\n";
                N_TRUE_HIST++;
            }
            else
            {
                N_FALSE_HIST++;
                //TRY SECOND BEST MATCH
                nh2.getTransform(1,T_HIST);
                T_HIST = T_HIST*Tinverse;
                errorQ = T_HIST.rotation();
                double angle = acos(id.dot(errorQ))/2;

                std::cout<<"h: "<<T_HIST.rotation().eulerAngles(0,1,2).transpose()<<" --> "<<angle<<endl;
                if(fabsf(angle) < angle_threshold)
                {
                    cout<< ">>>>>>>>>>>>>> PASS 2\n";
                    N_PASS1++;
                }
                else
                {
                    nh2.getTransform(2,T_HIST);
                    T_HIST = T_HIST*Tinverse;
                    errorQ = T_HIST.rotation();
                    double angle = acos(id.dot(errorQ))/2;

                    std::cout<<"h: "<<T_HIST.rotation().eulerAngles(0,1,2).transpose()<<" --> "<<angle<<endl;
                    if(fabsf(angle) < angle_threshold)
                    {
                        cout<< ">>>>>>>>>>>>>> PASS 3\n";
                        N_PASS2++;
                    }
                    else
                    {
                        cout<< "FAIL "<<angle*180/M_PI<<endl;
                    }
                }
            }

            //double sim = nh2.getSimilarity(nh1);
            //cout<<"scan similarity "<<sim<<endl;

            //FPFH fixed = nd1 = cloud[j-2], moving = nd2
            //start timing
            gettimeofday(&tv_start,NULL);
            matchFPFH(cloud[j-2],cloud[j],T_FPFH);
            //stop timing1
            gettimeofday(&tv_end,NULL);
            msec_FPFH += (tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.;

            T_FPFH = T_FPFH*Tinverse;
            errorQ = T_FPFH.rotation();
            angle = acos(id.dot(errorQ))/2;
            angleErrorsFPFH.push_back(angle);
            timesFPFH.push_back((tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.);

            std::cout<<"f: "<<T_FPFH.rotation().eulerAngles(0,1,2).transpose()<<" --> "<<angle<<endl;
            if(fabsf(angle) < angle_threshold)
            {
                cout<< "PASS\n";
                N_TRUE_FPFH++;
            }
            else
            {
                cout<< "FAIL "<<angle*180/M_PI<<endl;
                N_FALSE_FPFH++;
            }
            /*
            */
            /*
            	    //OUTPUT:
            	    char fname[500];
            	    snprintf(fname,499,"%03d.wrl",10*i+j);
            	    cloud2 = lslgeneric::transformPointCloud(T_HIST,cloud[j-2]);
            	    //cloud3 = lslgeneric::transformPointCloud(T_FPFH,cloud[j-2]);

            	    FILE *f = fopen(fname,"w");
            	    fprintf(f,"#VRML V2.0 utf8\n");
            	    //green = target
            	    lslgeneric::writeToVRML(f,cloud[j-2],Eigen::Vector3d(0,1,0));
            	    //red = init
            	    lslgeneric::writeToVRML(f,cloud[j],Eigen::Vector3d(1,0,0));
            	    //white = final hist
            	    lslgeneric::writeToVRML(f,cloud2,Eigen::Vector3d(1,1,1));
            	    //blue = final fpfh
            	    //lslgeneric::writeToVRML(f,cloud3,Eigen::Vector3d(0,0,1));
            	    fclose(f);
            */
        }
    }

    cout<<"=================================\n HISTOGRAM: \n SUCC: "
        <<N_TRUE_HIST<<" FAIL: "<<N_FALSE_HIST<<" P1 "<<N_PASS1<<" P2 "<<N_PASS2<<" avg time "<<msec_HIST/(nPos*10)<<" msec\n FPFH: \n SUCC: "
        <<N_TRUE_FPFH<<" FAIL: "<<N_FALSE_FPFH<<" avg time "<<msec_FPFH/(nPos*10)<<" msec\n";

    ofstream logger(argv[3]);

    logger<<"Eh = [";
    for(int i=0; i<angleErrorsHIST.size(); i++)
    {
        logger<<angleErrorsHIST[i]<<" ";
    }
    logger<<"];\n";

    logger<<"Ef = [";
    for(int i=0; i<angleErrorsFPFH.size(); i++)
    {
        logger<<angleErrorsFPFH[i]<<" ";
    }
    logger<<"];\n";

    logger<<"Th = [";
    for(int i=0; i<timesHIST.size(); i++)
    {
        logger<<timesHIST[i]<<" ";
    }
    logger<<"];\n";

    logger<<"Tf = [";
    for(int i=0; i<timesFPFH.size(); i++)
    {
        logger<<timesFPFH[i]<<" ";
    }
    logger<<"];\n";

    logger<<"succh = "<<N_TRUE_HIST<<"; failh = "<<N_FALSE_HIST
          <<"; succf= "<<N_TRUE_FPFH<<"; failf = "<<N_FALSE_FPFH<<";\n"<<" P1 = "<<N_PASS1<<" P2= "<<N_PASS2<<"\n;";

    return (0);
}



