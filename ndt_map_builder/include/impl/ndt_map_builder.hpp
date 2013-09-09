#include<pointcloud_utils.h>
#include<Eigen/Eigen>

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include "pcl/registration/icp.h"
#include "pcl/filters/voxel_grid.h"

namespace lslgeneric
{

//ICP wrapper
template <typename PointT>
bool NDTMapBuilder<PointT>::matchICP(pcl::PointCloud<PointT> &target,  pcl::PointCloud<PointT> &source,
                                     Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &Tout, double &finalscore)
{

    typename pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);

    typename pcl::PointCloud<PointT>::ConstPtr m (new pcl::PointCloud<PointT>(source) );
    typename pcl::PointCloud<PointT>::ConstPtr f (new pcl::PointCloud<PointT>(target) );

    typename pcl::VoxelGrid<PointT> gr1;
    typename pcl::VoxelGrid<PointT> gr2;
    gr1.setLeafSize(0.1,0.1,0.1);
    gr2.setLeafSize(0.1,0.1,0.1);

    gr1.setInputCloud(m);
    gr2.setInputCloud(f);

    cloud_in->height = 1;
    cloud_in->width = cloud_in->points.size();
    cloud_out->height = 1;
    cloud_out->width = cloud_out->points.size();
    cloud_in->is_dense = false;
    cloud_out->is_dense = false;

    gr1.filter(*cloud_in);
    gr2.filter(*cloud_out);

    pcl::IterativeClosestPoint<PointT, PointT> icp;

    icp.setMaximumIterations(1000);
    std::cout<<"max itr are "<<icp.getMaximumIterations()<<std::endl;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);

    icp.setRANSACOutlierRejectionThreshold (2);
    icp.setMaxCorrespondenceDistance(10);
    icp.setTransformationEpsilon(0.00001);
    pcl::PointCloud<PointT> Final;
    icp.align(Final);

    //finalscore is the sum of square error. we want to minimize it...
    finalscore = icp.getFitnessScore();
    Tout = (icp.getFinalTransformation()).template cast<double>();
    return icp.hasConverged();
}


template <typename PointT>
bool NDTMapBuilder<PointT>::addScan(pcl::PointCloud<PointT> scan, int id)
{
    //if first scan, add to vertices.
    if(id == -1)   //automatic IDs
    {
        id = vertices.size();
    }

    if(id == 0)
    {
        MapVertex<PointT> mv;
        mv.scan = scan;
        mv.pose.setIdentity();
        mv.id = id;

        vertices.push_back(mv);
        return true;
    }

    //find last added scan
    if(id-1 <0 || id-1 > vertices.size())
        return false;

    std::cout<<"Matching scans with ids "<<vertices.back().id<<" and "<<id<<std::endl;
    //try to match them, obtain relative pose
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T, Tout, Tndt;
    Eigen::Matrix<double,6,6> cov;
    bool succ = false;
    struct timeval tv_start, tv_end;
    double bestscore = INT_MAX;
    double finalscore;
    double time_match = 0;
    Tout.setIdentity();


    lslgeneric::NDTMap<PointT> fixed(&tr);
    fixed.loadPointCloud(vertices.back().scan);
    lslgeneric::NDTMap<PointT> moving(&tr);
    moving.loadPointCloud(scan);

    moving.computeNDTCells();
    fixed.computeNDTCells();

    lslgeneric::NDTHistogram<PointT> fixedH(fixed);
    lslgeneric::NDTHistogram<PointT> movingH(moving);

    movingH.bestFitToHistogram(fixedH,T);
    pcl::PointCloud<PointT> cloud3;

    //check in between the top 3 histogram positions and the no initial guess solutions
    //such a good place for some threads! TODO
    for(int q=0; q<4; q++)
    {
        if(!doHistogram && q != 3) continue;
        if(q!=3)
        {
            movingH.getTransform(q,T);
        }
        else
        {
            T.setIdentity();
        }
//	cout<<"T init "<<T.translation().transpose()<<" r "<<T.rotation().eulerAngles(0,1,2).transpose()<<endl;
        Tndt.setIdentity();
        cloud3 = lslgeneric::transformPointCloud(T,scan);

        bool ret;
        gettimeofday(&tv_start,NULL);
        if(isF2F)
        {
            ret = matcherF2F->match(vertices.back().scan,cloud3,Tndt);
            finalscore = matcherF2F->finalscore;
        }
        else if(isP2F)
        {
            ret = matcherP2F->match(vertices.back().scan,cloud3,Tndt);
            finalscore = matcherP2F->finalscore;
        }
        else
        {
            //icp
            ret = this->matchICP(vertices.back().scan,cloud3,Tndt,finalscore);
        }
        gettimeofday(&tv_end,NULL);

        if(finalscore < bestscore)
        {
            Tout = Tndt*T;
            bestscore = finalscore;
            if(isF2F)
            {
                //matcherF2F->covariance(vertices.back().scan,cloud3,Tndt,cov);
            }
            else if(isP2F)
            {
                //matcherP2F->covariance(vertices.back().scan,cloud3,Tndt,cov);
            }
            else
            {
                cov.setIdentity();
                cov = 0.1*cov;
            }
            std::cout<<"score = "<<bestscore<<"best is "<<q<<std::endl;
        }
        std::cout<<"T fin "<<Tout.translation().transpose()<<" r "<<Tout.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
        time_match += (tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.;
    }

    std::cout<<" TIME: (MATCHING ONLY) "<< time_match << std::endl;


    //best fit pose is stored in Tout
    //add vertex with global pose calculated based on pose of previous scan.
    MapVertex<PointT> vert;
    vert.scan = scan;
    vert.pose = vertices.back().pose*Tout;
    vert.id = id;
    vert.hist = movingH;
    vert.timeRegistration = time_match;
//	(tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.;

    //add edge with covariance and relative pose
    MapEdge edge;
    edge.idFirst = vertices.back().id;
    edge.idSecond = id;
    edge.relative_pose = Tout;
    edge.covariance = cov;

    //add them
    vertices.push_back(vert);
    edges.push_back(edge);

    //go through all previous scans and check for similarity
#if 0
    for(int i=0; i<vertices.size()-2; i++)
    {
        //lslgeneric::NDTMap f(&tr);
        //f.loadPointCloud(vertices[i].scan);
        //f.computeNDTCells();
        lslgeneric::NDTHistogram<PointT> fH = vertices[i].hist;

        //compare fH and movingH
        double sim = movingH.getSimilarity(fH);

        if(sim < 0.22)
        {
            //try to match
            std::cout<<"Trying to match.. "<<id<<" to "<<vertices[i].id<<" at i "<<i<<" scan similarity "<<sim<<std::endl;
            //check score of match, if good --- add new constraint in the edge graph
            /*bestscore = INT_MAX;
            for(int q=0; q<1; q++) {
            if(q!=3) {
                movingH.getTransform(q,T);
            } else {
                T.setIdentity();
            }
            pcl::PointCloud<PointT> cloud3 = lslgeneric::transformPointCloud(T,scan);
            bool ret = matcherF2F->match(vertices[i].scan,cloud3,Tndt);
            finalscore = matcherF2F->finalscore;
            if(finalscore < bestscore) {
                Tout = Tndt*T;
                bestscore = finalscore;
                matcherF2F->covariance(vertices.back().scan,cloud3,Tndt,cov);
                cout<<"score = "<<bestscore<<"best is "<<q<<endl;
            }
            }*/

            movingH.getTransform(0,T);
            pcl::PointCloud<PointT> cloud3 = lslgeneric::transformPointCloud(T,scan);

            bool ret;
            if(isF2F)
            {
                ret = matcherF2F->match(vertices[i].scan,cloud3,Tndt);
                finalscore = matcherF2F->finalscore;
            }
            else if(isP2F)
            {
                finalscore = 0;
            }
            else
            {
                //icp
                finalscore = 0;
            }

            if(finalscore < -0.8)
            {
                std::cout<<"NEW EDGE!\n";
                //add edge with covariance and relative pose
                edge.idFirst = vertices[i].id;
                edge.idSecond = id;
                edge.relative_pose = Tout;
                edge.covariance = cov;
                edges.insert(edges.begin(),edge);
            }
        }
    }
#endif

}

template <typename PointT>
void NDTMapBuilder<PointT>::saveG2OlogFile(const char* fname)
{
    //open file
    FILE *fout = fopen(fname,"w");
    Eigen::Matrix<double,6,6> eye;
    eye.setIdentity();
    double reg = 1e-4;

    //go through verices and dump them -- these are the "initial guess"
    for(int i=0; i<vertices.size(); i++)
    {
        Eigen::Vector3d t = vertices[i].pose.translation();
        Eigen::Quaternion<double> q;
        q = vertices[i].pose.rotation();
        fprintf(fout,"VERTEX_SE3:QUAT %d %lf %lf %lf %lf %lf %lf %lf\n",vertices[i].id,
                t(0),t(1),t(2),q.x(),q.y(),q.z(),q.w());
    }

    //go through edges and dump them
    for(int i=0; i<edges.size(); i++)
    {
        Eigen::Vector3d t = edges[i].relative_pose.translation();
        Eigen::Quaterniond q;
        q = edges[i].relative_pose.rotation();
        fprintf(fout,"EDGE_SE3:QUAT %d %d %lf %lf %lf %lf %lf %lf %lf ",edges[i].idFirst,edges[i].idSecond,
                t(0),t(1),t(2),q.x(),q.y(),q.z(),q.w());

        Eigen::Matrix<double,6,6> information = (edges[i].covariance+reg*eye).inverse();
        for(int p=0; p<6; p++)
        {
            for(int q=p; q<6; q++)
            {
                fprintf(fout,"%lf ",information(p,q));
            }
        }
        fprintf(fout,"\n");
    }
    fclose(fout);
}

template <typename PointT>
void NDTMapBuilder<PointT>::saveDatlogFile(const char* fname)
{
    //open file
    FILE *fout = fopen(fname,"w");

    //go through verices and dump them -- these are the "initial guess"
    for(int i=0; i<vertices.size(); i++)
    {
        Eigen::Vector3d t = vertices[i].pose.translation();
        Eigen::Quaternion<double> q;
        q = vertices[i].pose.rotation();
        fprintf(fout,"%d %lf %lf %lf %lf %lf %lf %lf\n",vertices[i].id,
                t(0),t(1),t(2),q.x(),q.y(),q.z(),q.w());
    }
    fclose(fout);

    char fn[500];
    snprintf(fn,499,"%s.times",fname);

    fout = fopen(fn,"w");
    fprintf(fout,"Tr = [");
    for( int i=1; i<vertices.size(); i++)
    {
        fprintf(fout,"%lf ",vertices[i].timeRegistration);
    }
    fprintf(fout,"];\n");

    fclose(fout);
}

template <typename PointT>
void NDTMapBuilder<PointT>::printNodePositions()
{
    //go through verices and dump them
    for(int i=0; i<vertices.size(); i++)
    {
        std::cout<<vertices[i].pose.translation().transpose()<<" "<<vertices[i].pose.rotation().eulerAngles(0,1,2)<<std::endl;
    }
}

template <typename PointT>
void NDTMapBuilder<PointT>::theMotherOfAllPointClouds(const char* fname)
{
    //go through and dump stuff
    FILE *fout = fopen(fname,"w");
    fprintf(fout,"#VRML V2.0 utf8\n");
    for( int i=0; i<vertices.size(); i++)
    {
        pcl::PointCloud<PointT> cloud3 = lslgeneric::transformPointCloud(vertices[i].pose,vertices[i].scan);
        pcl::PointCloud<PointT> cloud3SS;
        for(int q=0; q<cloud3.points.size(); q++)
        {
            if(q%10 == 0) cloud3SS.points.push_back(cloud3.points[q]);
        }

        //ODD = green, even = white
        if(i%2)
        {
            //green = target
            lslgeneric::writeToVRML(fout,cloud3SS,Eigen::Vector3d(0,1,0));
        }
        else
        {
            //white = final
            lslgeneric::writeToVRML(fout,cloud3SS,Eigen::Vector3d(1,1,1));
        }
    }
    fclose(fout);

}
}
