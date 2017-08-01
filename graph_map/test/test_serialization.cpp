#include "stdio.h"
#include "iostream"
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"
#include "Eigen/Geometry"
#include "ndt_map/lazy_grid.h"
#include "ndt_map/ndt_map.h"
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include "ndt_generic/serialization.h"
#include "boost/serialization/set.hpp"
#include "boost/serialization/vector.hpp"
#include <stdlib.h>
#include <time.h>
#define BOOST_SERIALIZATION_DYN_LINK 1
using namespace std;
using namespace Eigen;
using namespace lslgeneric;
string filename="ndtmap_Serialization.dat";


class testNDTMap{
public:
  testNDTMap(){
    NDTMap *map;
    cout<<"TEST NDT MAP "<<endl;
    CreateNdtMap(&map);
    // cout<<"CREATED:\n="<<(map)->ToString()<<endl;
    saveNdtMap(map);
    cout<<"SAVED TO FILE:="<<filename<<endl;
    NDTMap *map2;
    LoadNdtMap(&map2);
    cout<<"cells shared="<<map2->getAllCells().size()<<","<<map2->getAllCellsShared().size()<<","<<map2->getAllInitializedCells().size()<<endl;
    //    cout<<"Loaded ndt map from file with parameters\n="<<(map2)->ToString()<<endl;

  }
private:
  void CreateNdtMap(NDTMap **map){
    srand (time(NULL));
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for(int i=0;i<=30;i++){
      double x=(rand()%1000)/400.0-1.0;
      double y=(rand()%1000)/400.0-1.0;
      double z=(rand()%1000)/500.0;
      cout<<"x="<<x<<"y="<<y<<"z="<<z<<endl;
      pcl::PointXYZ p1(x,y,z);
      cloud.push_back(p1);
    }



    (*map)= new lslgeneric::NDTMap((new lslgeneric::LazyGrid(1.0)));
    (*map)->initialize(0,0,0,100,100,10);
    cout<<"init lazy map"<<endl;
    cout<<"add pointcloud with a size of ="<<cloud.size()<<endl;
    (*map)->addPointCloud(Vector3d(0,0,0),cloud, 0.1, 100.0, 0.1);

    //map1->computeNDTCellsSimple();
    Eigen::Vector3d localMapSize(40,40,10);
    (*map)->addPointCloudMeanUpdate(Eigen::Vector3d(0,0,0),cloud,localMapSize, 1e5, 25, 2*3, 0.06);
    // (*map)->computeNDTCells();
    cout<<"Number of cells generated="<<(*map)->getAllCells().size()<<endl;
    cout<<(*map)->ToString()<<endl;
    std::vector<lslgeneric::NDTCell*> cells=(*map)->getAllCells();

   //   cout<< (*map)->ToString()<<endl;;
  }
  void LoadNdtMap(NDTMap ** map){
    std::ifstream ifs(filename);
    boost::archive::text_iarchive ia(ifs);
    ia >> *map;
    cout<<"loaded ndtmap:"<< (*map)->ToString()<<endl;;

  }

  void saveNdtMap(NDTMap * map){
    std::ofstream ofs( filename );
    boost::archive::text_oarchive ar(ofs);
    // Save the data
    ar << (map);
    ofs.close();

  }
};
void TestNdtCell(){

  NDTCell *cell1=new NDTCell();
  cell1->setMean(Eigen::Vector3d(1,2,3));
  Matrix3d m;
  m<<1,2,3,4,5,6,7,8,9;
  cell1->setCov(m);
  //std::set<c*> update_set,set_to_load;
  //update_set.insert(cell1);
  cout<<"cell1 created with mean=\n"<<cell1->getMean()<<"\nAnd cov=\n"<<cell1->getCov()<<endl;

  std::ofstream ofs( filename );
  boost::archive::text_oarchive ar(ofs);

  ar << cell1;// Save the data
  ofs.close();

  NDTCell *cell3;

  cout<<"load ndt map"<<endl;
  std::ifstream ifs(filename);
  boost::archive::text_iarchive ia(ifs);
  ia >>cell3;
  std::cout<<"Loaded new mean="<<cell3->getMean()<<"\nAnd cov=\n"<<cell3->getCov()<<std::endl;
  //ia >> set_to_load;
  //for(std::set<NDTCell*>::iterator it=set_to_load.begin(); it!=set_to_load.end(); ++it)
  //  cout<<"loaded cov=="<< (*it)->getCov()<<endl;
  ifs.close();
}
void TestCellVector(){
  const int sizex=10,sizey=20,sizez=30;
  CellVector3d vek(sizex,sizey,sizez,NULL);
  NDTCell* cell=new NDTCell();
  cell->setMean(Eigen::Vector3d(1,1,1));
  cout<<"created cell with mean=\n"<<cell->getMean()<<endl;
  vek.SetVal(1,2,3, cell);
  if(vek.GetVal(1,2,3)!=NULL)
    cout<<"and in the vector we find that the same cel exists=\n"<<vek.GetVal(1,2,3)->getMean()<<endl;
  else{
    cerr<<"couldnt find the cell in the vector"<<endl;
    exit(0);
  }

  cout<<"lets modify hte original to:";
  cell->setMean(Eigen::Vector3d(5,5,5));
  cout<<cell->getMean()<<endl;
  if(vek.GetVal(1,2,3)!=NULL)
    cout<<"and in the vector we find that the same cel is also modified to=\n"<<vek.GetVal(1,2,3)->getMean()<<endl;
  else{
    cerr<<"couldnt find the cell in the vector"<<endl;
    exit(0);
  }

  cout<<"now time to test for proper storage"<<endl;
  int i=0,j=0,k=0;

  CellVector3d *vek2=new CellVector3d(sizex,sizey,sizez);

  std::ofstream ofs( filename );
  boost::archive::text_oarchive ar(ofs);
  ar << vek2;// Save the data

  ofs.close();
  NDTCell *temp;
  for(i=0;i<sizex;i++){
    for(j=0;j<sizey;j++){
      for(k=0;k<sizez;k++){
        NDTCell *temp;
        cout<<"create ndtcell"<<endl;
        temp=new NDTCell();
        temp->setMean(Eigen::Vector3d(i,j,k));
        vek2->SetVal(i,j,k,temp);
        cout<<"written"<<temp->getMean()<<endl;
      }
    }
  }
  for(i=0;i<sizex;i++){
    for(j=0;j<sizey;j++){
      for(k=0;k<sizez;k++){

        NDTCell* temp=vek2->GetVal(i,j,k);
        Eigen::Vector3d v3=temp->getMean();
        cout<<"v3=\n"<<v3<<"vs"<<i<<"\n"<<j<<"\n"<<k<<endl;
      }
    }
  }

  cout<<"passed test"<<endl;
}
void SaveEmptyCellVector(){
  CellVector3d *vek2;//=new CellVector3d(sizex,sizey,sizez);

  std::ofstream ofs( filename );
  boost::archive::text_oarchive ar(ofs);

  ar << vek2;// Save the data

  ofs.close();

  std::ifstream ifs(filename);
  boost::archive::text_iarchive ia(ifs);
  ia >> vek2;;

}

void CreateAndSaveLazyGrid(){
  NDTCell *cell1=new NDTCell();
  cell1->setMean(Eigen::Vector3d(1,2,3));

  LazyGrid *lz=new LazyGrid(101,101,10,0.4,0.4,0.4,15,15,15,cell1);
  double cx,cy,cz;
  lz->getCenter(cx,cy,cz);
  cout<<"center="<<cx<<","<<cy<<",cz"<<cz<<endl;


  std::ofstream ofs( filename );
  boost::archive::text_oarchive ar(ofs);

  ar << lz;// Save the data
  ofs.close();


}
void LoadLazyGrid(){
  double xc2,yc2,zc2;
  LazyGrid *lz2;
  cout<<"load lazy grid from file"<<endl;
  std::ifstream ifs(filename);
  boost::archive::text_iarchive ia(ifs);
  ia >> lz2;;
  lz2->getCenter(xc2,yc2,zc2);
  cout<<"loaded cov=="<< xc2<<","<<yc2<<","<<zc2<<endl;
  ifs.close();
}
/*

  pcl::PointXYZ center_;
  double xsize_, ysize_, zsize_;
  Eigen::Matrix3d cov_;		/// Contains the covariance of the normal distribution
  Eigen::Matrix3d icov_;  /// Precomputed inverse covariance (updated every time the cell is updated)
  Eigen::Matrix3d evecs_; /// Eigen vectors
  Eigen::Vector3d mean_;  /// Mean of the normal distribution
  Eigen::Vector3d evals_; /// Eigen values
  CellClass cl_;
  static bool parametersSet_;													// ???
  static double EVAL_ROUGH_THR;		// = 0.1;								// ???
  static double EVEC_INCLINED_THR; 	// = cos(8*M_PI/18);//10 degree slope;	// ???
  static double EVAL_FACTOR;													// ???
  double d1_,d2_;
  unsigned int N; 	///Number of points used for Normal distribution estimation so far
  int emptyval;			///The number of times a cell was observed empty (using ray casting)
  double emptylik;
  double emptydist;
  float R,G,B; 			///RGB values [0..1] - Special implementations for PointXYZRGB & PointXYZI
  float occ;   			///Occupancy value stored as "Log odds" (if you wish)
  float max_occu_;
  TEventData edata;



 */
int main(int argc, char **argv){
  ros::init(argc, argv, "testGraphLib");
  ros::NodeHandle n;
  cout<<"Serialization test software, wanna continue ? (y/n)"<<endl;

  char c=getchar();

  if(c=='n'||c=='N')
    exit(0);
  else
    testNDTMap test;


  //TestNdtCell();
  //testNDTMap testndtmap;

  //CreateAndSaveLazyGrid();
  // LoadLazyGrid();
  //TestNdtCell();




}


