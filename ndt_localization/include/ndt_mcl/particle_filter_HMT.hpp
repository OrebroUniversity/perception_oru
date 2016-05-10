#ifndef PARTICLE_FILTER_HMT_HPP
#define PARTICLE_FILTER_HMT_HPP
//C++ librarys
#include <vector>
#include <iostream>
#include <random>
#include <cmath>
#include <time.h>
//perception_oru
#include "ndt_mcl/particle.hpp"
#include "ndt_map/ndt_map_hmt.h"
//
namespace lslgeneric{
  // enum init_type{
  //   uniform_map,
  //   normal_guess,
  //   map_initial
  // };

  struct bucket{
    double c_x;
    double c_y;
    double c_th;
    double lik;
    double pro;
  };

  class particle_filter_HMT{
    NDTMapHMT *ndtMap;
    bool be2D;
    bool forceSIR;
    //    init_type initializationType;
    int particleCount;
    double varLimit;
    int sirCount;
    int sinceSIR;
    std::vector<particle> tmp;

    lslgeneric::NDTMapHMT* ndt_ISSMap;
    double sx,sy;
    void SIRUpdate();
    void Normalize();

    void Predict2D(double x, double y, double th, double sx, double sy, double sth);
    void to2PI(double &a);
    void toPI(double &a);
    void GetRandomPoint(lslgeneric::NDTCell* cell,double &x, double &y, double &th);
  public:
    particle_filter_HMT(std::string mapFile_, int particleCount_/*, init_type initializationType_*/);
    particle_filter_HMT(lslgeneric::NDTMapHMT *ndtMap_, int particleCount_/*, init_type initializationType_*/, bool be2D_=true, bool forceSIR_=true,double varLimit_=0, int sirCount_=0);
    void UpdateAndPredict(Eigen::Affine3d tMotion,lslgeneric::NDTMap* ndtLocalMap_);
    void UpdateAndPredictEff(Eigen::Affine3d tMotion, lslgeneric::NDTMap* ndtLocalMap_, double subsample_level, double z_cut);
    void UpdateAndPredict(Eigen::Affine3d tMotion,lslgeneric::NDTMap ndtLocalMap_);
    void GetPoseMeanAndVariance2D(Eigen::Vector3d &mean, Eigen::Matrix3d &cov);
    Eigen::Vector3d GetMeanPose2D();
    void Reset();
    void EigenSort( Eigen::Vector3d &eigenvalues,Eigen::Matrix3d &eigenvectors );
    void InitializeFilter();
    void InitializeUniformMap();
    void InitializeNormal(double x, double y, double var);
    void InitializeNormal(double x, double y, double th, double var);
    std::vector<particle> particleCloud;
    Eigen::Affine3d getAsAffine(float x, float y, float yaw);
    };
}
#endif
