#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/line.h>
#include "ndt_line_visual.hpp"

namespace lslgeneric{
  NDTLineVisual::NDTLineVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ){
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
    NDT_l1_.reset(new rviz::Line(scene_manager_,frame_node_ ));
    NDT_l2_.reset(new rviz::Line(scene_manager_,frame_node_ ));
    NDT_l3_.reset(new rviz::Line(scene_manager_,frame_node_ ));
  }

  NDTLineVisual::~NDTLineVisual()
  {
    scene_manager_->destroySceneNode( frame_node_ );
  }

  void NDTLineVisual::setCell(ndt_map::NDTCellMsg cell, double resolution){
    Ogre::Vector3 position(cell.mean_x,cell.mean_y,cell.mean_z);
    //eigen values aka size of the elipsoid
    Eigen::Matrix3d cov;
    int m_itr=0;
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        cov(i,j)=cell.cov_matrix[m_itr];
        m_itr++;
      }
    }
    Eigen::Matrix3d m_eigVec = Eigen::Matrix3d::Zero(3,3);
    Eigen::Matrix3d m_eigVal = Eigen::Matrix3d::Zero(3,3);
    Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
    m_eigVal = es.pseudoEigenvalueMatrix();
    m_eigVec = es.pseudoEigenvectors();
    m_eigVal = m_eigVal.cwiseSqrt();

    Eigen::Quaternion<double> q(m_eigVec);
    Ogre::Vector3 scale(m_eigVal(0,0),m_eigVal(1,1),m_eigVal(2,2));
    Ogre::Quaternion orient(q.w(),q.x(),q.y(),q.z());

    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> Sol (cov);
    // Eigen::Matrix3d evecs;
    // Eigen::Vector3d evals;
    // evecs = Sol.eigenvectors().real();
    // evals = Sol.eigenvalues().real();
    // Eigen::Quaternion<double> q(evecs);
    // Ogre::Vector3 scale(30*evals(0),30*evals(1),30*evals(2));
    // Ogre::Quaternion orient(q.w(),q.x(),q.y(),q.z());

    NDT_l1_->setPosition(position);
    NDT_l1_->setOrientation(orient);
    NDT_l1_->setPoints(Ogre::Vector3(-scale[0],0,0), Ogre::Vector3(scale[0],0, 0));

    NDT_l2_->setPosition(position);
    NDT_l2_->setOrientation(orient);
    NDT_l2_->setPoints(Ogre::Vector3(-scale[1],0,0), Ogre::Vector3(scale[1],0,0));

    NDT_l3_->setPosition(position);
    NDT_l3_->setOrientation(orient);
    NDT_l3_->setPoints(Ogre::Vector3(-scale[2],0,0), Ogre::Vector3(scale[2],0,0));

    // NDT_elipsoid_->setScale(scale);
    // NDT_elipsoid_->setPosition(position);
    // NDT_elipsoid_->setOrientation(orient);
   
  }

  void NDTLineVisual::setFramePosition( const Ogre::Vector3& position ){
    frame_node_->setPosition( position );
  }

  void NDTLineVisual::setFrameOrientation( const Ogre::Quaternion& orientation ){
    frame_node_->setOrientation( orientation );
  }

  void NDTLineVisual::setColor( float r, float g, float b, float a ){
    NDT_l1_->setColor( r, g, b, a );
    NDT_l2_->setColor( r, g, b, a );
    NDT_l3_->setColor( r, g, b, a );
  }
} 

