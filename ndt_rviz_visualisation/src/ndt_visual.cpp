#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <rviz/ogre_helpers/shape.h>
#include "ndt_visual.hpp"

namespace lslgeneric{
  NDTVisual::NDTVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ){
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
    NDT_elipsoid_.reset(new rviz::Shape(rviz::Shape::Sphere,scene_manager_,frame_node_ ));
  }

  NDTVisual::~NDTVisual()
  {
    scene_manager_->destroySceneNode( frame_node_ );
  }

  void NDTVisual::setCell(ndt_map::NDTCellMsg cell, double resolution){
    Ogre::Vector3 position(cell.mean_x,cell.mean_y,cell.mean_z);
    //eigen values aka size of the elipsoid
    Eigen::Matrix3d cov;
    int m_itr=0;
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        cov(j,i)=cell.cov_matrix[m_itr];
        m_itr++;
      }
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> Sol (cov);
    Eigen::Matrix3d evecs;
    Eigen::Vector3d evals;
    evecs = Sol.eigenvectors().real();
    evals = Sol.eigenvalues().real();
    Eigen::Quaternion<double> q(evecs);

    Ogre::Vector3 scale(30*evals(0),30*evals(1),30*evals(2));
    Ogre::Quaternion orient(q.w(),q.x(),q.y(),q.z());
    NDT_elipsoid_->setScale(scale);
    NDT_elipsoid_->setPosition(position);
    NDT_elipsoid_->setOrientation(orient);
   
  }

  void NDTVisual::setFramePosition( const Ogre::Vector3& position ){
    frame_node_->setPosition( position );
  }

  void NDTVisual::setFrameOrientation( const Ogre::Quaternion& orientation ){
    frame_node_->setOrientation( orientation );
  }

  void NDTVisual::setColor( float r, float g, float b, float a ){
    NDT_elipsoid_->setColor( r, g, b, a );
  }
} 

