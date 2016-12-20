#ifndef NDT_LINE_VISUAL_H
#define NDT_LINE_VISUAL_H

#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/NDTCellMsg.h>

namespace Ogre{
  class Vector3;
  class Quaternion;
}

namespace rviz{
  class Line;
}
namespace lslgeneric{

  class NDTLineVisual{
  public:
    NDTLineVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
    virtual ~NDTLineVisual();
    void setCell(ndt_map::NDTCellMsg msg, double resolution);
    void setFramePosition(const Ogre::Vector3& position);
    void setFrameOrientation(const Ogre::Quaternion& orientation);
    void setColor( float r, float g, float b, float a );
  private:
    boost::shared_ptr<rviz::Line> NDT_l1_, NDT_l2_, NDT_l3_;
    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;
  };
}
#endif 
