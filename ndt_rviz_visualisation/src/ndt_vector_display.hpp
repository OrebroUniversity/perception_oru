#ifndef NDT_VECTORMAPS_DISPLAY_H_04042017
#define NDT_VECTORMAPS_DISPLAY_H_04042017

#include <boost/circular_buffer.hpp>

#include <ndt_map/NDTMapMsg.h>
#include <rviz/message_filter_display.h>
#include "ndt_feature/NDTGraphMsg.h"

namespace Ogre
{
  class SceneNode;
}

namespace rviz
{
  class ColorProperty;
  class FloatProperty;
  class IntProperty;
}

namespace perception_oru{
namespace ndt_rviz_visualisation{

  class NDTLineVisual;

  class NDTVectorMapsDisplay: public rviz::MessageFilterDisplay<ndt_feature::NDTVectorMapsMsg>{
    Q_OBJECT
    public:

    NDTVectorMapsDisplay();
    virtual ~NDTVectorMapsDisplay();

  protected:
    virtual void onInitialize();

    virtual void reset();

  private Q_SLOTS:
    void updateColorAndAlpha();
    void updateHistoryLength();

  private:
    void processMessage(const ndt_feature::NDTVectorMapMsg::ConstPtr& msg);

    std::deque<boost::shared_ptr<NDTLineVisual> > visuals_;

    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::IntProperty* history_length_property_;
  };
}
}

#endif 

