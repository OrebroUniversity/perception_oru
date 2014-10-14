#ifndef NDT_DISPLAY_H
#define NDT_DISPLAY_H

#include <boost/circular_buffer.hpp>

#include <ndt_map/NDTMapMsg.h>
#include <rviz/message_filter_display.h>

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

namespace lslgeneric{

  class NDTVisual;

  class NDTDisplay: public rviz::MessageFilterDisplay<ndt_map::NDTMapMsg>{
    Q_OBJECT
    public:

    NDTDisplay();
    virtual ~NDTDisplay();

  protected:
    virtual void onInitialize();

    virtual void reset();

  private Q_SLOTS:
    void updateColorAndAlpha();
    void updateHistoryLength();

  private:
    void processMessage(const ndt_map::NDTMapMsg::ConstPtr& msg);

    std::vector<boost::shared_ptr<NDTVisual> > visuals_;

    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::IntProperty* history_length_property_;
  };
}

#endif 

