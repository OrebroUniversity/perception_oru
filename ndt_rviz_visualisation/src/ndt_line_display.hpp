#ifndef NDT_LINE_DISPLAY_H
#define NDT_LINE_DISPLAY_H

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
  class BoolProperty;
}

namespace lslgeneric{

  class NDTLineVisual;

  class NDTLineDisplay: public rviz::MessageFilterDisplay<ndt_map::NDTMapMsg>{
    Q_OBJECT
    public:

    NDTLineDisplay();
    virtual ~NDTLineDisplay();

  protected:
    virtual void onInitialize();

    virtual void reset();

  private Q_SLOTS:
    void updateColorAndAlpha();
    void updateHistoryLength();
    void updateDrawLines();

  private:
    void processMessage(const ndt_map::NDTMapMsg::ConstPtr& msg);

    std::vector<boost::shared_ptr<NDTLineVisual> > visuals_;

    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::IntProperty* history_length_property_;
    rviz::BoolProperty* draw_lines_;
  };
}

#endif 

