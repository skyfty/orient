
#ifndef ORIENT_RVIZ_PLUGINS__CURRENT_CHECKPOINT_DISPLAY_HPP_
#define ORIENT_RVIZ_PLUGINS__CURRENT_CHECKPOINT_DISPLAY_HPP_


#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/ros_topic_display.hpp"
#include "tf2_ros/transform_listener.h"
#include "orient_interfaces/msg/check_point_state_stamp.hpp"      // CHANGE

namespace Ogre
{
    class ManualObject;
} // namespace Ogre

namespace rviz_common
{
    namespace properties
    {
        class EnumProperty;
        class ColorProperty;
        class FloatProperty;
        class IntProperty;
        class BoolProperty; 
        class VectorProperty;
        class StringProperty;
        class QuaternionProperty;
        class VectorProperty;
    } // namespace properties
} // namespace rviz_common


namespace orient_rviz_plugins
{
class CurrentCheckPointsDisplay : public rviz_common::MessageFilterDisplay<orient_interfaces::msg::CheckPointStateStamp>
{
    Q_OBJECT

public:
    CurrentCheckPointsDisplay(rviz_common::DisplayContext *display_context, Ogre::SceneNode *scene_node);
    CurrentCheckPointsDisplay();
    ~CurrentCheckPointsDisplay() override;

private Q_SLOTS:
    /// Update arrow geometry
    void updateShapeChoice();
  
protected:
    void onInitialize() override;
    void reset() override;
    void updateDisplay();
    void processMessage(orient_interfaces::msg::CheckPointStateStamp::ConstSharedPtr msg) override;

    bool setTransform(std_msgs::msg::Header const & header);
    void initializeProperties();
    
private:
    Ogre::SceneNode * shapes_node_;
    std::unique_ptr<rviz_rendering::Shape> shape_;
    Ogre::Vector3 ogre_checkpoint_;
    rviz_common::properties::ColorProperty * color_property_;
    rviz_common::properties::FloatProperty * height_property_;
    rviz_common::properties::FloatProperty * alpha_property_;
};

}
#endif