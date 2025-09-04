
#ifndef ORIENT_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__PARTICLE_CLOUD_DISPLAY_HPP_
#define ORIENT_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__PARTICLE_CLOUD_DISPLAY_HPP_

#include <memory>
#include <vector>

#include "orient_interfaces/msg/reflector_grid.hpp"     // CHANGE
#include "orient_interfaces/msg/reflector.hpp"      // CHANGE
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/ros_topic_display.hpp"
#include "tf2_ros/transform_listener.h"

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
struct OgreReflector
{
  Ogre::Vector3 position;

  float diameter;
  float intensity; 
};

/** @brief Displays a nav2_msgs/ParticleCloud message as a bunch of line-drawn weighted arrows. */
class ReflectorCloudDisplay : public rviz_common::MessageFilterDisplay<orient_interfaces::msg::ReflectorGrid>
{
    Q_OBJECT

    enum {
        Recognize = 0,
        Map = 1,
        Matched = 2,
    };
    
public:
    // TODO(botteroa-si): Constructor for testing, remove once ros_nodes can be mocked and call
    // initialize instead
    ReflectorCloudDisplay(rviz_common::DisplayContext *display_context, Ogre::SceneNode *scene_node);
    ReflectorCloudDisplay();
    ~ReflectorCloudDisplay() override;

    void processMessage(orient_interfaces::msg::ReflectorGrid::ConstSharedPtr msg) override;

protected:
    void onInitialize() override;
    void reset() override;

private Q_SLOTS:
    /// Update arrow geometry
    void updateShapeChoice();
  
private:
    void initializeProperties();
    bool setTransform(std_msgs::msg::Header const & header);
    void updateDisplay();
    void setIntensityColor(rviz_rendering::Shape *shapes,float intensity);

private:
    std::vector<std::unique_ptr<rviz_rendering::Shape>> shapes_;
    std::vector<OgreReflector> ogre_reflectors_;
    rviz_common::properties::EnumProperty * shape_property_;
    rviz_common::properties::ColorProperty * color_property_;
    rviz_common::properties::FloatProperty * height_property_;
    rviz_common::properties::FloatProperty * alpha_property_;

    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    Ogre::SceneNode * shapes_node_;
    float min_length_;
    float max_length_;
};
}
#endif // ORIENT_RVIZ_PLUGINS__PARTICLE_CLOUD_DISPLAY__PARTICLE_CLOUD_DISPLAY_HPP_