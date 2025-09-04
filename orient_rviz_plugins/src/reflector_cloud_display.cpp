
#include "orient_rviz_plugins/reflector_cloud_display.hpp"

#include <memory>
#include <string>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/validate_floats.hpp"

namespace orient_rviz_plugins
{
ReflectorCloudDisplay::ReflectorCloudDisplay(rviz_common::DisplayContext *display_context, Ogre::SceneNode *scene_node)
    : ReflectorCloudDisplay()
{
    context_ = display_context;
    scene_node_ = scene_node;
    scene_manager_ = context_->getSceneManager();
    shapes_node_ = scene_node_->createChildSceneNode();
}
ReflectorCloudDisplay::ReflectorCloudDisplay()
    : rviz_common::MessageFilterDisplay<orient_interfaces::msg::ReflectorGrid>()
{
    
    initializeProperties();

    shape_property_->addOption("Recognize", Recognize);
    shape_property_->addOption("Map",Map);
    shape_property_->addOption("Matched", Matched);
}

ReflectorCloudDisplay::~ReflectorCloudDisplay()
{
    // Destructor implementation here
}
void ReflectorCloudDisplay::onInitialize()
{
    // Initialization code here
    rviz_common::MessageFilterDisplay<orient_interfaces::msg::ReflectorGrid>::onInitialize();

    MFDClass::onInitialize();
    shapes_node_ = scene_node_->createChildSceneNode();
    // Additional initialization code if needed
}
void ReflectorCloudDisplay::reset()
{
    // Reset code here
    rviz_common::MessageFilterDisplay<orient_interfaces::msg::ReflectorGrid>::reset();
   
    MFDClass::reset();
    shapes_.clear();
}
void ReflectorCloudDisplay::processMessage(orient_interfaces::msg::ReflectorGrid::ConstSharedPtr msg)
{
    setTransform(msg->header);
    // Check if the message is valid
    ogre_reflectors_.resize(msg->reflectors.size());

    for (std::size_t i = 0; i < msg->reflectors.size(); ++i) {
 
        
        ogre_reflectors_[i].position = rviz_common::pointMsgToOgre(msg->reflectors[i].point);
        ogre_reflectors_[i].intensity = static_cast<float>(msg->reflectors[i].intensity);
        ogre_reflectors_[i].diameter = static_cast<float>(msg->reflectors[i].diameter);
    }
    updateDisplay();

    context_->queueRender();
}

void ReflectorCloudDisplay::setIntensityColor(rviz_rendering::Shape *shapes,float intensity)
{
    float r, g, b;

    // Convert intensity to rainbow color
    float normalized_intensity = std::min(std::max(intensity, 0.0f), 1.0f); // Clamp between 0 and 1
    float hue = normalized_intensity * 240.0f; // Map intensity to hue (0-240 degrees)
    float s = 1.0f; // Full saturation
    float v = 1.0f; // Full brightness

    // Convert HSV to RGB
    int h_i = static_cast<int>(hue / 60.0f) % 6;
    float f = (hue / 60.0f) - h_i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - f * s);
    float t = v * (1.0f - (1.0f - f) * s);

    switch (h_i) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        case 5: r = v; g = p; b = q; break;
        default: r = g = b = 0.0f; break;
    }

    shapes->setColor(r, g, b, 1.0f);
}

void ReflectorCloudDisplay::updateDisplay() {
    while (shapes_.size() < ogre_reflectors_.size()) {
        shapes_.emplace_back(std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Cylinder, scene_manager_, shapes_node_));
    }
    
    while (shapes_.size() > ogre_reflectors_.size()) {
        shapes_.pop_back();
    }

    float height = height_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();
    color.a = alpha_property_->getFloat();

    int shape_type = shape_property_->getOptionInt();
    for (std::size_t i = 0; i < ogre_reflectors_.size(); ++i) {
        auto &shapes = shapes_[i];
        auto &reflector = ogre_reflectors_[i];
        shapes->setPosition(reflector.position);
        shapes->setScale(Ogre::Vector3(height, height, height));
        shapes->setOrientation(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X));

        switch(shape_type) {
            case Recognize: {
                setIntensityColor(shapes.get(), reflector.intensity);
                break;
            }
            case Matched: {
                shapes->setColor(color);
                break;
            }
            default: {
                shapes->setColor(color);
                break;
            }
        }
    }
}

bool ReflectorCloudDisplay::setTransform(std_msgs::msg::Header const &header)
{
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(header, position, orientation)) {
        setMissingTransformToFixedFrame(header.frame_id);
        return false;
    }
    setTransformOk();

    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
    return true;
}

void ReflectorCloudDisplay::updateShapeChoice()
{
    shapes_.clear();
    shapes_node_->removeAndDestroyAllChildren();

    if (initialized()) {
        updateDisplay();
    }
}
void ReflectorCloudDisplay::initializeProperties()
{
    shape_property_ = new rviz_common::properties::EnumProperty(
        "Type", "Recognize", "Shape to display the pose as.", this, SLOT(updateShapeChoice()));
    height_property_ = new rviz_common::properties::FloatProperty(
        "Height", 0.1, "Height of the shape.", this, SLOT(updateShapeChoice()));
    color_property_ = new rviz_common::properties::ColorProperty(
        "Color", QColor(204, 204, 204), "Color of the shape.", this, SLOT(updateShapeChoice()));
    alpha_property_ = new rviz_common::properties::FloatProperty(
        "Alpha", 1.0, "Alpha value of the shape.", this, SLOT(updateShapeChoice()));
    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);
}
} // namespace orient_rviz_plugins
// Include the necessary headers for the display
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(orient_rviz_plugins::ReflectorCloudDisplay, rviz_common::Display)
// This macro registers the class as a plugin for RViz, allowing it to be loaded dynamically
// when RViz starts up. The class must be derived from rviz_common::Display or a subclass of it.