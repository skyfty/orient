#include "orient_rviz_plugins/current_checkpoint_display.hpp"


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
CurrentCheckPointsDisplay::CurrentCheckPointsDisplay(rviz_common::DisplayContext *display_context, Ogre::SceneNode *scene_node)
    : CurrentCheckPointsDisplay()
{
    context_ = display_context;
    scene_node_ = scene_node;
    scene_manager_ = context_->getSceneManager();
    shapes_node_ = scene_node_->createChildSceneNode();
}
CurrentCheckPointsDisplay::CurrentCheckPointsDisplay()
    : rviz_common::MessageFilterDisplay<orient_interfaces::msg::CheckPointStateStamp>()
{
    
    initializeProperties();
}

CurrentCheckPointsDisplay::~CurrentCheckPointsDisplay()
{
    // Destructor implementation here
}
void CurrentCheckPointsDisplay::onInitialize()
{
    // Initialization code here
    rviz_common::MessageFilterDisplay<orient_interfaces::msg::CheckPointStateStamp>::onInitialize();

    MFDClass::onInitialize();
    shapes_node_ = scene_node_->createChildSceneNode();
    // Additional initialization code if needed
}


void CurrentCheckPointsDisplay::reset()
{
    // Reset code here
    rviz_common::MessageFilterDisplay<orient_interfaces::msg::CheckPointStateStamp>::reset();
   
    MFDClass::reset();
    shape_.reset();
}


void CurrentCheckPointsDisplay::processMessage(orient_interfaces::msg::CheckPointStateStamp::ConstSharedPtr msg) {
    setTransform(msg->header);
    // Check if the message is valid
    if (msg->checkpoint.point.x == (uint32_t)-1) {
        shape_.reset();
    } else {
        ogre_checkpoint_ = rviz_common::pointMsgToOgre(msg->checkpoint.position);
        updateDisplay();
    }
    context_->queueRender();
}


bool CurrentCheckPointsDisplay::setTransform(std_msgs::msg::Header const &header) {
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

void CurrentCheckPointsDisplay::updateDisplay() {
    float height = height_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();
    color.a = alpha_property_->getFloat();;

    shape_ = std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Cone, scene_manager_, shapes_node_);
    shape_->setPosition(ogre_checkpoint_);
    shape_->setScale(Ogre::Vector3(height, height, height));
    shape_->setOrientation(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X));
    shape_->setColor(color);
}

void CurrentCheckPointsDisplay::updateShapeChoice()
{
    shape_.reset();  // Reset the shape to ensure it is recreated with new properties
    shapes_node_->removeAndDestroyAllChildren();

    if (initialized()) {
        updateDisplay();
    }
}
void CurrentCheckPointsDisplay::initializeProperties()
{
    height_property_ = new rviz_common::properties::FloatProperty(
        "Height", 0.1, "Height of the shape.", this, SLOT(updateShapeChoice()));
    color_property_ = new rviz_common::properties::ColorProperty(
        "Color", QColor(204, 204, 204), "Color of the shape.", this, SLOT(updateShapeChoice()));
    alpha_property_ = new rviz_common::properties::FloatProperty(
        "Alpha", 1.0, "Alpha value of the shape.", this, SLOT(updateShapeChoice()));
    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);
}
}
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(orient_rviz_plugins::CurrentCheckPointsDisplay, rviz_common::Display)
// This macro registers the class as a plugin for RViz, allowing it to be loaded dynamically
// when RViz starts up. The class must be derived from rviz_common::Display or a subclass of it.