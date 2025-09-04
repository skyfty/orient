#include "orient_rviz_plugins/checkpoint_display.hpp"

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
CheckPointsDisplay::CheckPointsDisplay(rviz_common::DisplayContext *display_context, Ogre::SceneNode *scene_node)
    : CheckPointsDisplay()
{
    context_ = display_context;
    scene_node_ = scene_node;
    scene_manager_ = context_->getSceneManager();
    shapes_node_ = scene_node_->createChildSceneNode();
}
CheckPointsDisplay::CheckPointsDisplay()
    : rviz_common::MessageFilterDisplay<orient_interfaces::msg::CheckPointStateMap>()
{
    
    initializeProperties();
}

CheckPointsDisplay::~CheckPointsDisplay()
{
    // Destructor implementation here
}
void CheckPointsDisplay::onInitialize()
{
    // Initialization code here
    rviz_common::MessageFilterDisplay<orient_interfaces::msg::CheckPointStateMap>::onInitialize();

    MFDClass::onInitialize();
    shapes_node_ = scene_node_->createChildSceneNode();
    // Additional initialization code if needed
}


void CheckPointsDisplay::reset()
{
    // Reset code here
    rviz_common::MessageFilterDisplay<orient_interfaces::msg::CheckPointStateMap>::reset();
   
    MFDClass::reset();
    shapes_.clear();
}


void CheckPointsDisplay::processMessage(orient_interfaces::msg::CheckPointStateMap::ConstSharedPtr msg) {
    msg_ = msg;
    setTransform(msg->header);
    // Check if the message is valid
    ogre_checkpoints_.resize(msg_->checkpoints.size());

    for (std::size_t i = 0; i < msg_->checkpoints.size(); ++i) {
        ogre_checkpoints_[i] = rviz_common::pointMsgToOgre(msg_->checkpoints[i].position);
    }
    updateDisplay();
    context_->queueRender();
}


bool CheckPointsDisplay::setTransform(std_msgs::msg::Header const &header) {
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

void CheckPointsDisplay::updateDisplay() {
    float height = height_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();
    color.a = alpha_property_->getFloat();
    float height2 = height2_property_->getFloat();
    Ogre::ColourValue color2 = color2_property_->getOgreColor();
    color2.a = alpha_property_->getFloat();

    shapes_.clear();
    if (msg_ == nullptr) {
        return;
    }
    for (std::size_t i = 0; i < msg_->checkpoints.size(); ++i) {
        auto shape1 = std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Cone, scene_manager_, shapes_node_);
        shape1->setPosition(ogre_checkpoints_[i]);
        shape1->setScale(Ogre::Vector3(height, height, height));
        shape1->setOrientation(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X));
        shape1->setColor(color);
        shapes_.push_back(std::move(shape1));
        if (!msg_->checkpoints[i].clientid.empty()) {
            auto shape2 = std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Cylinder, scene_manager_, shapes_node_);
            shape2->setPosition(ogre_checkpoints_[i]);
            shape2->setScale(Ogre::Vector3(height2, height2, height2));
            shape2->setOrientation(Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_X));
            shape2->setColor(color2);
            shapes_.push_back(std::move(shape2));
        }
    }
}

void CheckPointsDisplay::updateShapeChoice()
{
    shapes_.clear();
    shapes_node_->removeAndDestroyAllChildren();

    if (initialized()) {
        updateDisplay();
    }
}

void CheckPointsDisplay::initializeProperties()
{
    height_property_ = new rviz_common::properties::FloatProperty(
        "Height", 0.1, "Height of the shape.", this, SLOT(updateShapeChoice()));
    color_property_ = new rviz_common::properties::ColorProperty(
        "Color", QColor(204, 204, 204), "Color of the shape.", this, SLOT(updateShapeChoice()));
    height2_property_ = new rviz_common::properties::FloatProperty(
        "Height2", 0.1, "Height2 of the shape.", this, SLOT(updateShapeChoice()));
    color2_property_ = new rviz_common::properties::ColorProperty(
        "Color2", QColor(204, 204, 204), "Color2 of the shape.", this, SLOT(updateShapeChoice()));
    alpha_property_ = new rviz_common::properties::FloatProperty(
        "Alpha", 1.0, "Alpha value of the shape.", this, SLOT(updateShapeChoice()));
    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);
}
}

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(orient_rviz_plugins::CheckPointsDisplay, rviz_common::Display)
// This macro registers the class as a plugin for RViz, allowing it to be loaded dynamically
// when RViz starts up. The class must be derived from rviz_common::Display or a subclass of it.