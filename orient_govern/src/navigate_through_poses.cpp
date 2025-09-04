#include "orient_govern/navigate_through_poses.hpp"
#include <rcpputils/env.hpp>

namespace orient_govern {

bool NavigateThroughPosesNavigator::configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
    if (!nav2_bt_navigator::NavigateThroughPosesNavigator::configure(parent_node, odom_smoother)) {
        return false;
    }
    auto node = parent_node.lock();

    default_bt_xml_filename_ =  getDefaultBTFilepath(node);

    if (!node->has_parameter("orient_clientid")) {
        node->declare_parameter("orient_clientid", rcpputils::get_env_var("ORIENT_CLIENTID"));
    }
    orient_clientid_ = node->get_parameter("orient_clientid").as_string();
    auto blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::string>("orient_clientid", orient_clientid_);
    return true;
}

bool NavigateThroughPosesNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
    if (!nav2_bt_navigator::NavigateThroughPosesNavigator::goalReceived(goal)) {
        return false;
    }
    return true;
}

void
NavigateThroughPosesNavigator::goalCompleted( 
    typename ActionT::Result::SharedPtr /*result*/, const nav2_behavior_tree::BtStatus /*final_bt_status*/)
{
    if (!bt_action_server_->loadBehaviorTree(default_bt_xml_filename_)) {
        RCLCPP_ERROR(
            logger_, "Error loading default XML file: %s.", default_bt_xml_filename_.c_str());
    }

}
}