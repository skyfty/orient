#ifndef ORIENT__NAVIGATE_THROUGH_POSES_HPP_
#define ORIENT__NAVIGATE_THROUGH_POSES_HPP_

#include "nav2_bt_navigator/navigators/navigate_through_poses.hpp"


namespace orient_govern {
class NavigateThroughPosesNavigator : public nav2_bt_navigator::NavigateThroughPosesNavigator
{
public:
    NavigateThroughPosesNavigator()
    : nav2_bt_navigator::NavigateThroughPosesNavigator() {}

    bool configure(
        rclcpp_lifecycle::LifecycleNode::WeakPtr node,
        std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

protected:
    bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;
    void goalCompleted(typename ActionT::Result::SharedPtr result,const nav2_behavior_tree::BtStatus final_bt_status) override;
protected:
    // Add any additional members or methods if needed
    std::string orient_clientid_;
    std::string default_bt_xml_filename_;
};
}

#endif