
#include <chrono>
#include <string>
#include <memory>
#include <fstream>
#include <sys/wait.h>
#include <std_msgs/msg/string.hpp>
#include "nav2_behaviors/timed_behavior.hpp"
#include "orient_interfaces/action/forklift.hpp"

using namespace nav2_behaviors;

namespace nav2_behaviors {
using ForkliftAction = orient_interfaces::action::Forklift;

class Forklift : public nav2_behaviors::TimedBehavior<ForkliftAction>
{
public:
    Forklift()
        : nav2_behaviors::TimedBehavior<ForkliftAction>(),
        feedback_(std::make_shared<ForkliftAction::Feedback>())
    {
    }

    ~Forklift() = default;

    Status onRun(const std::shared_ptr<const ForkliftAction::Goal> goal) override {
        RCLCPP_INFO(logger_, "Forklift action started with action: %s", goal->action_name.c_str());
        std_msgs::msg::String cmd_msg;
        cmd_msg.data = goal->action_name;
        cmd_pub_->publish(cmd_msg);
        
        action_server_->publish_feedback(feedback_);
        wait_end_ = node_.lock()->now() + rclcpp::Duration(goal->time);

        return Status::SUCCEEDED;
    }

    Status onCycleUpdate() override {
        auto current_point = node_.lock()->now();
        auto time_left = wait_end_ - current_point;
        feedback_->time_left = time_left;
        action_server_->publish_feedback(feedback_);

        if (time_left.nanoseconds() > 0) {
            return Status::RUNNING;
        } else {
            return Status::SUCCEEDED;
        }

    }
   

    void onConfigure() override {
        RCLCPP_INFO(logger_, "Configuring Forklift behavior");
        auto node = node_.lock();
        if (!node) {
            throw std::runtime_error{"Failed to lock node"};
        }

        // 创建一个Publisher用于发布到"dac63004/cmd"
        cmd_pub_ = node_.lock()->create_publisher<std_msgs::msg::String>("dac63004/cmd", 10);

    }
protected:
  ForkliftAction::Feedback::SharedPtr feedback_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
  rclcpp::Time wait_end_;
};

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Forklift, nav2_core::Behavior)
