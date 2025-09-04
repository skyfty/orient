
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2_ros/buffer.h"


namespace orient_behavior_tree
{
class GetRobotPose :public BT::ConditionNode
{
public:
    GetRobotPose(
        const std::string & service_node_name,
        const BT::NodeConfiguration & conf):
        BT::ConditionNode(service_node_name, conf) {

        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        double transform_tolerance = 0.1;
        node_->get_parameter("transform_tolerance", transform_tolerance);
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

        node_->get_parameter("global_frame", global_frame_);

        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        robot_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "checkpoint_pose",
            rclcpp::QoS(1),
            [&](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
                robot_pose_.header = msg->header;
                robot_pose_.pose = msg->pose.pose;
            },
            sub_option);
        tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    }

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("robot_pose", "The robot's current pose"),
        };
    }

    BT::NodeStatus tick() {
        callback_group_executor_.spin_some();
        setOutput("robot_pose", robot_pose_);
        return BT::NodeStatus::SUCCESS;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_sub_;
    geometry_msgs::msg::PoseStamped robot_pose_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string global_frame_;
    tf2::Duration transform_tolerance_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orient_behavior_tree::GetRobotPose>("GetRobotPose");
}
