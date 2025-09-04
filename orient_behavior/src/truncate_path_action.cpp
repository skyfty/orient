#include <memory>
#include <string>
#include <limits>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "tf2_ros/buffer.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace orient_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to shorten path to some distance around robot
 */
class TruncatePath : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::TruncatePathLocal constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TruncatePath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf)
{
}


  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
      BT::OutputPort<nav_msgs::msg::Path>(
        "output_path", "Path truncated to a certain distance around robot"),
      BT::InputPort<double>(
        "distance_forward", 8.0,
        "Distance in forward direction"),
      BT::InputPort<double>(
        "distance_backward", 0.0,
        "Distance in backward direction"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "robot_pose", "Manually specified pose to be used"
        "if overriding current robot pose"),
      BT::InputPort<double>(
        "angular_distance_weight", 0.0,
        "Weight of angular distance relative to positional distance when finding which path "
        "pose is closest to robot. Not applicable on paths without orientations assigned"),
      BT::InputPort<double>(
        "max_robot_pose_search_dist", std::numeric_limits<double>::infinity(),
        "Maximum forward integrated distance along the path (starting from the last detected pose) "
        "to bound the search for the closest pose to the robot. When set to infinity (default), "
        "whole path is searched every time"),
    };
  }

private:

    void halt() override {}


    BT::NodeStatus tick() override
    {
        setStatus(BT::NodeStatus::RUNNING);

        double distance_forward, distance_backward;
        geometry_msgs::msg::PoseStamped pose;
        double angular_distance_weight;
        double max_robot_pose_search_dist;

        getInput("distance_forward", distance_forward);
        getInput("distance_backward", distance_backward);
        getInput("angular_distance_weight", angular_distance_weight);
        getInput("max_robot_pose_search_dist", max_robot_pose_search_dist);

        bool path_pruning = std::isfinite(max_robot_pose_search_dist);
        nav_msgs::msg::Path new_path;
        getInput("input_path", new_path);
        if (!path_pruning || new_path != path_) {
            path_ = new_path;
            closest_pose_detection_begin_ = path_.poses.begin();
        }
        getInput("robot_pose", pose);

        if (path_.poses.empty()) {
            setOutput("output_path", path_);
            return BT::NodeStatus::SUCCESS;
        }

        auto closest_pose_detection_end = path_.poses.end();
        if (path_pruning) {
            closest_pose_detection_end = nav2_util::geometry_utils::first_after_integrated_distance(
            closest_pose_detection_begin_, path_.poses.end(), max_robot_pose_search_dist);
        }

        // find the closest pose on the path
        auto current_pose = nav2_util::geometry_utils::min_by(
            closest_pose_detection_begin_, closest_pose_detection_end,
            [&pose, angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
            return poseDistance(pose, ps, angular_distance_weight);
            });

        if (path_pruning) {
            closest_pose_detection_begin_ = current_pose;
        }

        // expand forwards to extract desired length
        auto forward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
            current_pose, path_.poses.end(), distance_forward);

        // expand backwards to extract desired length
        // Note: current_pose + 1 is used because reverse iterator points to a cell before it
        auto backward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
            std::reverse_iterator(current_pose + 1), path_.poses.rend(), distance_backward);

        nav_msgs::msg::Path output_path;
        output_path.header = path_.header;
        output_path.poses = std::vector<geometry_msgs::msg::PoseStamped>(
            backward_pose_it.base(), forward_pose_it);
        setOutput("output_path", output_path);

        return BT::NodeStatus::SUCCESS;
    }

    static double poseDistance(
        const geometry_msgs::msg::PoseStamped & pose1,
        const geometry_msgs::msg::PoseStamped & pose2,
        const double angular_distance_weight)
    {
        double dx = pose1.pose.position.x - pose2.pose.position.x;
        double dy = pose1.pose.position.y - pose2.pose.position.y;
        // taking angular distance into account in addition to spatial distance
        // (to improve picking a correct pose near cusps and loops)
        tf2::Quaternion q1;
        tf2::convert(pose1.pose.orientation, q1);
        tf2::Quaternion q2;
        tf2::convert(pose2.pose.orientation, q2);
        double da = angular_distance_weight * std::abs(q1.angleShortestPath(q2));
        return std::sqrt(dx * dx + dy * dy + da * da);
    }

  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path::_poses_type::iterator closest_pose_detection_begin_;
};

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<orient_behavior_tree::TruncatePath>("TruncatePath");
}
