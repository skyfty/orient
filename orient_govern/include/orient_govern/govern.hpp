#ifndef GOVERN_HPP
#define GOVERN_HPP
#include <mutex>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "orient_interfaces/msg/task.hpp"
#include "orient_interfaces/msg/task_report.hpp"
#include "orient_interfaces/srv/check_point_pose.hpp"
#include "orient_interfaces/srv/set_task.hpp"
#include "orient_interfaces/action/follow_checkpoints.hpp"             // CHANGE
#include "orient_interfaces/msg/check_point_state.hpp"             // CHANGE
#include "orient_interfaces/msg/check_point_map.hpp"             // CHANGE
#include "orient_interfaces/msg/check_point.hpp"             // CHANGE
#include "orient_interfaces/msg/check_points.hpp"             // CHANGE
#include "orient_interfaces/srv/load_check_point.hpp"             // CHANGE
#include "orient_interfaces/msg/check_point_state_map.hpp"             // CHANGE
#include "orient_interfaces/msg/checkpoint_track.hpp"             // CHANGE

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "orient_interfaces/srv/set_checkpoint_state.hpp"             // CHANGE
#include "orient_interfaces/srv/is_checkpoint_employed.hpp"             // CHANGE
#include "orient_interfaces/srv/is_checkpoint_free.hpp"             // CHANGE
#include "orient_interfaces/srv/is_checkpoint_track_empty.hpp"             // CHANGE
#include "orient_interfaces/srv/get_checkpoint_scout.hpp"             // CHANGE

#include <rmf_fleet_msgs/msg/robot_mode.hpp>

#include "orient_interfaces/msg/check_point_state_stamp.hpp"             // CHANGE
#include "orient_interfaces/action/forklift.hpp"             // CHANGE

#include "nav2_util/service_client.hpp"
#include "orient_govern/navigate_through_poses.hpp"
#include <queue>

namespace orient_govern {

enum class ActionStatus
{
    IDLE = 0,
    PROCESSING = 1,
    FAILED = 2,
    SUCCEEDED = 3
};

class Govern : public nav2_util::LifecycleNode {
public:
    using NavigateClientT = nav2_msgs::action::NavigateThroughPoses;
    using NavigateActionClient = rclcpp_action::Client<NavigateClientT>;
    using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<NavigateClientT>;

    explicit Govern(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~Govern() = default;

protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

    void process();
    void processing();
    void failed();
    void successed();
    void start_new_task();
    bool send_navigate_goal(const std::vector<geometry_msgs::msg::Pose> &poses, int start_path,const std::string& behavior_tree);
    void checkpoint_state_update(const orient_interfaces::msg::CheckPointStateMap::SharedPtr check_point);
    void current_checkpoint_update(const orient_interfaces::msg::CheckPointStateStamp::SharedPtr check_point);
    void publish_checkpoint_state(const orient_interfaces::msg::CheckPoint &checkpoint,bool state);
    void publish_checkpoint_scout();
    void publish_checkpoint_track();
    void set_global_checkpoints(const std::vector<orient_interfaces::msg::CheckPointState> &track);
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateClientT>::SharedPtr & goal);
    void result_callback(const rclcpp_action::ClientGoalHandle<NavigateClientT>::WrappedResult & result);
    void scout_checkpoint_track(const orient_interfaces::msg::CheckPoint &check_point);

public:
    void set_checkpoint_state(
        const std::shared_ptr<orient_interfaces::srv::SetCheckpointState::Request> request,
        std::shared_ptr<orient_interfaces::srv::SetCheckpointState::Response> response);
    void get_checkpoint_scout(
        const std::shared_ptr<orient_interfaces::srv::GetCheckpointScout::Request> request,
        std::shared_ptr<orient_interfaces::srv::GetCheckpointScout::Response> response);
    void set_task(
        const std::shared_ptr<orient_interfaces::srv::SetTask::Request> request,
        std::shared_ptr<orient_interfaces::srv::SetTask::Response> response);


protected:

    std::string orient_clientid_;
    std::string global_frame_;
    std::string robot_frame_;
    double transform_tolerance_;
    std::string odom_topic_;
    std::shared_ptr<rclcpp::Duration> service_timeout_;
    rclcpp::Client<orient_interfaces::srv::CheckPointPose>::SharedPtr appraise_client_;

    NavigateActionClient::SharedPtr nav_to_pose_client_;
    NavigationGoalHandle::SharedPtr navigation_goal_handle_;

    nav2_bt_navigator::NavigatorMuxer plugin_muxer_;

    // Odometry smoother object
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

    // Spinning transform that can be used by the BT nodes
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<NavigateThroughPosesNavigator>  poses_navigator_;

    orient_interfaces::msg::Task::SharedPtr current_task_;

    rclcpp_lifecycle::LifecyclePublisher<orient_interfaces::msg::TaskReport>::SharedPtr task_report_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr path_poses_pub_;
    rclcpp::Subscription<orient_interfaces::msg::CheckPointStateMap>::SharedPtr checkpoint_state_sub_;
    rclcpp::Subscription<orient_interfaces::msg::CheckPointStateStamp>::SharedPtr current_checkpoint_sub_;
    rclcpp_lifecycle::LifecyclePublisher<orient_interfaces::msg::CheckPointStateMap>::SharedPtr checkpoint_state_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<orient_interfaces::msg::CheckPointStateMap>::SharedPtr checkpoint_scout_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<orient_interfaces::msg::CheckpointTrack>::SharedPtr checkpoint_track_publisher_;
    rclcpp::Service<orient_interfaces::srv::SetCheckpointState>::SharedPtr set_check_point_state_service_;
    rclcpp::Service<orient_interfaces::srv::GetCheckpointScout>::SharedPtr get_checkpoint_scout_service_;
    rclcpp::Service<orient_interfaces::srv::SetTask>::SharedPtr set_task_service_;

    std::string orient_behaviors_dir_;
    ActionStatus current_goal_status_ = ActionStatus::IDLE;
    double frequency_;
    std::atomic<bool> active_{false};
    rclcpp::TimerBase::SharedPtr timer_;
    mutable std::recursive_mutex update_mutex_;
    orient_interfaces::msg::CheckPoint current_check_point_;
    orient_interfaces::msg::CheckPointState last_checkpoint_track_;
    std::vector<orient_interfaces::msg::CheckPointState> checkpoint_tracks_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
    std::unique_ptr<nav2_util::NodeThread> executor_thread_;
};

}

#endif // GOVERN_HPP
