#include "orient_govern/govern.hpp"

#include <rcpputils/env.hpp>
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::placeholders;

namespace orient_govern
{

static const std::vector<std::string> plugin_libs = {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_compute_path_through_poses_action_bt_node",
};

Govern::Govern(const rclcpp::NodeOptions &options)
    : nav2_util::LifecycleNode("orient_govern", "", options)
{

    RCLCPP_INFO(get_logger(), "Creating");
    declare_parameter("orient_clientid", rcpputils::get_env_var("ORIENT_CLIENTID"));
    declare_parameter("task_report_topic", "govern/task/report");
    declare_parameter("frequency", rclcpp::ParameterValue(0.5));
    declare_parameter("global_frame", "map");
    declare_parameter("timeout", 6.0);
    declare_parameter("behaviors", "");
    declare_parameter_if_not_declared(
        this, "plugin_lib_names", rclcpp::ParameterValue(plugin_libs));
    declare_parameter_if_not_declared(
        this, "transform_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(
        this, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
    declare_parameter_if_not_declared(
        this, "odom_topic", rclcpp::ParameterValue(std::string("odom")));
    current_check_point_.index = std::numeric_limits<uint32_t>::max();
}

nav2_util::CallbackReturn Govern::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring");

    orient_clientid_ = get_parameter("orient_clientid").as_string();
    if (orient_clientid_.empty()) {
        throw std::runtime_error("ORIENT_CLIENTID environment variable not set");
    }
    orient_behaviors_dir_ = get_parameter("behaviors").as_string();
    if (orient_behaviors_dir_.empty()) {
        throw std::runtime_error("behaviors is not set");
    }
    orient_behaviors_dir_ = orient_behaviors_dir_ + "/";

    service_timeout_ = std::make_shared<rclcpp::Duration>(
        rclcpp::Duration::from_seconds(get_parameter("timeout").as_double()));

    global_frame_ = get_parameter("global_frame").as_string();
    get_parameter("frequency", frequency_);

    global_frame_ = get_parameter("global_frame").as_string();
    robot_frame_ = get_parameter("robot_base_frame").as_string();
    transform_tolerance_ = get_parameter("transform_tolerance").as_double();
    odom_topic_ = get_parameter("odom_topic").as_string();

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    callback_group_executor_->add_callback_group(callback_group_, get_node_base_interface());


    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);
    tf_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

    auto service_qos = rclcpp::ServicesQoS().get_rmw_qos_profile();

    set_check_point_state_service_ = create_service<orient_interfaces::srv::SetCheckpointState>(
        "set_checkpoint_state",
        std::bind(
        &Govern::set_checkpoint_state, this,
        std::placeholders::_1, std::placeholders::_2));

    get_checkpoint_scout_service_ = create_service<orient_interfaces::srv::GetCheckpointScout>(
        "get_checkpoint_scout",
        std::bind(&Govern::get_checkpoint_scout, this, _1, _2));
    set_task_service_ = create_service<orient_interfaces::srv::SetTask>(
        "set_task",
        std::bind(&Govern::set_task, this, _1, _2));

    nav_to_pose_client_ = rclcpp_action::create_client<NavigateClientT>(shared_from_this(),"navigate_through_poses");

    // Odometry smoother object for getting current speed
    odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(shared_from_this(), 0.3, odom_topic_);
    nav2_bt_navigator::FeedbackUtils feedback_utils;
    feedback_utils.tf = tf_;
    feedback_utils.global_frame = global_frame_;
    feedback_utils.robot_frame = robot_frame_;
    feedback_utils.transform_tolerance = transform_tolerance_;
    poses_navigator_ = std::make_unique<NavigateThroughPosesNavigator>();

    auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();
    if (!poses_navigator_->on_configure( shared_from_this(), plugin_lib_names, feedback_utils, &plugin_muxer_, odom_smoother_)) {
        return nav2_util::CallbackReturn::FAILURE;
    }

    appraise_client_ = create_client<orient_interfaces::srv::CheckPointPose>("checkpoint_appraise",service_qos);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    checkpoint_state_publisher_ = create_publisher<orient_interfaces::msg::CheckPointStateMap>("checkpoint/state/apply", rclcpp::ServicesQoS());
    checkpoint_scout_publisher_ = create_publisher<orient_interfaces::msg::CheckPointStateMap>("checkpoint/scout", qos);
    checkpoint_track_publisher_ = create_publisher<orient_interfaces::msg::CheckpointTrack>("checkpoint/track", qos);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;
    // initialize to default
    checkpoint_state_sub_ = create_subscription<orient_interfaces::msg::CheckPointStateMap>(
        "checkpoints", rclcpp::ServicesQoS(),
        std::bind(&Govern::checkpoint_state_update, this, std::placeholders::_1));
    current_checkpoint_sub_ = create_subscription<orient_interfaces::msg::CheckPointStateStamp>(
        "checkpoint/current", rclcpp::ServicesQoS(),
        [this](const orient_interfaces::msg::CheckPointStateStamp::SharedPtr check_point_msg){
            if (active_) {
                current_checkpoint_update(check_point_msg);
            }
        },sub_opt);

    task_report_publisher_ = create_publisher<orient_interfaces::msg::TaskReport>(
        get_parameter("task_report_topic").as_string(), rclcpp::ServicesQoS());

    path_poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("checkpoint/path_poses", rclcpp::ServicesQoS());
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(int(frequency_ * 1000)), [this]{
        if (active_) {
            process();
        }
      },callback_group_);
    executor_thread_ = std::make_unique<nav2_util::NodeThread>(callback_group_executor_);
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Govern::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "Activating");
    task_report_publisher_->on_activate();
    checkpoint_state_publisher_->on_activate();
    checkpoint_scout_publisher_->on_activate();
    checkpoint_track_publisher_->on_activate();
    path_poses_pub_->on_activate();
    poses_navigator_->on_activate();
    active_ = true;

    createBond();

    return nav2_util::CallbackReturn::SUCCESS;
}


nav2_util::CallbackReturn Govern::on_deactivate(const rclcpp_lifecycle::State &)
{
    active_ = false;
    RCLCPP_INFO(get_logger(), "Deactivating");
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
    task_report_publisher_->on_deactivate();  
    path_poses_pub_->on_deactivate();      
    checkpoint_state_publisher_->on_deactivate();
    checkpoint_scout_publisher_->on_deactivate();
    checkpoint_track_publisher_->on_deactivate();
    poses_navigator_->on_deactivate();
    destroyBond();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Govern::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up");
    tf_listener_.reset();
    tf_.reset();
    task_report_publisher_.reset();
    nav_to_pose_client_.reset();
    appraise_client_.reset();
    path_poses_pub_.reset();
    checkpoint_state_publisher_.reset();
    checkpoint_scout_publisher_.reset();
    checkpoint_track_publisher_.reset();
    set_check_point_state_service_.reset();
    get_checkpoint_scout_service_.reset();
    checkpoint_tracks_.clear();;
    checkpoint_state_sub_.reset();
    current_checkpoint_sub_.reset();
    poses_navigator_->on_cleanup();
    poses_navigator_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Govern::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
}

void Govern::process() {
    if (current_task_ == nullptr) {
        return;
    }
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    switch (current_goal_status_)
    {
    case ActionStatus::IDLE:
    {
        start_new_task();
        break;
    }
    case ActionStatus::SUCCEEDED:
    {
        successed();
        break;
    }
    case ActionStatus::FAILED:
    {
        failed();
        break;
    }
    case ActionStatus::PROCESSING:
    {
        processing();
        break;
    }
    default:
    {
        break;
    }
    }
}

void Govern::publish_checkpoint_scout() {
    if (checkpoint_scout_publisher_->get_subscription_count()  == 0) {
        return;
    }
    orient_interfaces::msg::CheckPointStateMap state_map;
    state_map.header.stamp = this->now();
    state_map.header.frame_id = global_frame_;
    state_map.checkpoints =  checkpoint_tracks_;
    checkpoint_scout_publisher_->publish(state_map);


}
void Govern::publish_checkpoint_track() {
    if (checkpoint_track_publisher_->get_subscription_count()  == 0) {
        return;
    }
    orient_interfaces::msg::CheckpointTrack track;
    track.header.stamp = this->now();
    track.header.frame_id = global_frame_;
    track.last =  last_checkpoint_track_;
    track.remains =  checkpoint_tracks_.size();
    if (checkpoint_tracks_.empty()) {
        track.next.point.x = track.next.point.y = (uint32_t)-1;
    } else {
        track.next = checkpoint_tracks_.front();
    }
    checkpoint_track_publisher_->publish(track);
}

void Govern::processing() {
    publish_checkpoint_track();
}

void Govern::successed() {
    RCLCPP_INFO(get_logger(), "successed: task %s succeeded", current_task_->path_request.task_id.c_str());
    orient_interfaces::msg::TaskReport task_report;
    task_report.task_id = current_task_->path_request.task_id;
    task_report.state = orient_interfaces::msg::TaskReport::RESULT_SUCCESS;

    task_report_publisher_->publish(task_report);
    checkpoint_tracks_.clear();
    publish_checkpoint_scout();
    navigation_goal_handle_.reset();
    current_goal_status_ = ActionStatus::IDLE;
    current_check_point_.index = std::numeric_limits<uint32_t>::max();
    current_task_.reset();
}

void Govern::failed() {
    orient_interfaces::msg::TaskReport task_report;
    task_report.task_id = current_task_->path_request.task_id;
    task_report.state = orient_interfaces::msg::TaskReport::RESULT_UNDEFINED_FAILURE;
    task_report_publisher_->publish(task_report);
    current_task_.reset();
    checkpoint_tracks_.clear();
    publish_checkpoint_scout();
    navigation_goal_handle_.reset();
    current_goal_status_ = ActionStatus::IDLE;
    current_check_point_.index = std::numeric_limits<uint32_t>::max();
}

void Govern::start_new_task() {
    auto is_action_server_ready = appraise_client_->wait_for_service(std::chrono::seconds(5));
    if (!is_action_server_ready) {
        RCLCPP_ERROR(get_logger(), "Action server is not available after waiting");
        current_goal_status_ = ActionStatus::FAILED;
        return;
    }
    auto appraise_request = std::make_shared<orient_interfaces::srv::CheckPointPose::Request>();
    appraise_request->path_request = current_task_->path_request;
    appraise_client_->async_send_request(std::move(appraise_request),
        [this](rclcpp::Client<orient_interfaces::srv::CheckPointPose>::SharedFuture future) {
            auto result = future.get();
            if (result && result->track.size() > 0 && result->poses.size() > 0) {
                set_global_checkpoints(result->track);
                if (!send_navigate_goal(result->poses, result->start_path, current_task_->behavior)) {
                    current_goal_status_ = ActionStatus::FAILED;
                }
            } else {
                current_goal_status_ = ActionStatus::FAILED;
            }
        });
    current_goal_status_ = ActionStatus::PROCESSING;
}


void Govern::set_global_checkpoints(const std::vector<orient_interfaces::msg::CheckPointState> &track) {
    checkpoint_tracks_ = track;
    last_checkpoint_track_.point.index = std::numeric_limits<uint32_t>::max();
    current_check_point_.index = std::numeric_limits<uint32_t>::max();
    publish_checkpoint_scout();
}

void Govern::publish_checkpoint_state(const orient_interfaces::msg::CheckPoint &checkpoint,bool state) {
    orient_interfaces::msg::CheckPointStateMap state_map;
    orient_interfaces::msg::CheckPointState check_point_state;
    check_point_state.clientid = orient_clientid_;
    check_point_state.point = checkpoint;
    check_point_state.state = state;
    state_map.checkpoints.push_back(check_point_state);
    checkpoint_state_publisher_->publish(state_map);
}

void Govern::current_checkpoint_update(const orient_interfaces::msg::CheckPointStateStamp::SharedPtr check_point_msg) {
    const auto &checkpoint = check_point_msg->checkpoint;
    if (checkpoint_tracks_.empty()) {
        orient_interfaces::msg::CheckPointStateMap state_map;
        state_map.header = check_point_msg->header;
        if (current_check_point_.index != std::numeric_limits<uint32_t>::max()) {
            orient_interfaces::msg::CheckPointState check_point_state;
            check_point_state.clientid = orient_clientid_;
            check_point_state.point = current_check_point_;
            check_point_state.state = false;
            state_map.checkpoints.push_back(check_point_state);
        }
        if (checkpoint.point.index != std::numeric_limits<uint32_t>::max()) {
            orient_interfaces::msg::CheckPointState check_point_state;
            check_point_state.clientid = orient_clientid_;
            check_point_state.point = checkpoint.point;
            check_point_state.state = true;
            state_map.checkpoints.push_back(check_point_state);
        }
        checkpoint_state_publisher_->publish(state_map);
    } else if (checkpoint.point.index != std::numeric_limits<uint32_t>::max() && current_check_point_ != checkpoint.point ) {
        scout_checkpoint_track(checkpoint.point);
    }
    current_check_point_ = checkpoint.point;
}

void Govern::scout_checkpoint_track(const orient_interfaces::msg::CheckPoint &check_point) {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    auto iter = std::find_if(checkpoint_tracks_.begin(), checkpoint_tracks_.end(),[&](const auto &track){
        return track.point == check_point;
    });
    if (iter != std::end(checkpoint_tracks_)) {
        last_checkpoint_track_ = *iter;
        checkpoint_tracks_.erase(checkpoint_tracks_.begin(), iter + 1);  
    }
    publish_checkpoint_scout();
    publish_checkpoint_track();
}

bool Govern::send_navigate_goal(const std::vector<geometry_msgs::msg::Pose> &poses, int start_path, const std::string &behavior_tree) {
    NavigateActionClient::Goal navigate_goal;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = global_frame_;
    pose_stamped.header.stamp = now();
    if (start_path != -1) {
        pose_stamped.pose.position.x = pose_stamped.pose.position.y = -1;
        pose_stamped.pose.position.z = start_path;
        navigate_goal.poses.push_back(pose_stamped);
    }
    for (const geometry_msgs::msg::Pose &pose : poses) {
        pose_stamped.pose = pose;
        navigate_goal.poses.push_back(pose_stamped);
    }
    navigate_goal.behavior_tree = orient_behaviors_dir_ + behavior_tree;

    auto send_goal_options = rclcpp_action::Client<NavigateClientT>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&Govern::result_callback, this, _1);
    send_goal_options.goal_response_callback =std::bind(&Govern::goal_response_callback, this, _1);
    auto is_action_server_ready = nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready) {
        RCLCPP_ERROR(get_logger(), "Action server is not available after waiting");
        return false;
    }

    if (path_poses_pub_->get_subscription_count() > 0) {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.frame_id = global_frame_;
        pose_array.header.stamp = this->now();
        pose_array.poses = poses;
        path_poses_pub_->publish(pose_array);
    }
    nav_to_pose_client_->async_send_goal(navigate_goal, send_goal_options);
    return true;
}

void Govern::goal_response_callback(const rclcpp_action::ClientGoalHandle<NavigateClientT>::SharedPtr & goal) {
    if (goal) {
        navigation_goal_handle_ = goal;
    } else {
        navigation_goal_handle_.reset();
        current_goal_status_ = ActionStatus::FAILED;
    }

}
void Govern::result_callback(const rclcpp_action::ClientGoalHandle<NavigateClientT>::WrappedResult & result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        current_goal_status_ = ActionStatus::SUCCEEDED;
        break;
    default:
        current_goal_status_ = ActionStatus::FAILED;
        break;
    }
    navigation_goal_handle_.reset();
}


void Govern::checkpoint_state_update(const orient_interfaces::msg::CheckPointStateMap::SharedPtr checkpoint_map) {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    const auto &checkpoints = checkpoint_map->checkpoints;
    auto set_checkpoint_state = [&](orient_interfaces::msg::CheckPointState &track) {
        auto iter = std::find_if(checkpoints.begin(), checkpoints.end(),[&](const auto &checkpoint){
            return checkpoint.point == track.point;
        });
        if (iter != checkpoints.end()) {
            track.state = iter->state;
            track.clientid = iter->clientid;
        } else {
            track.state = false;
            track.clientid.clear();
        }
    };
    for(orient_interfaces::msg::CheckPointState &track : checkpoint_tracks_) {
        set_checkpoint_state(track);
    }
    set_checkpoint_state(last_checkpoint_track_);
}

void Govern::set_checkpoint_state(
        const std::shared_ptr<orient_interfaces::srv::SetCheckpointState::Request> request,
        std::shared_ptr<orient_interfaces::srv::SetCheckpointState::Response> response) {
    response->result = true;
    publish_checkpoint_state(request->checkpoint, request->state);
}

void Govern::get_checkpoint_scout(
    const std::shared_ptr<orient_interfaces::srv::GetCheckpointScout::Request>,
    std::shared_ptr<orient_interfaces::srv::GetCheckpointScout::Response> response) {
    response->checkpoint_tracks = checkpoint_tracks_;
}

void Govern::set_task(
    const std::shared_ptr<orient_interfaces::srv::SetTask::Request> request,
    std::shared_ptr<orient_interfaces::srv::SetTask::Response> response) {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    if (navigation_goal_handle_) {
        nav_to_pose_client_->async_cancel_goal(navigation_goal_handle_);
    }
    current_task_ = std::make_shared<orient_interfaces::msg::Task>(request->task);
    response->result = true;
    current_goal_status_ = ActionStatus::IDLE;
}
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(orient_govern::Govern)
