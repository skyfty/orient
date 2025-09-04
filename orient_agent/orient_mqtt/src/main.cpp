
#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <iostream>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <mqtt/async_client.h>
#include "nav2_util/lifecycle_node.hpp"
#include <rcpputils/env.hpp>
#include "orient_reflector/utils.hpp"
#include "yaml-cpp/yaml.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include <nav_msgs/msg/odometry.hpp>
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "orient_interfaces/msg/reflector_map.hpp" // CHANGE
#include "orient_checkpoint/utils.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "orient_interfaces/msg/check_point_map.hpp" // CHANGE
#include "orient_interfaces/msg/check_point_state.hpp" // CHANGE
#include "orient_interfaces/msg/check_point_state_stamp.hpp" // CHANGE
#include "orient_interfaces/msg/task.hpp" // CHANGE
#include "orient_interfaces/srv/set_task.hpp" // CHANGE

#include "orient_interfaces/srv/load_check_point.hpp" // CHANGE
#include "orient_interfaces/srv/load_reflector_map.hpp" // CHANGE
#include "orient_interfaces/srv/get_reflector_map.hpp" // CHANGE
#include "geometry_msgs/msg/pose_stamped.hpp" // CHANGE
#include "geometry_msgs/msg/pose_array.hpp" // CHANGE
#include "geometry_msgs/msg/pose.hpp" // CHANGE
#include "orient_interfaces/msg/task_report.hpp" // CHANGE
#include "orient_interfaces/msg/agent_state.hpp" // CHANGE
#include "orient_interfaces/msg/audio.hpp" // CHANGE
#include "orient_interfaces/msg/check_point_state_map.hpp" // CHANGE
#include "nav2_util/robot_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/pause_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>

#include <rclcpp/qos.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::placeholders;
/**
 * @brief Struct containing broker parameters
 */
struct BrokerConfig
{
    std::string host; ///< broker host
    int port;         ///< broker port
    std::string user; ///< username
    std::string pass; ///< password
};
/**
 * @brief Struct containing client parameters
 */
struct ClientConfig
{
    std::string ip; ///< client IP address                        ///< client buffer-related variables
    struct
    {
        std::string topic;      ///< last-will topic
        std::string message;    ///< last-will message
        int qos;                ///< last-will QoS value
        bool retained;          ///< whether last-will is retained
    } last_will;                ///< last-will-related variables
    bool clean_session;         ///< whether client requests clean session
    double keep_alive_interval; ///< keep-alive interval
    int max_inflight;           ///< maximum number of inflight messages                                  ///< SSL/TLS-related variables
};

class Mqtt : public rclcpp::Node,
             public virtual mqtt::callback,
             public virtual mqtt::iaction_listener
{

    using LoadCheckPointClientT = orient_interfaces::srv::LoadCheckPoint;
    using LoadCheckPointActionClient = rclcpp::Client<LoadCheckPointClientT>;


    using LoadReflectorMapClientT = orient_interfaces::srv::LoadReflectorMap;
    using LoadReflectorMapActionClient = rclcpp::Client<LoadReflectorMapClientT>;
    
public:
    Mqtt(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("orient_agent_mqtt_node", options)
    {
        RCLCPP_INFO(get_logger(), "Creating");

        rcl_interfaces::msg::ParameterDescriptor param_desc;

        param_desc.description = "IP address or hostname of the machine running the MQTT broker";
        declare_parameter("broker.host", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
        param_desc.description = "port the MQTT broker is listening on";
        declare_parameter("broker.port", rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);

        param_desc.description = "unique ID used to identify the client (broker may allow empty ID and automatically generate one)";
        declare_parameter("client.id", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
        param_desc.description = "maximum number of messages buffered by the bridge when not connected to broker (only available if client ID is not empty)";
        declare_parameter("client.buffer.size", rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);
        param_desc.description = "directory used to buffer messages when not connected to broker (relative to ROS_HOME)";
        declare_parameter("client.buffer.directory", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
        param_desc.description = "topic used for this client's last-will message (no last will, if not specified)";
        declare_parameter("client.last_will.topic", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
        param_desc.description = "last-will message";
        declare_parameter("client.last_will.message", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
        param_desc.description = "QoS value for last-will message";
        declare_parameter("client.last_will.qos", rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);
        param_desc.description = "whether to retain last-will message";
        declare_parameter("client.last_will.retained", rclcpp::ParameterType::PARAMETER_BOOL, param_desc);
        param_desc.description = "whether to use a clean session for this client";
        declare_parameter("client.clean_session", rclcpp::ParameterType::PARAMETER_BOOL, param_desc);
        param_desc.description = "keep-alive interval in seconds";
        declare_parameter("client.keep_alive_interval", rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
        param_desc.description = "maximum number of inflight messages";
        declare_parameter("client.max_inflight", rclcpp::ParameterType::PARAMETER_INTEGER, param_desc);
        declare_parameter("client.ip", rclcpp::ParameterType::PARAMETER_STRING, param_desc);

        declare_parameter("map_topic", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
        declare_parameter("odom_topic", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
        declare_parameter("orient_clientid", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
        declare_parameter("map_frame_id", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
        declare_parameter("robot_base_frame","base_footprint");    
        declare_parameter("transform_tolerance", 0.1);

        declare_parameter("frequency", 1.0);
        loadParameter("map_frame_id", map_frame_id_, "map");
        loadParameter("orient_clientid", orient_clientid_, rcpputils::get_env_var("ORIENT_CLIENTID"));
        loadParameter("robot_base_frame", robot_base_frame_);

        transform_tolerance_ = get_parameter("transform_tolerance").as_double();
        if (orient_clientid_.empty()) {
            throw std::runtime_error("ORIENT_CLIENTID environment variable not set");
        }
        // load broker parameters from parameter server
        loadParameter("broker.host", broker_config_.host, "localhost");
        loadParameter("broker.port", broker_config_.port, 1883);
        if (loadParameter("broker.user", broker_config_.user)) {
            loadParameter("broker.pass", broker_config_.pass, "");
        }

        // load client parameters from parameter server
        std::string client_buffer_directory;

        if (loadParameter("client.last_will.topic", client_config_.last_will.topic))
        {
            loadParameter("client.last_will.message", client_config_.last_will.message,
                          "offline");
            loadParameter("client.last_will.qos", client_config_.last_will.qos, 0);
            loadParameter("client.last_will.retained",
                          client_config_.last_will.retained, false);
        }
        loadParameter("client.clean_session", client_config_.clean_session, true);
        loadParameter("client.keep_alive_interval", client_config_.keep_alive_interval, 60.0);
        loadParameter("client.max_inflight", client_config_.max_inflight, 65535);
        loadParameter("client.ip", client_config_.ip, "localhost");

        loadParameter("map_topic", map_topic_, "map");
        loadParameter("odom_topic", odom_topic_, "odometry/filtered");

        // initialize MQTT client
        setupClient();
        setupSubscription();

        // connect to MQTT broker
        connect();
    }

protected:
  rclcpp::TimerBase::SharedPtr current_pose_timer_;
  double frequency_ = 1.0;
  std::string robot_base_frame_;       ///< The global frame for the costmap

  /// @brief Output cmd_vel publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;

  double transform_tolerance_;
    void setupSubscription() {

        callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);

        tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_->setUsingDedicatedThread(true);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

        state_publisher_ = create_publisher<orient_interfaces::msg::AgentState>("agent/state", rclcpp::ServicesQoS());

        load_reflector_map_client_ = create_client<LoadReflectorMapClientT>(
            "load_reflector_map",
            rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);

        load_checkpoint_client_ = create_client<LoadCheckPointClientT>(
            "load_checkpoint",
            rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;
        reflector_map_subscription_ = create_subscription<orient_interfaces::msg::ReflectorMap>("reflector/map",
            qos,
            std::bind(&Mqtt::reflector_map_callback, this, _1));

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, qos,
            std::bind(&Mqtt::map_received, this, std::placeholders::_1),sub_opt);    

        set_task_client_ = create_client<orient_interfaces::srv::SetTask>(
            "set_task",
            rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);

        task_sub_ = create_subscription<orient_interfaces::msg::TaskReport>(
            "govern/task/report", rclcpp::ServicesQoS(),
            std::bind(&Mqtt::govern_task_report, this, std::placeholders::_1),sub_opt);

        check_point_state_sub_ = create_subscription<orient_interfaces::msg::CheckPointStateMap>(
            "checkpoint/state/apply", rclcpp::ServicesQoS(),
            std::bind(&Mqtt::checkpoint_state_apply, this, std::placeholders::_1));
        checkpoint_state_publisher_ = create_publisher<orient_interfaces::msg::CheckPointStateMap>("checkpoint/state/update", rclcpp::ServicesQoS());
    
        frequency_ = get_parameter("frequency").as_double();
        current_pose_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(frequency_ * 1000)),
            [this]() {
                publish_robot_state();
            },callback_group_);
        current_checkpoint_sub_ = create_subscription<orient_interfaces::msg::CheckPointStateStamp>(
            "checkpoint/current", rclcpp::ServicesQoS(),
            [this](const orient_interfaces::msg::CheckPointStateStamp::SharedPtr check_point_msg){
               current_check_point_ = check_point_msg;
            },sub_opt);

        callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        callback_group_executor_->add_callback_group(callback_group_, get_node_base_interface());
        executor_thread_ = std::make_unique<nav2_util::NodeThread>(callback_group_executor_);
    }

    void checkpoint_state_apply(const orient_interfaces::msg::CheckPointStateMap::SharedPtr checkpoint) {
        YAML::Emitter e;
        e << YAML::BeginSeq;
        for (const auto &cp : checkpoint->checkpoints) {
            e << YAML::BeginMap;
            e << YAML::Key << "clientid" << YAML::Value << cp.clientid;
            e << YAML::Key << "point" << YAML::Value;
                e << YAML::BeginMap;
                e << YAML::Key << "x" << YAML::Value << cp.point.x;
                e << YAML::Key << "y" << YAML::Value << cp.point.y;
                e << YAML::Key << "z" << YAML::Value << cp.point.z;
                e << YAML::EndMap;
            e << YAML::Key << "position" << YAML::Value;
                e << YAML::BeginMap;
                e << YAML::Key << "x" << YAML::Value << cp.position.x;
                e << YAML::Key << "y" << YAML::Value << cp.position.y;
                e << YAML::Key << "z" << YAML::Value << cp.position.z;
                e << YAML::EndMap;
            e << YAML::Key << "state" << YAML::Value << cp.state;
            e << YAML::EndMap;
        }
        e << YAML::EndSeq;
        publish_mqtt_message("AGV/ApplyCheckPointState", e.c_str());
    }

    void message_arrived_apply_checkpoint_state(const std::string &str_msg) {
        orient_interfaces::msg::CheckPointStateMap checkpoint_map;
        checkpoint_map.header.stamp = this->now();
        checkpoint_map.header.frame_id = "map";
        YAML::Node checkpoint_nodes = YAML::Load(str_msg);
        for (const auto& checkpoint_node : checkpoint_nodes) {
            orient_interfaces::msg::CheckPointState checkpoint_state;
            checkpoint_state.clientid = checkpoint_node["clientid"].as<std::string>();
            checkpoint_state.point.x = checkpoint_node["point"]["x"].as<int>();
            checkpoint_state.point.y = checkpoint_node["point"]["y"].as<int>();
            checkpoint_state.point.z = checkpoint_node["point"]["z"].as<int>();
            checkpoint_state.position.x = checkpoint_node["position"]["x"].as<double>();
            checkpoint_state.position.y = checkpoint_node["position"]["y"].as<double>();
            checkpoint_state.position.z = checkpoint_node["position"]["z"].as<double>();
            checkpoint_state.state = checkpoint_node["state"].as<bool>();
            checkpoint_map.checkpoints.push_back(checkpoint_state);
        }
        checkpoint_state_publisher_->publish(checkpoint_map);
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    void map_received(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_msg_ = msg;
        // Convert the occupancy grid data to PGM format
        std::stringstream pgm_buffer;

        // Write the PGM header
        pgm_buffer 
        << "P5\n" 
            << msg->info.width << " " 
            << msg->info.height << " " 
            << msg->info.resolution << " " 
            << msg->info.origin.position.x << " " 
            << msg->info.origin.position.y
        << "\n255\n";

        // Convert the occupancy grid data to PGM pixel values
        // 0 = occupied, 100 = free, -1 = unknown
        int free_thresh_int = std::rint(0.25 * 100.0);
        int occupied_thresh_int = std::rint(0.65 * 100.0);
        std::vector<uint8_t> pixels(msg->info.width * msg->info.height);

        for (size_t y = 0; y < msg->info.height; y++) {
            for (size_t x = 0; x < msg->info.width; x++) {
                int8_t map_cell = msg->data[msg->info.width * (msg->info.height - y - 1) + x];
                uint8_t pixel = 205;
                if (map_cell < 0 || 100 < map_cell) {
                    pixel = 205;
                } else if (map_cell <= free_thresh_int) {
                    pixel =254;
                } else if (occupied_thresh_int <= map_cell) {
                    pixel = 0;
                }
                pixels[x + y * msg->info.width] = pixel;
            }
        }
        pgm_buffer.write(reinterpret_cast<const char *>(pixels.data()), pixels.size());
        // Publish the PGM data to MQTT
        publish_mqtt_message("AGV/Map", pgm_buffer.str(), 0);
    }
    
    bool  getRobotPose(geometry_msgs::msg::Pose & pose) {
        std::string tf_error;
        bool found = tf_->canTransform(robot_base_frame_, map_frame_id_, tf2::TimePointZero, &tf_error);
        if (!found) {
            return false;
        }
        geometry_msgs::msg::PoseStamped current_pose;
        if (!nav2_util::getCurrentPose(current_pose, *tf_, map_frame_id_, robot_base_frame_,transform_tolerance_)) {
            return false;
        }
        pose = current_pose.pose;
        return true;
    }

    void publish_agv_pose(const geometry_msgs::msg::Pose &rotbot_pose) {
        YAML::Emitter e;
        e << YAML::BeginMap;
            e << YAML::Key << "clientid" << YAML::Value << client_config_.ip;
            e << YAML::Key << "position" << YAML::Value;
                e << YAML::BeginMap;
                e << YAML::Key << "x" << YAML::Value << rotbot_pose.position.x;
                e << YAML::Key << "y" << YAML::Value << rotbot_pose.position.y;
                e << YAML::Key << "z" << YAML::Value << rotbot_pose.position.z;
                e << YAML::EndMap;
            e << YAML::Key << "orientation" << YAML::Value;
                e << YAML::BeginMap;
                e << YAML::Key << "x" << YAML::Value << rotbot_pose.orientation.x;
                e << YAML::Key << "y" << YAML::Value << rotbot_pose.orientation.y;
                e << YAML::Key << "z" << YAML::Value << rotbot_pose.orientation.z;
                e << YAML::Key << "w" << YAML::Value << rotbot_pose.orientation.w;
                e << YAML::EndMap;
        e << YAML::EndMap;
        publish_mqtt_message("AGV/Pose", e.c_str());
    }


    std::size_t sequence_ = 0;
    rmf_fleet_msgs::msg::RobotMode current_mode_;

    template<typename T>
    std::string serialize_message(const T &msg) {
        rclcpp::Serialization<T> serializer;
        rclcpp::SerializedMessage serialized_message;
        serializer.serialize_message(&msg, &serialized_message);
        auto sm = serialized_message.get_rcl_serialized_message();
        return std::string(sm.buffer, sm.buffer + sm.buffer_length);
    }

    template<typename T>
    T deserialize_message(const std::string &str_msg) {
        uint32_t payload_length = static_cast<uint32_t>(str_msg.size());
        rclcpp::SerializedMessage serialized_msg(payload_length);
        std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, &(str_msg[0]), payload_length);
        serialized_msg.get_rcl_serialized_message().buffer_length = payload_length;
        T msg;
        rclcpp::Serialization<T> serializer;
        serializer.deserialize_message(&serialized_msg, &msg);
        return msg;
    }

    orient_interfaces::msg::Task current_task_request_;

    void publish_robot_state() {

        geometry_msgs::msg::Pose rotbot_pose;
        if (map_msg_ == nullptr || !getRobotPose(rotbot_pose)) {
            return;
        }
        publish_agv_pose(rotbot_pose);

        rmf_fleet_msgs::msg::RobotState robot_state_msg;
        robot_state_msg.name =  orient_clientid_;
        robot_state_msg.location.x = static_cast<unsigned int>((rotbot_pose.position.x - map_msg_->info.origin.position.x) / map_msg_->info.resolution);
        auto my = static_cast<unsigned int>((rotbot_pose.position.y - map_msg_->info.origin.position.y) / map_msg_->info.resolution);
        robot_state_msg.location.y = orient_checkpoint::invert_y_axis(map_msg_->info.height -  my - 1);
        robot_state_msg.location.yaw = tf2::getYaw(rotbot_pose.orientation);
        robot_state_msg.battery_percent = 50.0; // Placeholder for battery percentage
        robot_state_msg.model = "AGV";
        robot_state_msg.task_id = current_task_request_.path_request.task_id; // Placeholder for task ID
        robot_state_msg.seq = ++sequence_; // Placeholder for sequence number
        robot_state_msg.mode = current_mode_; // Placeholder for mode
        publish_mqtt_message("robot_state", serialize_message(robot_state_msg), 1);
    }
    
    void message_arrived_robot_task_requests(const std::string &str_msg) {
        set_task_client_->wait_for_service(std::chrono::seconds(1));
        if (!set_task_client_->service_is_ready()) {
            RCLCPP_ERROR(get_logger(), "SetTask Service not available after waiting");
            return;
        }

        orient_interfaces::msg::Task task_request = deserialize_message<orient_interfaces::msg::Task>(str_msg);
        if (task_request.clientid != orient_clientid_) {
            RCLCPP_WARN(get_logger(), "TaskRequest for robot %s does not match this robot", task_request.clientid.c_str());
            return;
        }
        for(rmf_fleet_msgs::msg::Location &loc : task_request.path_request.path) {
            loc.y = static_cast<uint32_t>(-loc.y);
        }
        current_task_request_ = task_request;

        auto request = std::make_shared<orient_interfaces::srv::SetTask::Request>();
        request->task = task_request;
        set_task_client_->async_send_request(request,
            [this](rclcpp::Client<orient_interfaces::srv::SetTask>::SharedFuture future) {
                current_mode_.mode  = future.get()->result?rmf_fleet_msgs::msg::RobotMode::MODE_MOVING:rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
            });
    }

    void govern_task_report(const orient_interfaces::msg::TaskReport::SharedPtr task_report) {
        RCLCPP_INFO(get_logger(), "Received TaskReport: %s", task_report->task_id.c_str());
        current_mode_.mode = rmf_fleet_msgs::msg::RobotMode::MODE_IDLE;
        rmf_fleet_msgs::msg::ModeRequest mode_request;
        mode_request.robot_name = current_task_request_.path_request.robot_name;
        mode_request.fleet_name = current_task_request_.path_request.fleet_name;
        mode_request.task_id = current_task_request_.path_request.task_id;
        mode_request.mode = current_mode_;
        publish_mqtt_message("action_execution_notice", serialize_message(mode_request), 0);
    }

    void reflector_map_callback(const orient_interfaces::msg::ReflectorMap::SharedPtr reflector_grid)
    {
        using namespace orient_reflector;
        std::stringstream ss;
        ss<< *reflector_grid;
        std::string str_msg = ss.str();
        publish_mqtt_message("AGV/ReflectorMap", str_msg, 0);
    }


    void message_arrived_checkpoint_map(const std::string &str_msg) {
        RCLCPP_INFO(get_logger(), "Received CheckPointMap message: %s", str_msg.c_str());
        auto request = std::make_shared<LoadCheckPointClientT::Request>();
        request->content = str_msg;
        if (!load_checkpoint_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(get_logger(), "CheckPointMap Service not available after waiting");
            return;
        }
        load_checkpoint_client_->async_send_request(request);
    }

    void message_arrived_reflector_map(const std::string &str_msg) {
        RCLCPP_INFO(get_logger(), "Received ReflectorMap message: %s", str_msg.c_str());
        using namespace orient_reflector;
        std::stringstream ss(str_msg);
        auto request = std::make_shared<LoadReflectorMapClientT::Request>();
        ss >> request->map;
        if (!load_reflector_map_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(get_logger(), "ReflectorMap Service not available after waiting");
            return;
        }
        load_reflector_map_client_->async_send_request(request);
    }


    void message_arrived_start_navigation(const std::string &str_msg) {
        YAML::Node checkpoint_node = YAML::Load(str_msg);
        if (checkpoint_node["IP"].as<std::string>() != client_config_.ip) {
            return;
        }
        RCLCPP_INFO(get_logger(), "Received StartNavigation message: \r\n %s", str_msg.c_str());

        // orient_interfaces::msg::Task task;
        // task.header.stamp = now();
        // task.header.frame_id = "map";
        // task.task_id = "task1";
        //         {
        //     orient_interfaces::msg::Action action;
        //     action.jobid = "job1";
        //     action.behavior = "follow_path.xml";
        //     {
        //         orient_interfaces::msg::CheckPoint cp;
        //         cp.x = 463;
        //         cp.y = 383;
        //         cp.z = 0;
        //         action.points.push_back(cp);
        //     }
        //     {
        //         orient_interfaces::msg::CheckPoint cp;
        //         cp.x = 377;
        //         cp.y = 127;
        //         cp.z = 0;
        //         action.points.push_back(cp);
        //     }
        //     // {
        //     //     orient_interfaces::msg::CheckPoint cp;
        
        //     //     cp.x = 289;
        //     //     cp.y = 169;
        //     //     cp.z = 0;
        //     //     action.points.push_back(cp);
            
        //     // }
        //     task.actions.push_back(action);
        //  }
         
        // //        {
        // //     orient_interfaces::msg::Action action;
        // //     action.jobid = "job2";
        // //     action.behavior = "backup_and_forklift_up.xml";
    
        // //     {

        // //         orient_interfaces::msg::CheckPoint cp;
        // //        cp.x = 289;
        // //         cp.y = 169;
        // //         cp.z = 0;
        // //         action.points.push_back(cp);
            
        // //     }
        // //     {
        // //         orient_interfaces::msg::CheckPoint cp;
    
        // //         cp.x = 381;
        // //         cp.y = 174;
        // //         cp.z = 0;
        // //         action.points.push_back(cp);
            
        // //     }
        // //     task.actions.push_back(action);
        // //  }
 
        // task_publisher_->publish(task);

  
        // // just for test
        // task_ = task;
    }

    std::filesystem::path resolvePath(const std::string &path_string)
    {
        std::filesystem::path path(path_string);
        if (path_string.empty())
            return path;
        if (!path.has_root_path())
        {
            std::string ros_home = rcpputils::get_env_var("ROS_HOME");
            if (ros_home.empty())
                ros_home = std::string(std::filesystem::current_path());
            path = std::filesystem::path(ros_home);
            path.append(path_string);
        }
        if (!std::filesystem::exists(path))
            RCLCPP_WARN(get_logger(), "Requested path '%s' does not exist",
                        std::string(path).c_str());
        return path;
    }

    void setupClient()
    {
        // basic client connection options
        connect_options_.set_automatic_reconnect(true);
        connect_options_.set_clean_session(client_config_.clean_session);
        connect_options_.set_keep_alive_interval(client_config_.keep_alive_interval);
        connect_options_.set_max_inflight(client_config_.max_inflight);

        // user authentication
        if (!broker_config_.user.empty())
        {
            connect_options_.set_user_name(broker_config_.user);
            connect_options_.set_password(broker_config_.pass);
        }

        // last will
        if (!client_config_.last_will.topic.empty())
        {
            mqtt::will_options will(
                client_config_.last_will.topic, client_config_.last_will.message,
                client_config_.last_will.qos, client_config_.last_will.retained);
            connect_options_.set_will(will);
        }

        // create MQTT client
        const std::string protocol = "tcp";
        const std::string uri = protocol + "://" + broker_config_.host + ":" + std::to_string(broker_config_.port);
        try
        {
            client_ = std::shared_ptr<mqtt::async_client>(new mqtt::async_client(uri, orient_clientid_, 1000));
        }
        catch (const mqtt::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Client could not be initialized: %s", e.what());
            exit(EXIT_FAILURE);
        }

        // setup MQTT callbacks
        client_->set_callback(*this);
    }

    void connect()
    {
        std::string as_client =std::string(" as '") + orient_clientid_ + std::string("'");
        RCLCPP_INFO(get_logger(), "Connecting to broker at '%s'%s ...", client_->get_server_uri().c_str(), as_client.c_str());
        try
        {
            client_->connect(connect_options_, nullptr, *this);
        }
        catch (const mqtt::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Connection to broker failed: %s", e.what());
            exit(EXIT_FAILURE);
        }
    }

    void publish_state(bool state) {

        orient_interfaces::msg::AgentState msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.state = !state;
        state_publisher_->publish(msg);
    }


    void message_arrived(mqtt::const_message_ptr mqtt_msg)
    {
        // instantly take arrival timestamp
        rclcpp::Time arrival_stamp(builtin_interfaces::msg::Time(rclcpp::Clock(RCL_SYSTEM_TIME).now()));

        std::string mqtt_topic = mqtt_msg->get_topic();
        const std::string str_msg = mqtt_msg->to_string();
        if (mqtt_topic == "AGV/CheckPointMap") {
            message_arrived_checkpoint_map(str_msg);
        } else if (mqtt_topic == "AGV/LoadReflectorMap") {
            message_arrived_reflector_map(str_msg);
        }  else if (mqtt_topic == "AGV/StartNavigation") {
            message_arrived_start_navigation(str_msg);
        }  else if (mqtt_topic == "AGV/ApplyCheckPointState") {
            message_arrived_apply_checkpoint_state(str_msg);
        } else if (mqtt_topic == "robot_task_requests") {
            message_arrived_robot_task_requests(str_msg);
        }
    }
    void connected(const std::string &cause)
    {
        (void)cause; // Avoid compiler warning for unused parameter.

        is_connected_ = true;
        std::string as_client = std::string(" as '") +  orient_clientid_ + std::string("'");
        RCLCPP_INFO(get_logger(), 
            "Connected to broker at '%s'%s",
            client_->get_server_uri().c_str(), as_client.c_str());
        static const char* mqtt_subscribes[] = {
            "AGV/CheckPointMap",
            "AGV/LoadReflectorMap",
            "AGV/StartNavigation",
            "AGV/ApplyCheckPointState",
            "robot_task_requests",
        };
        for(const auto& mqtt_topic : mqtt_subscribes) {
            client_->subscribe(mqtt_topic, 0);
        }
        publish_state(is_connected_);
    }

    void connection_lost(const std::string &cause)
    {
        (void)cause; // Avoid compiler warning for unused parameter.
        RCLCPP_ERROR(get_logger(), "Connection to broker lost, will try to reconnect...");
        is_connected_ = false;
        connect();
        publish_state(is_connected_);
    }
    
    template <typename T>
    bool loadParameter(const std::string &key, T &value,
                       const T &default_value)
    {
        bool found = get_parameter_or(key, value, default_value);
        if (!found)
            RCLCPP_WARN(get_logger(), "Parameter '%s' not set, defaulting to '%s'",
                        key.c_str(), std::to_string(default_value).c_str());
        if (found)
            RCLCPP_DEBUG(get_logger(), "Retrieved parameter '%s' = '%s'", key.c_str(),
                         std::to_string(value).c_str());
        return found;
    }

    bool loadParameter(const std::string &key, std::string &value)
    {
        bool found = get_parameter(key, value);
        if (found)
            RCLCPP_DEBUG(get_logger(), "Retrieved parameter '%s' = '%s'", key.c_str(),
                         value.c_str());
        return found;
    }

    bool loadParameter(const std::string &key, std::string &value,
                       const std::string &default_value)
    {
        bool found = get_parameter_or(key, value, default_value);
        if (!found)
            RCLCPP_WARN(get_logger(), "Parameter '%s' not set, defaulting to '%s'",
                        key.c_str(), default_value.c_str());
        if (found)
            RCLCPP_DEBUG(get_logger(), "Retrieved parameter '%s' = '%s'", key.c_str(),
                         value.c_str());
        return found;
    }

    void publish_mqtt_message(const std::string &topic, const std::string &payload,int qos = 0)
    {
        if (!is_connected_)
        {
            RCLCPP_WARN(get_logger(), "Not connected to broker, cannot publish message");
            return;
        }
        try
        {
            mqtt::message_ptr mqtt_msg = mqtt::make_message(topic, payload,qos, false);
            client_->publish(mqtt_msg);
        }
        catch (const mqtt::exception &e)
        {
            RCLCPP_WARN(
                get_logger(),
                "Publishing ROS message type information to MQTT failed: %s",e.what());
            publish_state(false);
        }
    }

    void delivery_complete(mqtt::delivery_token_ptr token)
    {

        (void)token; // Avoid compiler warning for unused parameter.
    }

    void on_success(const mqtt::token &token)
    {

        (void)token; // Avoid compiler warning for unused parameter.
        is_connected_ = true;
        publish_state(is_connected_);
    }

    void on_failure(const mqtt::token &token)
    {

        RCLCPP_ERROR(
            get_logger(),
            "Connection to broker failed (return code %d), will automatically "
            "retry...",
            token.get_return_code());
        is_connected_ = false;
        publish_state(is_connected_);
    }

private:
    std::string orient_clientid_;
    /**
     * @brief Status variable keeping track of connection status to broker
     */
    bool is_connected_ = false;
    /**
     * @brief Broker parameters
     */
    BrokerConfig broker_config_;

    /**
     * @brief Client parameters
     */
    ClientConfig client_config_;
    /**
     * @brief MQTT client variable
     */
    std::shared_ptr<mqtt::async_client> client_;


    std::string map_frame_id_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    /**
     * @brief MQTT client connection options
     */
    mqtt::connect_options connect_options_;
    rclcpp::Publisher<orient_interfaces::msg::AgentState>::SharedPtr state_publisher_;

    rclcpp::Subscription<orient_interfaces::msg::ReflectorMap>::SharedPtr reflector_map_subscription_;
    rclcpp::Publisher<orient_interfaces::msg::CheckPointMap>::SharedPtr checkpoint_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_subscription_;
    rclcpp::Client<orient_interfaces::srv::SetTask>::SharedPtr set_task_client_;

    rclcpp::Subscription<orient_interfaces::msg::TaskReport>::SharedPtr task_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscription_;
    rclcpp::Subscription<orient_interfaces::msg::CheckPointStateStamp>::SharedPtr current_checkpoint_sub_;
    orient_interfaces::msg::CheckPointStateStamp::SharedPtr current_check_point_;

  // Input/output speed controls
  /// @beirf Input cmd_vel subscriber
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr map_sub_;

    rclcpp::Subscription<orient_interfaces::msg::CheckPointStateMap>::ConstSharedPtr check_point_state_sub_;
    rclcpp::Publisher<orient_interfaces::msg::CheckPointStateMap>::SharedPtr checkpoint_state_publisher_;

    std::string map_topic_;
    std::string odom_topic_;
    
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
    std::unique_ptr<nav2_util::NodeThread> executor_thread_;

    LoadCheckPointActionClient::SharedPtr load_checkpoint_client_;
    LoadReflectorMapActionClient::SharedPtr load_reflector_map_client_;
};

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(Mqtt)

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mqtt>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
