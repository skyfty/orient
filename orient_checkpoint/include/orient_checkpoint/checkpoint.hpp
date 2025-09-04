
#ifndef ORIENT_CHECKPOINT_SERVER_HPP_
#define ORIENT_CHECKPOINT_SERVER_HPP_
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/parameter_value.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_navfn_planner/navfn.hpp"
#include "orient_interfaces/srv/check_point_pose.hpp" // CHANGE
#include "orient_interfaces/msg/check_point_map.hpp"             // CHANGE
#include "orient_interfaces/srv/load_check_point.hpp"             // CHANGE
#include "orient_interfaces/srv/get_check_point_map.hpp"             // CHANGE
#include "orient_interfaces/msg/check_point_state_map.hpp"
#include "orient_interfaces/msg/check_point_state.hpp"             // CHANGE
#include "orient_interfaces/msg/check_point_state_stamp.hpp"             // CHANGE

#include "orient_checkpoint/utils.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"


namespace orient_checkpoint {

class CheckpointServer : public nav2_util::LifecycleNode {
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, orient_interfaces::msg::CheckPoint, boost::property<boost::edge_weight_t, size_t>> Graph;
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
  typedef boost::graph_traits<Graph>::edge_descriptor Edge;

  using GlobalCheckPointMap = std::unordered_map<orient_interfaces::msg::CheckPoint, orient_interfaces::msg::CheckPointState, orient_checkpoint::PointHasher, orient_checkpoint::PointEqual>;
  using CheckPointVertexMap = std::unordered_map<orient_interfaces::msg::CheckPoint, Vertex, orient_checkpoint::PointHasher, orient_checkpoint::PointEqual>;

  using ActionThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using ActionServerThroughPoses = nav2_util::SimpleActionServer<ActionThroughPoses>;

public:
  explicit CheckpointServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CheckpointServer() = default;

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;


  void loadCheckPointCallback(
      const std::shared_ptr<rmw_request_id_t>/*request_header*/,
      const std::shared_ptr<orient_interfaces::srv::LoadCheckPoint::Request> request,
      std::shared_ptr<orient_interfaces::srv::LoadCheckPoint::Response> response);

  void get_checkpoint_map_callback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<orient_interfaces::srv::GetCheckPointMap::Request> request,
    std::shared_ptr<orient_interfaces::srv::GetCheckPointMap::Response> response);

  void appraise_checkpoint(
      const std::shared_ptr<rmw_request_id_t>/*request_header*/,
      const std::shared_ptr<orient_interfaces::srv::CheckPointPose::Request> request,
      std::shared_ptr<orient_interfaces::srv::CheckPointPose::Response> response);

  std::vector<orient_interfaces::msg::CheckPointState> 
  get_checkpoints_track(const std::vector<orient_interfaces::msg::CheckPoint> & in_points);

  void checkpoint_state_update(const orient_interfaces::msg::CheckPointStateMap::SharedPtr check_point);
  void set_global_checkpoints();
  void setGraph();
  void resetCheckPoint();

private:
  std::optional<CheckpointServer::Vertex> getVertex(const orient_interfaces::msg::CheckPoint & point);
  bool loadCheckPointFromYaml(const std::string & yaml_filename);
  bool loadMapFromYaml(const std::string & yaml_filename);
  void publish_checkpoint_map();
  void publish_checkpoint_path();
  int getCheckPointPath(const orient_interfaces::msg::CheckPoint &start,const orient_interfaces::msg::CheckPoint &end);

  void mapToWorld(uint32_t mx, uint32_t my, double & wx, double & wy) const;
  void worldToMap( double wx, double wy,uint32_t  & mx, uint32_t &  my) const;

  void publish_current_checkpoint(const orient_interfaces::msg::CheckPointState &current_check_point);
  void check_current_checkpoint();

  std::optional<orient_interfaces::msg::CheckPoint> location_checkpoint(const geometry_msgs::msg::Point &point);

  template<typename T>
  void convertPoint(const T &in_point,geometry_msgs::msg::Point &point) const{
      int ny = checkpoint_map_.info.height - in_point.y - 1;
      mapToWorld(in_point.x, ny, point.x, point.y);
  }

  void convertPoint(geometry_msgs::msg::Point &in_point,orient_interfaces::msg::CheckPoint &point) const{
      worldToMap(in_point.x, in_point.y, point.x, point.y);
      point.y = checkpoint_map_.info.height - point.y - 1;
  }
  /**
   * @brief The action server callback which calls planner to get the path
   * ComputePathThroughPoses
   */
  void computePlanThroughPoses();
    /**
   * @brief Check if an action server is valid / active
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isServerInactive(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a cancellation request pending
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isCancelRequested(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishPlan(const nav_msgs::msg::Path & path);
    /**
   * @brief Get the starting pose from costmap or message, if valid
   * @param action_server Action server to terminate if required
   * @param goal Goal to find start from
   * @param start The starting pose to use
   * @return bool If successful in finding a valid starting pose
   */
  template<typename T>
  bool getStartPose(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal,
    geometry_msgs::msg::PoseStamped & start);

  bool  getRobotPose(geometry_msgs::msg::PoseStamped & start);

  /**
   * @brief Check if an action server has a preemption request and replaces the goal
   * with the new preemption goal.
   * @param action_server Action server to get updated goal if required
   * @param goal Goal to overwrite
   */
  template<typename T>
  void getPreemptedGoalIfRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal);

  std::vector<geometry_msgs::msg::Point> getPlanPoints(
    const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & goal);

  bool get_checkpoint(const geometry_msgs::msg::Point & point, orient_interfaces::msg::CheckPoint & checkpoint);
  bool get_checkpoint(const rmf_fleet_msgs::msg::Location & point, orient_interfaces::msg::CheckPoint & checkpoint);
  std::vector<geometry_msgs::msg::Pose> make_path(
    const geometry_msgs::msg::PoseStamped &start,std::vector<geometry_msgs::msg::PoseStamped> goals, bool backup = false);
  std::vector<geometry_msgs::msg::Pose> make_path(const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped & goal);

  double getPointPotential(const geometry_msgs::msg::Point & world_point);
  bool getPlanFromPotential( const geometry_msgs::msg::Pose & goal,
  std::vector<geometry_msgs::msg::Pose> & plan);
  bool smooth(nav_msgs::msg::Path & path,const std::vector<geometry_msgs::msg::PoseStamped> &goals,
    const rclcpp::Duration & max_time= rclcpp::Duration::from_seconds(1));
  bool smooth(nav_msgs::msg::Path & path,bool & reversing_segment);

  inline double getFieldByDim(const geometry_msgs::msg::PoseStamped & msg,const unsigned int & dim);

  inline void setFieldByDim(geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,const double & value);
  std::vector<int> getPathIndex(const geometry_msgs::msg::Point & point);
  int getNearestPathPoint(const geometry_msgs::msg::Point & point, const std::vector<orient_interfaces::msg::CheckPoint> & path_points);

protected:
  std::string orient_clientid_;
  std::string global_frame_;       ///< The global frame for the costmap
  std::string robot_base_frame_;       ///< The global frame for the costmap
  float match_distance_thresh_ = 0.7f; // 匹配距离阈值(m)
  bool allow_reversing_ = false;

  double frequency_ = 1.0;
  rclcpp::TimerBase::SharedPtr current_checkpoint_timer_;
  double transform_tolerance_;
  double tolerance_, data_w_, smooth_w_;
  int max_its_, refinement_ctr_;
  bool do_refinement_;
  int refinement_num_;
  double interpolation_resolution_;

  rclcpp::Service<orient_interfaces::srv::LoadCheckPoint>::SharedPtr load_checkpoint_service_;
  rclcpp::Service<orient_interfaces::srv::CheckPointPose>::SharedPtr appraise_checkpoint_;
  rclcpp::Service<orient_interfaces::srv::GetCheckPointMap>::SharedPtr get_map_service_;
  rclcpp::Subscription<orient_interfaces::msg::CheckPointStateMap>::SharedPtr checkpoint_state_update_sub_;

  rclcpp_lifecycle::LifecyclePublisher<orient_interfaces::msg::CheckPointStateMap>::SharedPtr checkpoint_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<orient_interfaces::msg::CheckPointStateStamp>::SharedPtr current_checkpoint_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr current_position_publisher_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  orient_interfaces::msg::CheckPointMap checkpoint_map_;
  GlobalCheckPointMap global_checkpoints_map_;

  // Planner based on ROS1 NavFn algorithm

  std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::unique_ptr<nav2_navfn_planner::NavFn> planner_;

  Graph graph_;
  CheckPointVertexMap vertex_map_;
  orient_interfaces::msg::CheckPoint current_check_point_; 
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud>::SharedPtr global_plan_publisher_;
  std::unique_ptr<ActionServerThroughPoses> action_server_poses_;
  std::atomic<bool> active_{false};

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<nav2_util::NodeThread> executor_thread_;
};
}
#endif
