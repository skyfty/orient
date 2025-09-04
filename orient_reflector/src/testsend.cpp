#include <cstdio>
#include <string>
#include <fstream>
#include <optional>
#include <list>
#include <vector>
#include <boost/smart_ptr.hpp>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "orient_interfaces/msg/reflector_grid.hpp" // CHANGE
#include "orient_reflector/utils.hpp"
#include <laser_geometry/laser_geometry.hpp>
#include "sensor_msgs/msg/point_cloud.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <queue>


using namespace std::placeholders;
using namespace orient_interfaces::msg;
using namespace orient_reflector;


class TestSend : public nav2_util::LifecycleNode
{
public:
    TestSend(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : nav2_util::LifecycleNode("orient_recognize_testsend_node", "", options)
    {
        RCLCPP_INFO(get_logger(), "Creating");
        declare_parameter("publish_period_sec",100);
    }

protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
  
        get_parameter("publish_period_sec", publish_period_sec_);

        pub_reflector_grid_ = create_publisher<ReflectorGrid>("reflector/recognize", rclcpp::SensorDataQoS());

        cleanup_timer_ = create_wall_timer(std::chrono::milliseconds(publish_period_sec_), [this]() {
            send_callback();
        });
        return nav2_util::CallbackReturn::SUCCESS;
    }

            
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(get_logger(), "Activating");
        pub_reflector_grid_->on_activate();
        // create bond connection

        // Read reflector data from reflector.txt
        std::ifstream reflector_file("/home/sky/agv_vehicle/reflector.txt");
        if (!reflector_file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open reflector.txt");
            return nav2_util::CallbackReturn::FAILURE;
        }

        std::string line;
        while (std::getline(reflector_file, line)) {
            Reflector reflector;
            std::istringstream iss(line);
            iss >> reflector.point.x >> reflector.point.y >> reflector.point.z >> reflector.intensity >> reflector.diameter;
            reflectors_.push_back(reflector);
        }

        reflector_file.close();
        RCLCPP_INFO(get_logger(), "Loaded %zu reflectors from reflector.txt", reflectors_.size());

        // 绑定回调函数
        createBond();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(get_logger(), "Deactivating");
        pub_reflector_grid_->on_deactivate();
        // destroy bond connection
        destroyBond();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)  {
        RCLCPP_INFO(get_logger(), "Cleaning up");
        pub_reflector_grid_.reset();
        cleanup_timer_.reset();
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)  {
        RCLCPP_INFO(get_logger(), "Shutting down");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    std::deque<Reflector> serialized_;


    void send_callback() {

        if (reflectors_.size() < 3) {
            RCLCPP_WARN(get_logger(), "No reflectors to send.");
            return;
        }
    
        std::vector<Reflector> reflectors;
        for(int i = 0; i < 5; ++i) {
                // 创建点云消息
            if (index_ >= reflectors_.size()) {
                index_ = 0; // 重置索引
            }
            reflectors.push_back(reflectors_[index_]);
    
            index_ += 1;
        }

  
        
        // 反射柱轨迹更新
        ReflectorGrid reflector_grid;
        reflector_grid.header.stamp = this->now();
        reflector_grid.header.frame_id = "scanner_link";
        reflector_grid.reflectors = reflectors;
        pub_reflector_grid_->publish(reflector_grid);

    
        RCLCPP_INFO(get_logger(), "Sending %zu reflectors.", reflector_grid.reflectors.size());
        
    }



private:
    int publish_period_sec_;
    size_t index_ = 0;
    std::vector<Reflector> reflectors_;
    rclcpp::TimerBase::SharedPtr cleanup_timer_;
    rclcpp_lifecycle::LifecyclePublisher<ReflectorGrid>::SharedPtr pub_reflector_grid_;

};

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(TestSend)

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestSend>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
