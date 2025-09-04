
#include "rclcpp/rclcpp.hpp"
#include "orient_reflector/localization.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto node = std::make_shared<orient_reflector::Localization>(options);
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
