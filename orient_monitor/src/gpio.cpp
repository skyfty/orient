#include "orient_monitor/gpio.hpp"

namespace orient_monitor
{
   static  std::vector<int64_t> default_gpio_pins = {880, 890, 882};

Gpio::Gpio(const nav2_util::LifecycleNode::WeakPtr & parent_node)  {
    auto node = parent_node.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    node->declare_parameter("gpio.pins", default_gpio_pins);
    std::vector<long> gpio_pins = node->get_parameter("gpio.pins").as_integer_array();

    RCLCPP_INFO(node->get_logger(), "Initializing GPIO pins");

    for (const auto & pin : gpio_pins) {
       std::ifstream pin_file;
       pin_file.open("/sys/class/gpio/gpio" + std::to_string(pin) + "/value", std::ios::in | std::ios::binary);
       if (pin_file.is_open()) {
        pins_[pin] = std::move(pin_file);
       }
    }  
}

Gpio::~Gpio() {
    for (auto & pin : pins_) {
        if (pin.second.is_open()) {
            pin.second.close();
        }
    }
}

bool Gpio::check(std::map<int, int> & pin_values) {
    for (auto & pin : pins_) {
        if (pin.second.is_open()) {
            pin.second.seekg(0, std::ios::beg);
            // Read the value from the GPIO pin file
            // Assuming the value is stored as an integer (0 or 1)
            // Adjust the reading logic if the format is different
            int value = 0;
            pin.second.read((char*)&value, sizeof(value));
            if (value == 1) {
                pin_values[pin.first] = value;
            }
        }
    }
    return !pin_values.empty();
}

bool Gpio::check_any() {
    std::map<int, int> pin_values;
    return check(pin_values);
}
}