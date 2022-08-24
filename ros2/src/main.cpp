#include "sensor_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<T10Sensor>());
  rclcpp::shutdown();
  return 0;
}
