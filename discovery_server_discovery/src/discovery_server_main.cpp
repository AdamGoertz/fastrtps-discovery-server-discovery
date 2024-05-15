#include <rclcpp/rclcpp.hpp>

#include <discovery_server_discovery/listener.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto listener = std::make_shared<DiscoveryServerListener>();
  rclcpp::spin(listener);
  rclcpp::shutdown();

  return 0;
}
