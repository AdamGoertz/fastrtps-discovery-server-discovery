#pragma once

#include <discovery_interfaces/msg/server_info.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <rclcpp/node.hpp>
#include <filesystem>

// Convert an integer to the eProsima GUID prefix
// 44.53.<server-id-in-hex>.5f.45.50.52.4f.53.49.4d.41
// From the docs:
// This prefix schema has been chosen for its ASCII translation:
// DS<id_in_hex>_EPROSIMA
std::string id_to_guid_prefix(uint8_t id);

class DiscoveryServerListener : public rclcpp::Node {
  using ServerInfo = discovery_interfaces::msg::ServerInfo;

public:
  DiscoveryServerListener();

  void discovery_info_callback(ServerInfo::ConstSharedPtr);

  void update_server_info(const ServerInfo::ConstSharedPtr& info);

  void write_environment_file() const;

  std::string format_server_info() const;

private:
  rclcpp::Subscription<ServerInfo>::SharedPtr discovery_info_sub_;
  rclcpp::Publisher<ServerInfo>::SharedPtr discovery_info_pub_;
  eprosima::fastdds::dds::DomainParticipantQos aux_server_;
  std::filesystem::path fastdds_environment_file_;
  std::vector<ServerInfo::ConstSharedPtr> server_info_;
};
