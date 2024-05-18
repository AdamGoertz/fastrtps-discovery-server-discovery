#pragma once

#include <discovery_interfaces/msg/server_info.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <rclcpp/node.hpp>
#include <filesystem>

class DiscoveryServerListener : public rclcpp::Node {
  using ServerInfo = discovery_interfaces::msg::ServerInfo;

public:
  DiscoveryServerListener();

  void discovery_info_callback(ServerInfo::ConstSharedPtr);

  void update_server_info(const ServerInfo::ConstSharedPtr& info);

  void configure_auxiliary_server(const ServerInfo::ConstSharedPtr& info);

private:
  rclcpp::Subscription<ServerInfo>::SharedPtr discovery_info_sub_;
  rclcpp::Publisher<ServerInfo>::SharedPtr discovery_info_pub_;
  eprosima::fastdds::dds::DomainParticipantQos server_qos_;
  std::unique_ptr<eprosima::fastdds::dds::DomainParticipant> aux_server_;
  std::unordered_map<uint8_t, ServerInfo::ConstSharedPtr> server_info_;
};
