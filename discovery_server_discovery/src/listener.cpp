#include <discovery_server_discovery/listener.hpp>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include <rclcpp/subscription_options.hpp>

#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <ostream>

using eprosima::fastdds::dds::DomainParticipant;
using eprosima::fastdds::dds::DomainParticipantFactory;
using eprosima::fastdds::dds::DomainParticipantQos;
using eprosima::fastdds::rtps::get_server_client_default_guidPrefix;
using eprosima::fastrtps::rtps::DiscoveryProtocol_t;
using eprosima::fastrtps::rtps::IPLocator;
using eprosima::fastrtps::rtps::Locator_t;
using eprosima::fastrtps::rtps::RemoteServerAttributes;

using std::placeholders::_1;

DiscoveryServerListener::DiscoveryServerListener()
    : rclcpp::Node("discovery_server_manager") {
  // TODO: Add descriptor limiting range to [0, 255]
  declare_parameter("discovery_server_id", rclcpp::PARAMETER_INTEGER);
  declare_parameter("discovery_server_address", rclcpp::PARAMETER_STRING);
  // TODO: Add descriptor limiting range to [0, 65535]
  declare_parameter("discovery_server_port", rclcpp::PARAMETER_INTEGER);

  // Set reliable to guarantee other servers recieve our communications.
  // Set transient local so late-joining subscribers recieve information
  // about our discovery server.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                       .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  // Ignore local publications -- we already have our own
  // discovery server info
  rclcpp::SubscriptionOptions sub_options;
  sub_options.ignore_local_publications = true;

  discovery_info_sub_ = create_subscription<ServerInfo>(
      "/discovery_info", qos,
      std::bind(&DiscoveryServerListener::discovery_info_callback, this, _1),
      sub_options);
  discovery_info_pub_ = create_publisher<ServerInfo>("/discovery_info", qos);

  auto this_server_info = std::make_shared<ServerInfo>();
  this_server_info->name = get_fully_qualified_name();
  this_server_info->id =
      static_cast<uint8_t>(get_parameter("discovery_server_id").as_int());
  this_server_info->address =
      get_parameter("discovery_server_address").as_string();
  this_server_info->port = get_parameter("discovery_server_port").as_int();

  configure_auxiliary_server(this_server_info);

  // Publish our info immediately, transient local durability policy
  // will re-send if new subscribers connect later.
  discovery_info_pub_->publish(*this_server_info);
}

void DiscoveryServerListener::discovery_info_callback(
    ServerInfo::ConstSharedPtr info) {
  RCLCPP_INFO_STREAM(get_logger(),
                     "Received discovery server info:\n"
                         << discovery_interfaces::msg::to_yaml(*info));
  update_server_info(info);
}

void DiscoveryServerListener::update_server_info(
    const ServerInfo::ConstSharedPtr &info) {
  auto &entry = server_info_[info->id];

  if (entry != nullptr) {
    if (*entry != *info) {
      RCLCPP_WARN_STREAM(get_logger(),
                         "Received duplicate ID for server:\n"
                             << discovery_interfaces::msg::to_yaml(*entry)
                             << "with different configuration:\n"
                             << discovery_interfaces::msg::to_yaml(*info));
    } else {
      RCLCPP_INFO(get_logger(), "Ignoring duplicate discovery server info");
    }
    return;
  }
  entry = info;

  // TODO Validate IP/port/id
  Locator_t remote_server_locator;
  // TODO: Check address format and use appropriate locator
  IPLocator::setIPv4(remote_server_locator, info->address);
  remote_server_locator.port = info->port;

  RemoteServerAttributes remote_server_attr;
  get_server_client_default_guidPrefix(info->id, remote_server_attr.guidPrefix);
  remote_server_attr.metatrafficUnicastLocatorList.push_back(
      remote_server_locator);
  server_qos_.wire_protocol()
      .builtin.discovery_config.m_DiscoveryServers.push_back(
          remote_server_attr);
  aux_server_->set_qos(server_qos_);
}

void DiscoveryServerListener::configure_auxiliary_server(
    const ServerInfo::ConstSharedPtr &info) {
  RCLCPP_INFO_STREAM(get_logger(),
                     "Configuring discovery server:\n"
                         << discovery_interfaces::msg::to_yaml(*info));
  Locator_t server_locator;
  IPLocator::setIPv4(server_locator, info->address);
  server_locator.port = info->port;

  server_qos_.name(info->name);
  server_qos_.wire_protocol().builtin.discovery_config.discoveryProtocol =
      DiscoveryProtocol_t::SERVER;
  get_server_client_default_guidPrefix(info->id,
                                       server_qos_.wire_protocol().prefix);
  server_qos_.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(
      server_locator);

  aux_server_ = std::unique_ptr<DomainParticipant>(
      DomainParticipantFactory::get_instance()->create_participant(
          0, server_qos_));
}
