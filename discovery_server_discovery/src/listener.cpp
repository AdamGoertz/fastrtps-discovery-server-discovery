#include <discovery_server_discovery/listener.hpp>

#include <rclcpp/subscription_options.hpp>

#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <ostream>

using eprosima::fastdds::dds::DomainParticipantQos;
using std::placeholders::_1;

std::string id_to_guid_prefix(uint8_t id) {
  // We must convert `id` to uint16_t because C++ has special rules for
  // formatting `char` types, and uint8_t is an alias of `char`.
  std::ostringstream ss;
  ss << "44.53." << std::setw(2) << std::setfill('0') << std::hex
     << static_cast<uint16_t>(id) << ".5f.45.50.52.4f.53.49.4d.41";
  return ss.str();
}

DiscoveryServerListener::DiscoveryServerListener()
    : rclcpp::Node("discovery_server_manager") {

  auto env_file = std::getenv("FASTDDS_ENVIRONMENT_FILE");
  if (env_file == nullptr) {
    RCLCPP_FATAL(get_logger(),
                 "FASTDDS_ENVIRONMENT_FILE environment variable not set");
    exit(1);
  }
  fastdds_environment_file_ = env_file;
  RCLCPP_INFO_STREAM(get_logger(), "using FastDDS environment file: "
                                       << fastdds_environment_file_.native());

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

  update_server_info(this_server_info);

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
  // Ensure there is enough space for the new server
  server_info_.resize(info->id + 1);
  if (server_info_[info->id] != nullptr) {
    RCLCPP_WARN_STREAM(
        "Overwriting discovery info for server:\n"
        << discovery_interfaces::msg::to_yaml(server_info_[info->id])
        << "with new info:\n"
        << discovery_interfaces::msg::to_yaml(info));
  }
  server_info_[info->id] = info;
  write_environment_file();
}

void DiscoveryServerListener::write_environment_file() const {
  std::ofstream env_file(fastdds_environment_file_.native(), std::ios::out);
  env_file << "{\n"
           << "\t\"ROS_DISCOVERY_SERVER\": \"" << format_server_info() << "\"\n"
           << "}\n";
}

std::string DiscoveryServerListener::format_server_info() const {
  std::ostringstream ss;
  for (const auto &info : server_info_) {
    if (info) {
      ss << info->address << ":" << info->port;
    }
    ss << ";";
  }
  return ss.str();
}
