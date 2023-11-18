#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "std_msgs/msg/string.hpp"

namespace srs_mavros_plugins
{

class RebootPlugin : public mavros::plugin::Plugin
{
public:
  explicit RebootPlugin(mavros::plugin::UASPtr uas_) : Plugin(uas_, "reboot")
  {
    string_sub_ = node->create_subscription<std_msgs::msg::String>(
      "~/command", 10, std::bind(&RebootPlugin::onStringData, this, std::placeholders::_1));
  }

  Subscriptions get_subscriptions() override { return {}; }

private:
  void onStringData(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "reboot") {
      RCLCPP_WARN(get_logger(), "Ardupilot will reboot. exception will occurs.");
      mavlink::common::msg::COMMAND_LONG cmd{};
      uas->msg_set_target(cmd);

      cmd.command = mavros::utils::enum_value(mavlink::common::MAV_CMD::PREFLIGHT_REBOOT_SHUTDOWN);
      cmd.confirmation = 0;
      cmd.param1 = 1;
      cmd.param2 = 0;
      cmd.param3 = 0;
      cmd.param4 = 0;
      cmd.param5 = 0;
      cmd.param6 = 0;
      cmd.param7 = 0;
      uas->send_message(cmd);
    } else {
      RCLCPP_WARN(get_logger(), "unknown %s", msg->data.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_{nullptr};
};

}  // namespace srs_mavros_plugins

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(srs_mavros_plugins::RebootPlugin)
