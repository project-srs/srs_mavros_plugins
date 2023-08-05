#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace srs_mavros_plugins {

class LedControlPlugin : public mavros::plugin::Plugin {
 public:
  explicit LedControlPlugin(mavros::plugin::UASPtr uas_)
      : Plugin(uas_, "led_control") {
    color_led_sub_ = node->create_subscription<std_msgs::msg::ColorRGBA>(
        "~/color", 10,
        std::bind(&LedControlPlugin::onColorData, this, std::placeholders::_1));
  }

  Subscriptions get_subscriptions() override { return {}; }

 private:
  void onColorData(const std_msgs::msg::ColorRGBA::SharedPtr color) {
    mavlink::ardupilotmega::msg::LED_CONTROL msg = {};
    uas->msg_set_target(msg);
    msg.instance = 255;
    msg.pattern = 0;
    msg.custom_len = 3;
    msg.custom_bytes[0] = std::min(std::max((int)(color->r * 255), 0), 255);
    msg.custom_bytes[1] = std::min(std::max((int)(color->g * 255), 0), 255);
    msg.custom_bytes[2] = std::min(std::max((int)(color->b * 255), 0), 255);
    uas->send_message(msg);
  }

  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr color_led_sub_{
      nullptr};
};

}  // namespace srs_mavros_plugins

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(srs_mavros_plugins::LedControlPlugin)
