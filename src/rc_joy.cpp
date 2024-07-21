#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace srs_mavros_plugins
{

class RcJoyPlugin : public mavros::plugin::Plugin
{
public:
  explicit RcJoyPlugin(mavros::plugin::UASPtr uas_) : Plugin(uas_, "rc_joy")
  {
    joy_pub_ = node->create_publisher<sensor_msgs::msg::Joy>("~/joy", 1);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&RcJoyPlugin::handle_rc_channels),
    };
  }

private:
  void handle_rc_channels(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::RC_CHANNELS & channels,
    mavros::plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    sensor_msgs::msg::Joy output;
    output.header = uas->synchronized_header("", channels.time_boot_ms);

    if (8 <= channels.chancount) {
      // axes
      output.axes.push_back(-decodeStick(channels.chan4_raw)); // left-horizontal
      output.axes.push_back(decodeStick(channels.chan2_raw)); // left-vertical
      output.axes.push_back(0.0f);
      output.axes.push_back(-decodeStick(channels.chan1_raw)); // right-horizon
      output.axes.push_back(decodeStick(channels.chan3_raw)); // right-vertical
      output.axes.push_back(0.0f);
      // buttons
      output.buttons.push_back(decode_2pos(channels.chan5_raw));
      output.buttons.push_back(get_9pos_hi(channels.chan6_raw));
      output.buttons.push_back(get_9pos_lo(channels.chan6_raw));
    } else { // no RC connection
      // axes
      output.axes.push_back(0.0f);
      output.axes.push_back(0.0f);
      output.axes.push_back(0.0f);
      output.axes.push_back(0.0f);
      // buttons
      output.buttons.push_back(0);
      output.buttons.push_back(0);
      output.buttons.push_back(0);
    }
    joy_pub_->publish(output);
  }
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_{nullptr};

  static float decodeStick(const int value) {
    float raw_offset = value - axes_center_;
    return raw_offset / stick_scale_;
  }

  static int decode_2pos(const int value)
  {
    if (axes_center_ < value) {
      return 1;
    } else {
      return 0;
    }
  }

  static int get_9pos_hi(const int value)
  {
    int pos = decode_9pos(value);
    return (pos / 3) % 3;
  }

  static int get_9pos_lo(const int value)
  {
    int pos = decode_9pos(value);
    return pos % 3;
  }

  static int decode_9pos(const int value)
  {
    int step_half = (value - (axes_center_ - switch_scale_)) / (switch_scale_ / 8.0f);
    return (step_half + 1) / 2;
  }

  static constexpr int axes_center_ = 1515;
  static constexpr int switch_scale_ = 550;
  static constexpr int stick_scale_ = 420;
};

}  // namespace srs_mavros_plugins

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(srs_mavros_plugins::RcJoyPlugin)
