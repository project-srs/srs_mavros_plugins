#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "std_msgs/msg/string.hpp"

namespace srs_mavros_plugins
{

class PlayTuneV1Plugin : public mavros::plugin::Plugin
{
public:
  explicit PlayTuneV1Plugin(mavros::plugin::UASPtr uas_) : Plugin(uas_, "play_tune_v1")
  {
    sound_tune_sub_ = node->create_subscription<std_msgs::msg::String>(
      "~/tune", 10, std::bind(&PlayTuneV1Plugin::onTuneData, this, std::placeholders::_1));
  }

  Subscriptions get_subscriptions() override { return {}; }

private:
  void onTuneData(const std_msgs::msg::String::SharedPtr tune)
  {
    mavlink::common::msg::PLAY_TUNE msg = {};
    uas->msg_set_target(msg);
    mavlink::set_string_z(msg.tune, tune->data);
    uas->send_message(msg);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sound_tune_sub_{nullptr};
};

}  // namespace srs_mavros_plugins

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(srs_mavros_plugins::PlayTuneV1Plugin)
