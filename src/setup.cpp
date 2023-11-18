#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace srs_mavros_plugins
{

class SetupPlugin : public mavros::plugin::Plugin
{
public:
  explicit SetupPlugin(mavros::plugin::UASPtr uas_) : Plugin(uas_, "setup")
  {
    RCLCPP_INFO(get_logger(), "#####################");
    RCLCPP_INFO(get_logger(), "#####################");
    start_stamp_ = uas_->now();

    armed_pub_ = node->create_publisher<std_msgs::msg::Bool>("~/armed", 1);
    mode_pub_ = node->create_publisher<std_msgs::msg::String>("~/mode", 1);

    arming_srv_ =
      node->create_service<std_srvs::srv::SetBool>(
      "~/request_arming",
      std::bind(
        &SetupPlugin::arming_cb, this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3));

    guided_srv_ =
      node->create_service<std_srvs::srv::SetBool>(
      "~/request_guided",
      std::bind(
        &SetupPlugin::guided_cb, this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&SetupPlugin::handle_heartbeat),
    };
  }

private:
 void handle_heartbeat(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::minimal::msg::HEARTBEAT & hb, mavros::plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    std_msgs::msg::Bool bool_msg;
    bool_msg.data = !!(hb.base_mode & mavros::utils::enum_value(mavlink::minimal::MAV_MODE_FLAG::SAFETY_ARMED));
    armed_pub_->publish(bool_msg);

    std_msgs::msg::String string_msg;
    string_msg.data = uas->str_mode_v10(hb.base_mode, hb.custom_mode);
    mode_pub_->publish(string_msg);

    if (!gp_origin_done_) {
      float elapsed_duration = (uas->now() - start_stamp_).seconds();
      if (5.0f < elapsed_duration) {
        gp_origin_done_ = true;
        RCLCPP_INFO(get_logger(), "SET_GPS_GLOBAL_ORIGIN");
        mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN gpo = {};
        gpo.time_usec = get_time_usec(uas->now());
        gpo.target_system = uas->get_tgt_system();
        // set position zero
        // gpo.latitude = req->position.latitude * 1E7;
        // gpo.longitude = req->position.longitude * 1E7;
        // gpo.altitude = (req->position.altitude +
        //   uas->data.ellipsoid_to_geoid_height(req->position)) * 1E3;
        uas->send_message(gpo);
      }
    }
  }

  void arming_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res)
  {
    RCLCPP_INFO(get_logger(), "request arm %u", req->data);
    mavlink::common::msg::COMMAND_LONG cmd{};
    uas->msg_set_target(cmd);

    cmd.command = mavros::utils::enum_value(mavlink::common::MAV_CMD::COMPONENT_ARM_DISARM);
    cmd.confirmation = 1;
    cmd.param1 = (req->data) ? 1.0 : 0.0;
    cmd.param2 = 0;
    cmd.param3 = 0;
    cmd.param4 = 0;
    cmd.param5 = 0;
    cmd.param6 = 0;
    cmd.param7 = 0;
    uas->send_message(cmd);
    res->success = true; // fix true
  }

  void guided_cb(
    const std::shared_ptr<rmw_request_id_t> req_header [[maybe_unused]],
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res)
  {
    RCLCPP_INFO(get_logger(), "set guided mode");

    mavlink::common::msg::COMMAND_LONG cmd{};
    uas->msg_set_target(cmd);
    cmd.command = mavros::utils::enum_value(mavlink::common::MAV_CMD::DO_SET_MODE);
    cmd.confirmation = 1;
    cmd.param1 = mavros::utils::enum_value(mavlink::minimal::MAV_MODE_FLAG::CUSTOM_MODE_ENABLED);
    cmd.param2 = (req->data) ? 15 : 4; //GUIDED or HOLD
    cmd.param3 = 0;
    cmd.param4 = 0;
    cmd.param5 = 0;
    cmd.param6 = 0;
    cmd.param7 = 0;
    uas->send_message(cmd);
    res->success = true; // fix true
  }

  rclcpp::Time start_stamp_{};
  bool gp_origin_done_{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_{nullptr};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_{nullptr};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arming_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr guided_srv_;
};

}  // namespace srs_mavros_plugins

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(srs_mavros_plugins::SetupPlugin)
