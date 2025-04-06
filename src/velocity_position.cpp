#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace srs_mavros_plugins
{
class VelocityPositionPlugin : public mavros::plugin::Plugin
{
public:
  explicit VelocityPositionPlugin(mavros::plugin::UASPtr uas_) : Plugin(uas_, "velocity_position")
  {
    enable_node_watch_parameters();

    // header frame_id.
    // default to map (world-fixed, ENU as per REP-105).
    node_declare_and_watch_parameter("frame_id", "odom", [&](const rclcpp::Parameter & p) {
      frame_id_ = p.as_string();
      RCLCPP_INFO(get_logger(), "frame_id %s", frame_id_.c_str());
    });

    // Important tf subsection
    // Report the transform from world to base_link here.
    node_declare_and_watch_parameter(
      "tf.send", false, [&](const rclcpp::Parameter & p) { tf_send_ = p.as_bool(); });
    node_declare_and_watch_parameter(
      "tf.frame_id", "odom", [&](const rclcpp::Parameter & p) { tf_frame_id_ = p.as_string(); });
    node_declare_and_watch_parameter(
      "tf.child_frame_id", "base_link",
      [&](const rclcpp::Parameter & p) { tf_child_frame_id_ = p.as_string(); });

    auto sensor_qos = rclcpp::SensorDataQoS();
    local_odom_ = node->create_publisher<nav_msgs::msg::Odometry>("~/odom", sensor_qos);
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&VelocityPositionPlugin::handle_local_position_ned),
      make_handler(&VelocityPositionPlugin::handle_attitude),
      make_handler(&VelocityPositionPlugin::handle_wheel_distance),
    };
  }

  void handle_local_position_ned(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::LOCAL_POSITION_NED & pos_ned,
    mavros::plugin::filter::SystemAndOk filter [[maybe_unused]])
  {  
    geometry_msgs::msg::Point lpos_position_enu;
    lpos_position_enu.x = pos_ned.y;
    lpos_position_enu.y = pos_ned.x;
    lpos_position_enu.z = -pos_ned.z;
    lpos_position_enu_ = lpos_position_enu;

    geometry_msgs::msg::Vector3 lpos_velocity_enu;
    lpos_velocity_enu.x = pos_ned.vy;
    lpos_velocity_enu.y = pos_ned.vx;
    lpos_velocity_enu.z = -pos_ned.vz;
    lpos_velocity_enu_ = lpos_velocity_enu;

    // // generate body_twist
    // geometry_msgs::msg::Twist body_twist;
    // float euler_enu_roll = 0.0f;
    // float euler_enu_pitch = 0.0f;
    // if (imu_rpy_enu_) {
    //   const float vel_enu_x = pos_ned.vy;
    //   const float vel_enu_y = pos_ned.vx;
    //   euler_enu_roll = imu_rpy_enu_.value()[0];
    //   euler_enu_pitch = imu_rpy_enu_.value()[1];
    //   const float euler_enu_yaw = imu_rpy_enu_.value()[2];
    //   body_twist.linear.x =
    //     std::cos(-euler_enu_yaw) * vel_enu_x - std::sin(-euler_enu_yaw) * vel_enu_y;
    //   body_twist.linear.y =
    //     std::sin(-euler_enu_yaw) * vel_enu_x + std::cos(-euler_enu_yaw) * vel_enu_y;
    //   body_twist.linear.z = -pos_ned.vz;
    // }
    // if (imu_gyro_flu_) {
    //   body_twist.angular.x = imu_gyro_flu_.value()[0];
    //   body_twist.angular.y = imu_gyro_flu_.value()[1];
    //   if (wheel_yaw_rate_) {
    //     const float gyro_yaw_rate = imu_gyro_flu_.value()[2];
    //     const float gyro_marge_rate =
    //       std::min(std::max((std::fabs(gyro_yaw_rate) - 0.02f) / (0.5f - 0.02f), 0.0f), 1.0f);
    //     body_twist.angular.z =
    //       gyro_marge_rate * gyro_yaw_rate + (1 - gyro_marge_rate) * wheel_yaw_rate_.value();
    //   } else {
    //     body_twist.angular.z = imu_gyro_flu_.value()[2];
    //   }
    // }
    // const float pose_enu_z = -pos_ned.z;

    // if (last_stamp_) {
    //   // update 2d pose
    //   auto duration = (pos_ned.time_boot_ms - last_stamp_.value()) / 1000.0f;
    //   last_pose_2d_ = update2dPose(last_pose_2d_, body_twist, duration);
    //   // generate odometry msg
    //   nav_msgs::msg::Odometry odom;
    //   odom.header = uas->synchronized_header(frame_id_, pos_ned.time_boot_ms);
    //   odom.child_frame_id = tf_child_frame_id_;
    //   odom.pose.pose =
    //     generateVelocityPose3D(last_pose_2d_, pose_enu_z, euler_enu_roll, euler_enu_pitch);
    //   odom.twist.twist = body_twist;
    //   local_odom_->publish(odom);
    //   // broadcast TF
    //   if (tf_send_) {
    //     geometry_msgs::msg::TransformStamped transform = generateTransform(odom);
    //     uas->tf2_broadcaster.sendTransform(transform);
    //   }
    // }
    // last_stamp_ = pos_ned.time_boot_ms;
  }

  void handle_attitude(
    const mavlink::mavlink_message_t * msg [[maybe_unused]], mavlink::common::msg::ATTITUDE & att,
    mavros::plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    // gyro rate
    Eigen::Vector3d gyro_frd = Eigen::Vector3d(att.rollspeed, att.pitchspeed, att.yawspeed);
    Eigen::Vector3d gyro_flu = mavros::ftf::transform_frame_aircraft_baselink(gyro_frd);
    geometry_msgs::msg::Vector3 imu_gyro_flu;
    imu_gyro_flu.x = gyro_flu[0];
    imu_gyro_flu.y = gyro_flu[1];
    imu_gyro_flu.z = gyro_flu[2];
    imu_gyro_flu_ = imu_gyro_flu;

    // orientation rpy
    imu_rpy_enu_ = Eigen::Vector3d(att.roll, -att.pitch, M_PI / 2 - att.yaw);
  }

  void handle_wheel_distance(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::common::msg::WHEEL_DISTANCE & wheel_dist,
    mavros::plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    if (wheel_dist.count != 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *node->get_clock(), 1000, "count should be 2.");
      return;
    }

    if (last_wheel_distance_) {
      const float wheel_distance = 0.362f;
      const float diff_wheel_0 = wheel_dist.distance[0] - last_wheel_distance_.value().distance[0];
      const float diff_wheel_1 = wheel_dist.distance[1] - last_wheel_distance_.value().distance[1];
      const float diff_stamp = (wheel_dist.time_usec - last_wheel_distance_.value().time_usec) / 1000000.0f;
      if (0.001f < diff_stamp) {
        const float wheel_yaw_rate = (-diff_wheel_0 + diff_wheel_1) / wheel_distance / diff_stamp;
        const float wheel_front_vel = (diff_wheel_0 + diff_wheel_1) / 2 / diff_stamp;
        // RCLCPP_INFO(get_logger(), "(%f) %f %f", diff_stamp, wheel_front_vel, wheel_yaw_rate_.value());

        // update 2d pose
        last_pose_2d_ = update2dPose(last_pose_2d_, wheel_front_vel, wheel_yaw_rate, diff_stamp);
      }

      // generate odometry msg
      nav_msgs::msg::Odometry odom;
      odom.header.frame_id = frame_id_;
      odom.header.stamp = uas->synchronise_stamp(wheel_dist.time_usec);
      odom.child_frame_id = tf_child_frame_id_;
      // set pose
      if (imu_rpy_enu_.has_value() && lpos_position_enu_.has_value()) {
        odom.pose.pose = generateVelocityPose3D(last_pose_2d_, lpos_position_enu_.value().z, imu_rpy_enu_.value()[0], imu_rpy_enu_.value()[1]);
      } else {
        odom.pose.pose = generateVelocityPose3D(last_pose_2d_, 0.0f, 0.0f, 0.0f);
      }
      // set twist
      if (imu_rpy_enu_.has_value() && imu_gyro_flu_.has_value() && lpos_velocity_enu_.has_value()) {
        geometry_msgs::msg::Twist body_twist;
        const float euler_enu_yaw = imu_rpy_enu_.value()[2];
        body_twist.linear.x = std::cos(-euler_enu_yaw) * lpos_velocity_enu_.value().x - std::sin(-euler_enu_yaw) * lpos_velocity_enu_.value().y;
        body_twist.linear.y = std::sin(-euler_enu_yaw) * lpos_velocity_enu_.value().x + std::cos(-euler_enu_yaw) * lpos_velocity_enu_.value().y;
        body_twist.linear.z = lpos_velocity_enu_.value().z;
        body_twist.angular = imu_gyro_flu_.value();
        odom.twist.twist = body_twist;
      }
      local_odom_->publish(odom);
      if (tf_send_) {
        geometry_msgs::msg::TransformStamped transform = generateTransform(odom);
        uas->tf2_broadcaster.sendTransform(transform);
      }
    }
    last_wheel_distance_ = wheel_dist;
  }

private:
  static geometry_msgs::msg::Pose2D update2dPose(
    const geometry_msgs::msg::Pose2D & pose_2d_enu, const geometry_msgs::msg::Twist & twist_2d_frd,
    const float dt)
  {
    float vpos_half_yaw = pose_2d_enu.theta + twist_2d_frd.angular.z * dt / 2.0f;
    float vpos_vel_x = std::cos(vpos_half_yaw) * twist_2d_frd.linear.x -
                       std::sin(vpos_half_yaw) * twist_2d_frd.linear.y;
    float vpos_vel_y = std::sin(vpos_half_yaw) * twist_2d_frd.linear.x +
                       std::cos(vpos_half_yaw) * twist_2d_frd.linear.y;

    geometry_msgs::msg::Pose2D output;
    output.x = pose_2d_enu.x + vpos_vel_x * dt;
    output.y = pose_2d_enu.y + vpos_vel_y * dt;
    output.theta = pose_2d_enu.theta + twist_2d_frd.angular.z * dt;
    return output;
  }

  static geometry_msgs::msg::Pose2D update2dPose(
    const geometry_msgs::msg::Pose2D & pose_2d_enu, const float front_vel, const float yaw_rate,
    const float dt)
  {
    float vpos_half_yaw = pose_2d_enu.theta + yaw_rate * dt / 2.0f;
    float vpos_vel_x = std::cos(vpos_half_yaw) * front_vel;
    float vpos_vel_y = std::sin(vpos_half_yaw) * front_vel;

    geometry_msgs::msg::Pose2D output;
    output.x = pose_2d_enu.x + vpos_vel_x * dt;
    output.y = pose_2d_enu.y + vpos_vel_y * dt;
    output.theta = pose_2d_enu.theta + yaw_rate * dt;
    return output;
  }

  static geometry_msgs::msg::Pose generateVelocityPose3D(
    const geometry_msgs::msg::Pose2D & pose_2d_enu, const float pos_z, const float roll,
    const float pitch)
  {
    geometry_msgs::msg::Pose output;
    output.position.x = pose_2d_enu.x;
    output.position.y = pose_2d_enu.y;
    output.position.z = pos_z;
    // convert RPY -> Quaternion
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, pose_2d_enu.theta);
    output.orientation = tf2::toMsg(quat_tf);
    return output;
  }

  geometry_msgs::msg::TransformStamped generateTransform(nav_msgs::msg::Odometry & odom) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odom.header.stamp;
    transform.header.frame_id = tf_frame_id_;
    transform.child_frame_id = tf_child_frame_id_;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = odom.pose.pose.orientation;
    return transform;
  }

  void print(
    const geometry_msgs::msg::Pose & enu_pose, const geometry_msgs::msg::Twist & enu_twist,
    const float dt) const
  {
    RCLCPP_WARN(get_logger(), "diff %f", dt);
    RCLCPP_WARN(
      get_logger(), "%f %f %f", enu_pose.position.x, enu_pose.position.y, enu_pose.position.z);
    auto & quat = enu_pose.orientation;
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    RCLCPP_WARN(get_logger(), "%f %f %f", roll, pitch, yaw);
    RCLCPP_WARN(
      get_logger(), "%f %f %f", enu_twist.linear.x, enu_twist.linear.y, enu_twist.linear.z);
    RCLCPP_WARN(
      get_logger(), "%f %f %f", enu_twist.angular.x, enu_twist.angular.y, enu_twist.angular.z);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odom_{nullptr};

  geometry_msgs::msg::Pose2D last_pose_2d_;

  // from attitude
  std::optional<Eigen::Vector3d> imu_rpy_enu_;
  std::optional<geometry_msgs::msg::Vector3> imu_gyro_flu_;

  // from local_position
  std::optional<geometry_msgs::msg::Point> lpos_position_enu_;
  std::optional<geometry_msgs::msg::Vector3> lpos_velocity_enu_;
  
  // from wheel_distance
  std::optional<mavlink::common::msg::WHEEL_DISTANCE> last_wheel_distance_;

  // options set by parameter
  std::string frame_id_;           //!< frame for Pose
  std::string tf_frame_id_;        //!< origin for TF
  std::string tf_child_frame_id_;  //!< frame for TF
  std::atomic<bool> tf_send_;
};

}  // namespace srs_mavros_plugins

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(srs_mavros_plugins::VelocityPositionPlugin)
