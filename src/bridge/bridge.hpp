#pragma once
#include <qualisys_bridge/qualisys/RTProtocol.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

#include "body.hpp"
#include "qualisys_bridge/ekf.hpp"

namespace qualisys_bridge {
class Bridge : public rclcpp::Node {
 public:
  explicit Bridge(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    std::string server_address;
    std::vector<std::string> qtm_body_names;
    std::vector<std::string> ros_body_names;
    std::vector<bool> latch_timeout;
    std::vector<bool> publish_visual_odometry;
  } params_;
  void DeclareParams();
  void OnUpdate();
  bool Connect();
  bool CheckQTMConfig();
  void HandlePacket(CRTPacket *_packet);
  nav_msgs::msg::Odometry getEKFOdometryMsg();

  std::unordered_map<std::string, std::unique_ptr<Body>> bodies_;

  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr vehicle_odom_update_timer_;
  CRTProtocol rt_protocol_;

  std::string server_address_{"192.168.0.161"};
  unsigned short base_port_{22222};
  int major_version_{1};
  int minor_version_{19};
  bool big_endian_{false};
  unsigned short udp_port_{6734};
  bool data_available_{false};
  bool stream_frames_{false};
  double t_start_frame_ros_{0.0};
  double t_start_frame_qtm_{0.0};

  Eigen::Vector3d position_px4_;
  Eigen::Quaterniond orientation_px4_;
  Eigen::Vector3d velocity_px4_;
  Eigen::Vector3d body_rates_px4_;
  bool px4_odometry_updated_{false};
  bool valid_mocap_stream_{true};

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};
}  // namespace qualisys_bridge
