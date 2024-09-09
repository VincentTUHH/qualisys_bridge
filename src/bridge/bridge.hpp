#pragma once
#include <qualisys_bridge/qualisys/RTProtocol.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "qualisys_bridge/ekf.hpp"

namespace qualisys_bridge {
class Bridge : public rclcpp::Node {
 public:
  explicit Bridge(rclcpp::NodeOptions const &_options);

 private:
  struct Params {
    std::string body_name;
    std::string server_address;
    bool latch_timeout;
    bool publish_visual_odometry;
  } params_;
  void DeclareParams();
  void OnUpdate();
  bool Connect(unsigned short n_tries = 100);
  void OnMoCapTimeout();
  void HandlePacket(CRTPacket *_packet);
  void PublishGroundTruthOdometry();
  void PublishNaive(const Eigen::Vector3d &_position_measurement,
                    const Eigen::Quaterniond &_orientation_measurement,
                    double _time);
  void PublishAcceleration();
  void PublishVisualOdometry();
  nav_msgs::msg::Odometry getEKFOdometryMsg();

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      ground_truth_odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr naive_odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      naive_accel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr
      visual_odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      vehicle_velocity_inertial_debug_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
      px4_vehicle_odometry_sub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr vehicle_odom_update_timer_;
  rclcpp::TimerBase::SharedPtr mocap_timeout_timer_;
  CRTProtocol rt_protocol_;

  unsigned short base_port_{22222};
  int major_version_{1};
  int minor_version_{24};
  bool big_endian_{false};
  unsigned short udp_port_{6734};
  bool data_available_{false};
  bool stream_frames_{false};
  Ekf ekf_;
  Ekf::Matrix15d process_noise_;
  Ekf::Matrix6d measurement_noise_;
  double t_start_frame_ros_{0.0};
  double t_start_frame_qtm_{0.0};
  int body_index_{-1};

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
