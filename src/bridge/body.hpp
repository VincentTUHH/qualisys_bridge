#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "qualisys_bridge/ekf.hpp"
namespace qualisys_bridge {
class Body {
 public:
  struct Publishers {
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_odometry;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr naive_odometry;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        naive_accel;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr accel;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr
        px4_vehicle_odometry;
  };

  Body(rclcpp::Node &node, std::string qtm_body_name, std::string ros_body_name)
      : qtm_body_name_(qtm_body_name),
        ros_body_name_(ros_body_name),
        node_(node),
        ekf_() {
    Init();
  }

  void Init() { Init(qtm_body_name_, ros_body_name_); }

  void SetQTMBodyName(std::string name) { qtm_body_name_ = name; }
  void SetROSBodyName(std::string name) { ros_body_name_ = name; }
  void Update(Eigen::Quaterniond &orientation, Eigen::Vector3d &position);
  void LatchTimeout(bool enable) { latch_timeout_ = enable; }
  void PublishToFMU(bool enable) { publish_visual_odometry_ = enable; }

 private:
  void Init(std::string qtm_body_name, std::string ros_body_name) {
    SetQTMBodyName(qtm_body_name);
    SetROSBodyName(ros_body_name);
    mocap_timeout_timer_ = node_.create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&Body::OnMoCapTimeout, this));
    DestroyPublishers();
    DestroySubscriptions();
    CreatePublishers();
    CreateSubscriptions();
    InitEKF();
  }
  void InitEKF();
  void PublishVisualOdometry();
  void PublishGroundTruthOdometry();
  void PublishAcceleration();
  void PublishNaive(const Eigen::Vector3d &position,
                    const Eigen::Quaterniond &orientation, double time);
  void OnMoCapTimeout();
  void DestroyPublishers();
  void CreatePublishers();
  void DestroySubscriptions();
  void CreateSubscriptions();
  std::string qtm_body_name_{"qtm_body"};
  std::string ros_body_name_{"ros_body"};
  rclcpp::Node &node_;
  Publishers pubs_;
  rclcpp::TimerBase::SharedPtr mocap_timeout_timer_;
  Ekf ekf_;
  bool valid_mocap_stream_{false};
  bool latch_timeout_{false};
  bool publish_visual_odometry_{false};
  int lost_counter_{0};
};
}  // namespace qualisys_bridge
