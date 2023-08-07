#include "bridge.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>

#include <chrono>
#include <cmath>
#include <hippo_common/convert.hpp>
#include <hippo_common/tf2_utils.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "qualisys_bridge/qualisys/RTProtocol.h"

RCLCPP_COMPONENTS_REGISTER_NODE(qualisys_bridge::Bridge)

static constexpr double kMaxAcceleration = 10.0;
static constexpr double kFrameInterval = 0.01;
static constexpr double kPi = 3.141592653589793238463;
static const Eigen::Quaterniond q_ned_enu{
    hippo_common::tf2_utils::EulerToQuaternion(kPi, 0.0, kPi / 2.0)};
static const Eigen::Quaterniond q_flu_frd{
    hippo_common::tf2_utils::EulerToQuaternion(kPi, 0.0, 0.0)};

namespace qualisys_bridge {
Bridge::Bridge(rclcpp::NodeOptions const &_options)
    : Node("apriltag_simple", _options), ekf_() {
  DeclareParams();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  process_noise_.topLeftCorner<6, 6>() = 0.5 * Ekf::Matrix6d::Identity() *
                                         kFrameInterval * kFrameInterval *
                                         kMaxAcceleration;
  process_noise_.block<6, 6>(6, 6) =
      Ekf::Matrix6d::Identity() * kFrameInterval * kMaxAcceleration;
  process_noise_.bottomRightCorner<3, 3>() =
      10.0 * Eigen::Matrix3d::Identity() * kFrameInterval * kMaxAcceleration;
  process_noise_ *= process_noise_;

  measurement_noise_ = Ekf::Matrix6d::Identity() * 5e-3;
  measurement_noise_ *= measurement_noise_;

  ekf_.Init(process_noise_, measurement_noise_, 100);

  rclcpp::QoS px4_qos{1};
  px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

  ground_truth_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "ground_truth/odometry", rclcpp::SystemDefaultsQoS());

  vehicle_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "odometry", rclcpp::SystemDefaultsQoS());

  accel_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "acceleration", rclcpp::SystemDefaultsQoS());

  naive_odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "odometry_naive", rclcpp::SystemDefaultsQoS());

  naive_accel_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "acceleration_naive", rclcpp::SystemDefaultsQoS());

  visual_odometry_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      "fmu/in/vehicle_visual_odometry", px4_qos);

  vehicle_velocity_inertial_debug_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
          "debug/velocity_inertial", rclcpp::SystemDefaultsQoS());

  px4_vehicle_odometry_sub_ =
      create_subscription<px4_msgs::msg::VehicleOdometry>(
          "fmu/out/vehicle_odometry", px4_qos,
          std::bind(&Bridge::OnOdometry, this, std::placeholders::_1));

  update_timer_ =
      rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(5),
                           std::bind(&Bridge::OnUpdate, this));

  vehicle_odom_update_timer_ =
      rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(20),
                           std::bind(&Bridge::OnUpdateOdometry, this));

  mocap_timeout_timer_ =
      rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(500),
                           std::bind(&Bridge::OnMoCapTimeout, this));
}

void Bridge::OnOdometry(px4_msgs::msg::VehicleOdometry::ConstSharedPtr _msg) {
  px4_odometry_updated_ = true;
  for (size_t i = 0; i < 3; ++i) {
    velocity_px4_(i) = _msg->velocity.at(i);
    position_px4_(i) = _msg->position.at(i);
    body_rates_px4_(i) = _msg->angular_velocity.at(i);
  }
  orientation_px4_.w() = _msg->q.at(0);
  orientation_px4_.x() = _msg->q.at(1);
  orientation_px4_.y() = _msg->q.at(2);
  orientation_px4_.z() = _msg->q.at(3);
}

void Bridge::OnMoCapTimeout() {
  valid_mocap_stream_ = false;
  RCLCPP_ERROR(this->get_logger(),
               "Timeout for MoCap data, stop publishing states");
}

void Bridge::HandlePacket(CRTPacket *_packet) {
  static double t_prev_frame_qtm = 0.0;
  static int lost_counter = 0;
  float x, y, z;
  float R[9];
  double t_packet = _packet->GetTimeStamp() * 1e-6;
  if (t_start_frame_ros_ == 0.0) {
    t_start_frame_ros_ = now().nanoseconds() * 1e-9;
    t_start_frame_qtm_ = _packet->GetTimeStamp() * 1e-6;
    return;
  }
  if (t_prev_frame_qtm > t_packet) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Dropping older packet.");
    return;
  }
  t_prev_frame_qtm = t_packet;
  for (unsigned int i = 0; i < _packet->Get6DOFBodyCount(); ++i) {
    std::string name{rt_protocol_.Get6DOFBodyName(i)};
    if (name != params_.body_name) {
      continue;
    }
    if (!_packet->Get6DOFBody(i, x, y, z, R)) {
      RCLCPP_WARN(get_logger(), "Failed to retrieve 6DOF data.");
      continue;
    }
    bool matrix_nan{false};
    for (int j = 0; j < 9; ++j) {
      if (std::isnan(R[j])) {
        matrix_nan = true;
      }
    }
    if (std::isnan(x) || std::isnan(y) || std::isnan(z) || matrix_nan) {
      lost_counter++;
      if (lost_counter >= 50) {
        ekf_.Reset();
      }
      continue;
    }
    mocap_timeout_timer_->reset();
    // only set set to true if we do not latch the timeout state. if latched, we
    // should not recover from a timeout state.
    valid_mocap_stream_ = params_.latch_timeout ? valid_mocap_stream_ : true;
    lost_counter = 0;
    Eigen::Matrix<float, 3, 3, Eigen::ColMajor> R_mat(R);
    Eigen::Quaterniond orientation_measurement{R_mat.cast<double>()};
    Eigen::Vector3d position_measurement{x / 1000.0, y / 1000.0, z / 1000.0};
    position_measurement += Eigen::Vector3d{0.0, 0.0, 0.0};

    double time = t_start_frame_ros_ + (t_packet - t_start_frame_qtm_);

    if (!ekf_.IsReady()) {
      ekf_.SetInitialCondition(time, orientation_measurement,
                               position_measurement);
      continue;
    }
    ekf_.Predict(time);
    ekf_.Update(orientation_measurement, position_measurement);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
    hippo_common::convert::EigenToRos(ekf_.GetPosition(), pose.pose.position);
    hippo_common::convert::EigenToRos(ekf_.GetOrientation(),
                                      pose.pose.orientation);
    // PublishVisualOdometry(pose);
    PublishVisualOdometry();
    PublishOdometry();
    PublishAcceleration();
    PublishNaive(position_measurement, orientation_measurement, t_packet);
  }
}

void Bridge::PublishNaive(const Eigen::Vector3d &_position_measurement,
                          const Eigen::Quaterniond &_orientation_measurement,
                          double _time) {
  static double t_last = 0.0;
  double dt = _time - t_last;
  t_last = _time;
  static Eigen::Vector3d last_position{0.0, 0.0, 0.0};
  static Eigen::Vector3d last_v_lin{0.0, 0.0, 0.0};
  static Eigen::Quaterniond last_orientation{1.0, 0.0, 0.0, 0.0};

  Eigen::Vector3d v_lin = (_position_measurement - last_position) / dt;
  Eigen::Vector3d accel = (v_lin - last_v_lin) / dt;
  last_v_lin = v_lin;
  last_position = _position_measurement;

  Eigen::Quaterniond delta_q =
      _orientation_measurement * last_orientation.inverse();
  last_orientation = _orientation_measurement;
  Eigen::AngleAxisd delta_angle_axis{delta_q};
  double angle = delta_angle_axis.angle();
  if (std::abs(angle) > std::abs(2 * M_PI - angle)) {
    delta_angle_axis.angle() = 2 * M_PI - angle;
    delta_angle_axis.axis() = -delta_angle_axis.axis();
  }
  Eigen::Vector3d v_ang =
      delta_angle_axis.axis() * delta_angle_axis.angle() / dt;
  v_ang = _orientation_measurement.inverse() * v_ang;

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  odom_msg.header.stamp = now();
  hippo_common::convert::EigenToRos(_position_measurement,
                                    odom_msg.pose.pose.position);
  hippo_common::convert::EigenToRos(_orientation_measurement,
                                    odom_msg.pose.pose.orientation);
  hippo_common::convert::EigenToRos(v_lin, odom_msg.twist.twist.linear);
  hippo_common::convert::EigenToRos(v_ang, odom_msg.twist.twist.angular);
  naive_odometry_pub_->publish(odom_msg);

  geometry_msgs::msg::Vector3Stamped accel_msg;
  accel_msg.header = odom_msg.header;
  hippo_common::convert::EigenToRos(accel, accel_msg.vector);
  naive_accel_pub_->publish(accel_msg);
}

void Bridge::PublishOdometry() {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = now();
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  msg.child_frame_id = "base_link";
  hippo_common::convert::EigenToRos(ekf_.GetPosition(), msg.pose.pose.position);
  hippo_common::convert::EigenToRos(ekf_.GetOrientation(),
                                    msg.pose.pose.orientation);
  hippo_common::convert::EigenToRos(ekf_.GetLinearVelocity(),
                                    msg.twist.twist.linear);
  hippo_common::convert::EigenToRos(
      ekf_.GetOrientation().inverse() * ekf_.GetAngularVelocity(),
      msg.twist.twist.angular);
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> pose_covariance(
      msg.pose.covariance.begin());
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> twist_covariance(
      msg.twist.covariance.begin());

  Eigen::Matrix<double, 15, 15> covariance = ekf_.GetStateCovariance();

  pose_covariance.topLeftCorner<3, 3>() = covariance.block<3, 3>(3, 3);
  pose_covariance.topRightCorner<3, 3>() = covariance.block<3, 3>(3, 0);
  pose_covariance.bottomLeftCorner<3, 3>() = covariance.block<3, 3>(0, 3);
  pose_covariance.bottomRightCorner<3, 3>() = covariance.block<3, 3>(0, 0);

  twist_covariance.topLeftCorner<3, 3>() = covariance.block<3, 3>(9, 9);
  twist_covariance.topRightCorner<3, 3>() = covariance.block<3, 3>(9, 6);
  twist_covariance.bottomLeftCorner<3, 3>() = covariance.block<3, 3>(6, 9);
  twist_covariance.bottomRightCorner<3, 3>() = covariance.block<3, 3>(6, 6);

  ground_truth_odometry_pub_->publish(msg);
}

void Bridge::PublishAcceleration() {
  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(ekf_.GetLinearAcceleration(), msg.vector);
  accel_pub_->publish(msg);
}

void Bridge::OnUpdate() {
  if (!rt_protocol_.Connected()) {
    RCLCPP_INFO(get_logger(), "Trying to connect.");
    if (!Connect()) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Connected.");
  }
  if (!data_available_) {
    if (!rt_protocol_.Read6DOFSettings(data_available_)) {
      RCLCPP_WARN(get_logger(), "rtProtocol.StreamFrames: %s\n\n",
                  rt_protocol_.GetErrorString());
      return;
    }
  }

  if (!stream_frames_) {
    if (!rt_protocol_.StreamFrames(CRTProtocol::RateAllFrames, 0, udp_port_,
                                   NULL, CRTProtocol::cComponent6d)) {
      RCLCPP_WARN(get_logger(), "rtProtocol.StreamFrames: %s\n\n",
                  rt_protocol_.GetErrorString());
      return;
    }
    stream_frames_ = true;
    RCLCPP_INFO(get_logger(), "Start streaming.");
  }

  CRTPacket::EPacketType packet_type;
  if (rt_protocol_.Receive(packet_type, true) ==
      CNetwork::ResponseType::success) {
    if (packet_type == CRTPacket::PacketData) {
      CRTPacket *packet = rt_protocol_.GetRTPacket();
      HandlePacket(packet);
    }
  }
}

bool Bridge::Connect() {
  if (!rt_protocol_.Connect(params_.server_address.c_str(), base_port_,
                            &udp_port_, major_version_, minor_version_,
                            big_endian_)) {
    RCLCPP_ERROR(get_logger(), "Failed to connect: %s",
                 rt_protocol_.GetErrorString());
    return false;
  }
  return true;
}

void Bridge::OnUpdateOdometry() {
  if (!params_.ignore_mocap_timeout && !valid_mocap_stream_) {
    return;
  }

  if (!px4_odometry_updated_) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No odometry received from px4. Cannot publish odometry.");
    return;
  }
  px4_odometry_updated_ = false;

  rclcpp::Time stamp = now();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(position_px4_, pose.pose.position);
  hippo_common::convert::EigenToRos(orientation_px4_, pose.pose.orientation);
  pose.pose = hippo_common::tf2_utils::PosePx4ToRos(pose.pose);

  // velocity in inertial coordinate system in Ros
  Eigen::Vector3d velocity_ros = q_ned_enu.inverse() * velocity_px4_;
  geometry_msgs::msg::Vector3Stamped velocity_inertial_msg;
  velocity_inertial_msg.header.stamp = stamp;
  velocity_inertial_msg.header.frame_id =
      hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(velocity_ros, velocity_inertial_msg.vector);
  vehicle_velocity_inertial_debug_pub_->publish(velocity_inertial_msg);

  // transform velocity from inertial coordinate system in ros to body frame
  velocity_ros =
      (q_ned_enu.inverse() * orientation_px4_ * q_flu_frd.inverse()).inverse() *
      velocity_ros;

  Eigen::Vector3d angular_velocity_ros = q_flu_frd * body_rates_px4_;

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  odometry.child_frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
  odometry.pose.pose = pose.pose;
  hippo_common::convert::EigenToRos(velocity_ros, odometry.twist.twist.linear);
  hippo_common::convert::EigenToRos(angular_velocity_ros,
                                    odometry.twist.twist.angular);
  vehicle_odometry_pub_->publish(odometry);
}

void Bridge::PublishVisualOdometry(
    const geometry_msgs::msg::PoseStamped _pose) {
  static int counter = 0;

  counter++;
  if (counter < 3) {
    return;
  }
  counter = 0;
  geometry_msgs::msg::Pose px4_pose =
      hippo_common::tf2_utils::PoseRosToPx4(_pose.pose);
  px4_msgs::msg::VehicleOdometry visual_odometry;
  visual_odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  visual_odometry.position = {(float)px4_pose.position.x,
                              (float)px4_pose.position.y,
                              (float)px4_pose.position.z};
  visual_odometry.q = {
      (float)px4_pose.orientation.w, (float)px4_pose.orientation.x,
      (float)px4_pose.orientation.y, (float)px4_pose.orientation.z};
  visual_odometry.timestamp = now().nanoseconds() * 1e-3;
  visual_odometry.timestamp_sample =
      _pose.header.stamp.nanosec * 1e-3 + _pose.header.stamp.sec * 1e6;
  visual_odometry.velocity = {NAN, NAN, NAN};
  visual_odometry.angular_velocity = {NAN, NAN, NAN};

  visual_odometry_pub_->publish(visual_odometry);
}

void Bridge::PublishVisualOdometry() {
  px4_msgs::msg::VehicleOdometry visual_odometry;
  visual_odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  visual_odometry.velocity_frame =
      px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
  visual_odometry.timestamp = now().nanoseconds() * 1e-3;
  visual_odometry.timestamp_sample = now().nanoseconds() * 1e-3;
  geometry_msgs::msg::Pose ros_pose;
  hippo_common::convert::EigenToRos(ekf_.GetPosition(), ros_pose.position);

  hippo_common::convert::EigenToRos(ekf_.GetOrientation(),
                                    ros_pose.orientation);
  geometry_msgs::msg::Pose px4_pose =
      hippo_common::tf2_utils::PoseRosToPx4(ros_pose);
  visual_odometry.position = {(float)px4_pose.position.x,
                              (float)px4_pose.position.y,
                              (float)px4_pose.position.z};
  visual_odometry.q = {
      (float)px4_pose.orientation.w, (float)px4_pose.orientation.x,
      (float)px4_pose.orientation.y, (float)px4_pose.orientation.z};

  Eigen::Vector3d velocity_inertial_px4 = q_ned_enu * ekf_.GetLinearVelocity();
  Eigen::Vector3d angular_velocity_local_px4 =
      q_flu_frd.inverse() * ekf_.GetOrientation().inverse() *
      Eigen::Vector3d(ekf_.GetAngularVelocity());

  visual_odometry.velocity = {(float)velocity_inertial_px4.x(),
                              (float)velocity_inertial_px4.y(),
                              (float)velocity_inertial_px4.z()};
  visual_odometry.angular_velocity = {(float)angular_velocity_local_px4.x(),
                                      (float)angular_velocity_local_px4.y(),
                                      (float)angular_velocity_local_px4.z()};

  Eigen::Matrix<double, 3, 3> position_covariance;
  Eigen::Matrix<double, 3, 3> orientation_covariance;
  Eigen::Matrix<double, 3, 3> velocity_covariance;

  Eigen::Matrix<double, 15, 15> covariance = ekf_.GetStateCovariance();

  position_covariance = covariance.block<3, 3>(3, 3);
  orientation_covariance = covariance.block<3, 3>(0, 0);
  velocity_covariance = covariance.block<3, 3>(9, 9);

  position_covariance = q_ned_enu.toRotationMatrix() * position_covariance *
                        q_ned_enu.inverse().toRotationMatrix();
  velocity_covariance = q_ned_enu.toRotationMatrix() * velocity_covariance *
                        q_ned_enu.inverse().toRotationMatrix();
  // todo: properly transform orientation noise from ROS coordinate systems to
  // px4 coordinate systems
  for (int i = 0; i < 3; i++) {  // rows
    visual_odometry.position_variance[i] = (float)position_covariance(i, i);
    visual_odometry.orientation_variance[i] =
        (float)orientation_covariance(i, i);
    visual_odometry.velocity_variance[i] = (float)velocity_covariance(i, i);
  }
  visual_odometry_pub_->publish(visual_odometry);
}

nav_msgs::msg::Odometry Bridge::getEKFOdometryMsg() {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = now();
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  msg.child_frame_id = "base_link";
  hippo_common::convert::EigenToRos(ekf_.GetPosition(), msg.pose.pose.position);
  hippo_common::convert::EigenToRos(ekf_.GetOrientation(),
                                    msg.pose.pose.orientation);
  hippo_common::convert::EigenToRos(ekf_.GetLinearVelocity(),
                                    msg.twist.twist.linear);
  hippo_common::convert::EigenToRos(
      ekf_.GetOrientation().inverse() * ekf_.GetAngularVelocity(),
      msg.twist.twist.angular);
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> pose_covariance(
      msg.pose.covariance.begin());
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> twist_covariance(
      msg.twist.covariance.begin());

  Eigen::Matrix<double, 15, 15> covariance = ekf_.GetStateCovariance();

  pose_covariance.topLeftCorner<3, 3>() = covariance.block<3, 3>(3, 3);
  pose_covariance.topRightCorner<3, 3>() = covariance.block<3, 3>(3, 0);
  pose_covariance.bottomLeftCorner<3, 3>() = covariance.block<3, 3>(0, 3);
  pose_covariance.bottomRightCorner<3, 3>() = covariance.block<3, 3>(0, 0);

  twist_covariance.topLeftCorner<3, 3>() = covariance.block<3, 3>(9, 9);
  twist_covariance.topRightCorner<3, 3>() = covariance.block<3, 3>(9, 6);
  twist_covariance.bottomLeftCorner<3, 3>() = covariance.block<3, 3>(6, 9);
  twist_covariance.bottomRightCorner<3, 3>() = covariance.block<3, 3>(6, 6);
  return msg;
}

}  // namespace qualisys_bridge
