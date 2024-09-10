#include "body.hpp"

#include <hippo_common/convert.hpp>
#include <hippo_common/tf2_utils.hpp>

namespace qualisys_bridge {

static constexpr double kMaxAcceleration = 10.0;
static constexpr double kFrameInterval = 0.01;
static constexpr double kPi = 3.141592653589793238463;
static const Eigen::Quaterniond q_ned_enu{
    hippo_common::tf2_utils::EulerToQuaternion(kPi, 0.0, kPi / 2.0)};
static const Eigen::Quaterniond q_flu_frd{
    hippo_common::tf2_utils::EulerToQuaternion(kPi, 0.0, 0.0)};

void Body::InitEKF() {
  Ekf::Matrix15d process_noise;
  process_noise.topLeftCorner<6, 6>() = 0.5 * Ekf::Matrix6d::Identity() *
                                        kFrameInterval * kFrameInterval *
                                        kMaxAcceleration;
  process_noise.block<6, 6>(6, 6) =
      Ekf::Matrix6d::Identity() * kFrameInterval * kMaxAcceleration;
  process_noise.bottomRightCorner<3, 3>() =
      10.0 * Eigen::Matrix3d::Identity() * kFrameInterval * kMaxAcceleration;
  process_noise *= process_noise;

  Ekf::Matrix6d measurement_noise;
  measurement_noise = Ekf::Matrix6d::Identity() * 5e-3;
  measurement_noise *= measurement_noise;
  ekf_.Init(process_noise, measurement_noise, 100);
}

void Body::DestroyPublishers() {
  pubs_.odometry.reset();
  pubs_.naive_odometry.reset();
  pubs_.naive_accel.reset();
  pubs_.accel.reset();
  pubs_.px4_vehicle_odometry.reset();
}
void Body::CreatePublishers() {
  std::string topic;

  rclcpp::QoS qos = rclcpp::SensorDataQoS().keep_last(1);

  using geometry_msgs::msg::Vector3Stamped;
  using nav_msgs::msg::Odometry;
  using px4_msgs::msg::VehicleOdometry;

  topic = ros_body_name_ + "/ground_truth/odometry";
  pubs_.ground_truth_odometry = node_.create_publisher<Odometry>(topic, qos);

  topic = ros_body_name_ + "/acceleration";
  pubs_.accel = node_.create_publisher<Vector3Stamped>(topic, qos);

  topic = ros_body_name_ + "/odometry_naive";
  pubs_.naive_odometry = node_.create_publisher<Odometry>(topic, qos);

  topic = ros_body_name_ + "/acceleration_naive";
  pubs_.naive_accel = node_.create_publisher<Vector3Stamped>(topic, qos);

  rclcpp::QoS px4_qos{1};
  px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
  px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

  topic = ros_body_name_ = "/fmu/in/vehicle_visual_odometry";
  pubs_.px4_vehicle_odometry =
      node_.create_publisher<VehicleOdometry>(topic, px4_qos);
}

void Body::DestroySubscriptions() {}
void Body::CreateSubscriptions() {}

void Body::OnMoCapTimeout() {
  valid_mocap_stream_ = false;
  ekf_.Reset();
  RCLCPP_ERROR(node_.get_logger(),
               "Motion Capture data timed out for body <%s>",
               qtm_body_name_.c_str());
  // we can stop the timer since we already recognized the timeout.
  // It will be reset as soon as new valid data arrives.
  mocap_timeout_timer_->cancel();
}

void Body::Update(Eigen::Quaterniond &_orientation,
                  Eigen::Vector3d &_position) {
  mocap_timeout_timer_->reset();
  // data is there. so if we do not latch mocap timeouts, make sure to flag
  // the stream as valid. otherwise keep whatever state we are currently in.
  valid_mocap_stream_ = latch_timeout_ ? valid_mocap_stream_ : true;
  if (!valid_mocap_stream_) {
    return;
  }
  double t_now = node_.now().nanoseconds() * 1e-9;
  if (!ekf_.IsReady()) {
    ekf_.SetInitialCondition(t_now, _orientation, _position);
    return;
  }
  ekf_.Predict(t_now);
  ekf_.Update(_orientation, _position);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node_.now();
  pose.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(ekf_.GetPosition(), pose.pose.position);
  hippo_common::convert::EigenToRos(ekf_.GetOrientation(),
                                    pose.pose.orientation);

  if (publish_visual_odometry_) {
    PublishVisualOdometry();
  }
  PublishGroundTruthOdometry();
  PublishAcceleration();
  PublishNaive(_position, _orientation, t_now);
}

void Body::PublishVisualOdometry() {
  px4_msgs::msg::VehicleOdometry visual_odometry;
  visual_odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  visual_odometry.velocity_frame =
      px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
  visual_odometry.timestamp = node_.now().nanoseconds() * 1e-3;
  visual_odometry.timestamp_sample = node_.now().nanoseconds() * 1e-3;
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
  pubs_.px4_vehicle_odometry->publish(visual_odometry);
}
void Body::PublishNaive(const Eigen::Vector3d &_position_measurement,
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
  odom_msg.header.stamp = node_.now();
  hippo_common::convert::EigenToRos(_position_measurement,
                                    odom_msg.pose.pose.position);
  hippo_common::convert::EigenToRos(_orientation_measurement,
                                    odom_msg.pose.pose.orientation);
  hippo_common::convert::EigenToRos(v_lin, odom_msg.twist.twist.linear);
  hippo_common::convert::EigenToRos(v_ang, odom_msg.twist.twist.angular);
  pubs_.naive_odometry->publish(odom_msg);

  geometry_msgs::msg::Vector3Stamped accel_msg;
  accel_msg.header = odom_msg.header;
  hippo_common::convert::EigenToRos(accel, accel_msg.vector);
  pubs_.naive_accel->publish(accel_msg);
}

void Body::PublishGroundTruthOdometry() {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = node_.now();
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

  pubs_.ground_truth_odometry->publish(msg);
}

void Body::PublishAcceleration() {
  geometry_msgs::msg::Vector3Stamped msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
  hippo_common::convert::EigenToRos(ekf_.GetLinearAcceleration(), msg.vector);
  pubs_.accel->publish(msg);
}
}  // namespace qualisys_bridge
