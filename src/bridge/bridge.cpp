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
Bridge::Bridge(rclcpp::NodeOptions const &_options) : Node("mocap", _options) {
  DeclareParams();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  update_timer_ =
      rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(1),
                           std::bind(&Bridge::OnUpdate, this));

  if (params_.qtm_body_names.size() != params_.ros_body_names.size()) {
    RCLCPP_FATAL(get_logger(),
                 "qtm_body_names and ros_body_names parameters have to have "
                 "the same length! Node will be inactive.");
    update_timer_->cancel();
    tf_listener_ = nullptr;
    return;
  }
  for (std::size_t i = 0; i < params_.qtm_body_names.size(); ++i) {
    std::string qtm_name = params_.qtm_body_names.at(i);
    std::string ros_name = params_.ros_body_names.at(i);
    bodies_[qtm_name] = std::make_unique<Body>(*this, qtm_name, ros_name);
  }
}

void Bridge::HandlePacket(CRTPacket *_packet) {
  static double t_prev_frame_qtm = 0.0;
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
    if (!bodies_.count(name)) {
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
        break;
      }
    }
    if (std::isnan(x) || std::isnan(y) || std::isnan(z) || matrix_nan) {
      continue;
    }
    Eigen::Matrix<float, 3, 3, Eigen::ColMajor> R_mat(R);
    Eigen::Quaterniond orientation_measurement{R_mat.cast<double>()};
    Eigen::Vector3d position_measurement{x / 1000.0, y / 1000.0, z / 1000.0};
    position_measurement += Eigen::Vector3d{0.0, 0.0, 0.0};
    bodies_.at(name)->Update(orientation_measurement, position_measurement);
  }
}

bool Bridge::CheckQTMConfig() {
  if (rt_protocol_.Read6DOFSettings(data_available_)) {
    RCLCPP_WARN(get_logger(), "Could not read QTM Config: %s",
                rt_protocol_.GetErrorString());
    return false;
  }
  std::set<std::string> qtm_existing_bodies;
  for (unsigned int i = 0; i < rt_protocol_.Get6DOFBodyCount(); ++i) {
    qtm_existing_bodies.insert(rt_protocol_.Get6DOFBodyName(i));
  }
  for (const auto &requested_body : params_.qtm_body_names) {
    if (!qtm_existing_bodies.count(requested_body)) {
      RCLCPP_WARN(
          get_logger(),
          "Rigid body with name <%s> requested but does not exist in QTM.",
          requested_body.c_str());
    }
  }
  return true;
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
    if (!CheckQTMConfig()) {
      return;
    }
  }

  if (!stream_frames_) {
    // read with the full framerate configured in qtm.
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

}  // namespace qualisys_bridge
