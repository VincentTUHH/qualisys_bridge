#include "bridge.hpp"

namespace qualisys_bridge {

#define DECLARE_PARAM(x)                                    \
  do {                                                      \
    params_.x = declare_parameter<decltype(params_.x)>(#x); \
  } while (false)

void Bridge::DeclareParams() {
  DECLARE_PARAM(qtm_body_names);
  DECLARE_PARAM(ros_body_names);
  DECLARE_PARAM(server_address);
  DECLARE_PARAM(latch_timeout);
  DECLARE_PARAM(publish_visual_odometry);
}

}  // namespace qualisys_bridge
