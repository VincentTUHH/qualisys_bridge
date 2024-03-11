#include "bridge.hpp"

namespace qualisys_bridge {

#define DECLARE_PARAM(x)                                    \
  do {                                                      \
    params_.x = declare_parameter<decltype(params_.x)>(#x); \
  } while (false)

void Bridge::DeclareParams() {
  std::string name;
  DECLARE_PARAM(body_name);
  DECLARE_PARAM(server_address);
  DECLARE_PARAM(latch_timeout);
  DECLARE_PARAM(ignore_mocap_timeout);
  DECLARE_PARAM(ground_truth_only);
}

}  // namespace qualisys_bridge
