#include "bridge.hpp"

namespace qualisys_bridge {

#define DECLARE_PARAM(x)                  \
  do {                                    \
    auto &param = params_.x;              \
    param = declare_parameter(#x, param); \
  } while (false)

void Bridge::DeclareParams() {
  std::string name;
  DECLARE_PARAM(body_name);
  DECLARE_PARAM(server_address);
}

}  // namespace qualisys_bridge
