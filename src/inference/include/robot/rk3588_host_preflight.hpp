#pragma once

#include <string>

namespace inference {

// 固定部署脚本执行 live check；规则仅由 robot-rt-setup 维护。
bool verify_rk3588_host_layout(std::string& error);

}  // namespace inference
