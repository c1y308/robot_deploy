#pragma once

#include <string>

namespace inference {

struct Rk3588HostPaths {
    std::string proc_root{"/proc"};
    std::string sys_root{"/sys"};
    std::string run_root{"/run"};
    std::string dev_root{"/dev"};
    std::string etc_root{"/etc"};
};

bool verify_rk3588_host_layout(std::string& error,
                               const Rk3588HostPaths& paths = {});

}  // namespace inference
