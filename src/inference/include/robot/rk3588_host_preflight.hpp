#pragma once

#include <string>

namespace inference {

struct Rk3588HostPaths {
    std::string proc_root{"/proc"};
    std::string sys_root{"/sys"};
    std::string run_root{"/run"};
    std::string dev_root{"/dev"};
    std::string etc_root{"/etc"};
    std::string ethercat_device_id{"fe1c0000.ethernet"};
    std::string ethercat_driver{"rk_gmac-dwmac-ethercat"};
    std::string profile{"rk3588-rt"};
};

bool verify_rk3588_host_layout(std::string& error,
                               const Rk3588HostPaths& paths = {});

}  // namespace inference
