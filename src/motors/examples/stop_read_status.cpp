#include "EthercatAdapterIGH.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

const char* mode_to_string(myactua::ControlMode mode)
{
    switch (mode) {
        case myactua::ControlMode::NONE:
            return "NONE";
        case myactua::ControlMode::CSP:
            return "CSP";
        case myactua::ControlMode::CSV:
            return "CSV";
        case myactua::ControlMode::CST:
            return "CST";
        default:
            return "UNKNOWN";
    }
}

const char* run_state_to_string(bool stop_mode)
{
    return stop_mode ? "STOP" : "RESTART";
}

bool wait_all_slaves_ready(const std::shared_ptr<myactua::EthercatAdapterIGH>& adapter,
                           int num_motors,
                           int timeout_ms = 20000,
                           int poll_ms = 100)
{
    const auto start_time = std::chrono::steady_clock::now();
    const auto deadline = start_time + std::chrono::milliseconds(timeout_ms);
    auto next_log_time = start_time;

    while (std::chrono::steady_clock::now() < deadline) {
        adapter->receivePhysical();
        adapter->sendPhysical();

        int ready_count = 0;
        for (int i = 0; i < num_motors; ++i) {
            if (adapter->isConfigured(i)) {
                ++ready_count;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= next_log_time) {
            std::cout << "[连接检查] EtherCAT ready: "
                      << ready_count << "/" << num_motors << std::endl;
            next_log_time = now + std::chrono::milliseconds(poll_ms);
        }
        if (ready_count == num_motors) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return false;
}

myactua::ControlWordCommand get_next_control_word(uint16_t status_word)
{
    if (!myactua::is_switched_on(status_word) && !myactua::is_ready_to_switch_on(status_word)) {
        return myactua::CMD_SHUTDOWN;
    }
    if (!myactua::is_switched_on(status_word) && myactua::is_ready_to_switch_on(status_word)) {
        return myactua::CMD_SWITCH_ON;
    }
    if (myactua::is_switched_on(status_word) && !myactua::is_operation_enabled(status_word)) {
        return myactua::CMD_ENABLE_OPERATION;
    }
    return myactua::CMD_ENABLE_OPERATION;
}

class RawStopReadStatusController {
public:
    RawStopReadStatusController(std::shared_ptr<myactua::EthercatAdapterIGH> adapter,
                                int num_motors,
                                int tx_phase_delay_us)
        : _adapter(std::move(adapter)),
          _tx_phase_delay_us(tx_phase_delay_us < 0 ? 0 : tx_phase_delay_us)
    {
        for (int i = 0; i < num_motors; ++i) {
            _motors.emplace_back(i);
        }
    }

    ~RawStopReadStatusController()
    {
        shutdown();
    }

    void start()
    {
        if (_running.load()) {
            return;
        }
        _running = true;
        _rt_thread = std::thread(&RawStopReadStatusController::rt_thread_func, this);
    }

    void shutdown()
    {
        if (!_running.load()) {
            return;
        }
        _running = false;
        if (_rt_thread.joinable()) {
            _rt_thread.join();
        }
    }

    void set_stop_mode()
    {
        _stop_mode = true;
    }

    void set_restart_mode()
    {
        _stop_mode = false;
    }

private:
    struct MotorState {
        int slave_index;
        myactua::TxPDO tx;
        myactua::RxPDO rx;
        bool comm_ok;
        uint32_t comm_offline_total_count;

        explicit MotorState(int index)
            : slave_index(index),
              tx({}),
              rx({}),
              comm_ok(false),
              comm_offline_total_count(0)
        {
            tx.control_word = myactua::CMD_SHUTDOWN;
            tx.target_pos = 0;
            tx.target_vel = 0;
            tx.target_torque = 0;
            tx.max_torque = 5000;
            tx.op_mode = static_cast<int8_t>(myactua::ControlMode::CSV);
            tx.reserved = 0;
        }
    };

    void update_motor_command(MotorState& motor, bool stop_mode)
    {
        const auto rx_mode = static_cast<myactua::ControlMode>(motor.rx.op_mode);
        motor.tx.op_mode = static_cast<int8_t>(
            rx_mode == myactua::ControlMode::NONE ? myactua::ControlMode::CSV : rx_mode);
        motor.tx.max_torque = 5000;
        motor.tx.target_pos = motor.rx.pos;
        motor.tx.target_vel = 0;
        motor.tx.target_torque = 0;

        if (stop_mode) {
            motor.tx.control_word = myactua::CMD_DISABLE_OPERATION;
            return;
        }

        if (myactua::is_fault(motor.rx.status_word) || motor.rx.error != 0) {
            motor.tx.control_word = myactua::CMD_SHUTDOWN;
            return;
        }

        motor.tx.control_word = get_next_control_word(motor.rx.status_word);
    }

    void rt_thread_func()
    {
        auto next_period = std::chrono::steady_clock::now();
        int print_count = 0;

        while (_running.load()) {
            next_period += std::chrono::milliseconds(1);
            const bool stop_mode = _stop_mode.load();

            _adapter->receivePhysical();

            for (std::size_t i = 0; i < _motors.size(); ++i) {
                _motors[i].comm_ok = _adapter->isConfigured(_motors[i].slave_index);
                if (!_motors[i].comm_ok) {
                    ++_motors[i].comm_offline_total_count;
                    continue;
                }

                _motors[i].rx = _adapter->receive(_motors[i].slave_index);
                update_motor_command(_motors[i], stop_mode);
                if (_tx_phase_delay_us > 0) {
                    std::this_thread::sleep_for(std::chrono::microseconds(_tx_phase_delay_us));
                }
                _adapter->send(_motors[i].slave_index, _motors[i].tx);
            }

            _adapter->sendPhysical();

            if (++print_count >= 500) {
                print_count = 0;
                print_motors_info();
            }

            std::this_thread::sleep_until(next_period);
        }
    }

    void print_motors_info() const
    {
        printf("\033[2J\033[H");
        printf("\033[1;36m========================= RAW ETHERCAT MONITOR =========================\033[0m\n");
        printf("RUN_STATE: %s\n", run_state_to_string(_stop_mode.load()));
        printf("%-4s | %-7s | %-11s | %-8s | %-8s | %-8s | %-8s | %-8s | %-11s | %-11s | %-8s\n",
               "ID",
               "COMM_OK",
               "OFFLINE_CNT",
               "RX_MODE",
               "TX_MODE",
               "TX_CW",
               "OP_EN",
               "SW_ON",
               "RDY_SW_ON",
               "STATUS",
               "ERROR");
        printf("--------------------------------------------------------------------------------------------------------------\n");

        for (const auto& motor : _motors) {
            char sw_hex[16] = {};
            char err_hex[16] = {};
            char cw_hex[16] = {};
            std::snprintf(sw_hex, sizeof(sw_hex), "0x%04X", static_cast<unsigned>(motor.rx.status_word));
            std::snprintf(err_hex, sizeof(err_hex), "0x%04X", static_cast<unsigned>(motor.rx.error));
            std::snprintf(cw_hex, sizeof(cw_hex), "0x%04X", static_cast<unsigned>(motor.tx.control_word));

            printf("M%-3d | %-7s | %-11u | %-8s | %-8s | %-8s | %-8s | %-8s | %-11s | %-11s | %-8s\n",
                   motor.slave_index,
                   motor.comm_ok ? "Y" : "N",
                   static_cast<unsigned>(motor.comm_offline_total_count),
                   mode_to_string(static_cast<myactua::ControlMode>(motor.rx.op_mode)),
                   mode_to_string(static_cast<myactua::ControlMode>(motor.tx.op_mode)),
                   cw_hex,
                   myactua::is_operation_enabled(motor.rx.status_word) ? "Y" : "N",
                   myactua::is_switched_on(motor.rx.status_word) ? "Y" : "N",
                   myactua::is_ready_to_switch_on(motor.rx.status_word) ? "Y" : "N",
                   sw_hex,
                   err_hex);
            printf("      pos=%-12d vel=%-12d torque=%-8d\n",
                   static_cast<int>(motor.rx.pos),
                   static_cast<int>(motor.rx.vel),
                   static_cast<int>(motor.rx.torque));
        }

        printf("\033[1;36m=======================================================================\033[0m\n");
        fflush(stdout);
    }

    std::shared_ptr<myactua::EthercatAdapterIGH> _adapter;
    std::vector<MotorState> _motors;
    std::thread _rt_thread;
    std::atomic<bool> _running{false};
    std::atomic<bool> _stop_mode{true};
    int _tx_phase_delay_us{0};
};

}  // namespace

int main(int argc, char** argv)
{
    const std::string ifname = "enp8s0";
    const int num_motors =  2;
    int tx_phase_delay_us = 0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--tx-phase-us") {
            if (i + 1 >= argc) {
                std::cerr << "[错误] --tx-phase-us 缺少参数。" << std::endl;
                return -1;
            }
            char* end = nullptr;
            const long parsed = std::strtol(argv[++i], &end, 10);
            if (end == argv[i] || parsed < 0) {
                std::cerr << "[错误] --tx-phase-us 参数无效: " << argv[i] << std::endl;
                return -1;
            }
            tx_phase_delay_us = static_cast<int>(parsed);
        } else if (arg == "--help") {
            std::cout << "用法: stop_read_status [--tx-phase-us N]\n"
                      << "  --tx-phase-us N   在每次 _adapter->send 前插入 N 微秒延时(仅验证用)\n"
                      << "\n环境变量:\n"
                      << "  MYACTUA_ECAT_DIAG=0/1            开关 EtherCAT 诊断输出\n"
                      << "  MYACTUA_ECAT_DIAG_INTERVAL=500   诊断输出周期(控制周期数)\n";
            return 0;
        } else {
            std::cerr << "[错误] 未知参数: " << arg << std::endl;
            return -1;
        }
    }

    auto adapter = std::make_shared<myactua::EthercatAdapterIGH>();
    RawStopReadStatusController controller(adapter, num_motors, tx_phase_delay_us);
    std::cout << "[参数] tx_phase_delay_us=" << tx_phase_delay_us << std::endl;

    std::cout << "[1/5] 正在连接 EtherCAT..." << std::endl;
    if (!adapter->init(ifname.c_str())) {
        std::cerr << "[错误] 无法连接到 EtherCAT 网络！" << std::endl;
        return -1;
    }

    std::cout << "[2/5] 等待从站就绪..." << std::endl;
    if (!wait_all_slaves_ready(adapter, num_motors)) {
        std::cerr << "[错误] 超时：未等到所有从站就绪。" << std::endl;
        return -1;
    }

    std::cout << "[3/5] 启动实时线程..." << std::endl;
    controller.start();

    std::cout << "[4/5] 下发 STOP(广播到全部电机)..." << std::endl;
    controller.set_stop_mode();
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "[5/5] 下发 RESTART..." << std::endl;
    controller.set_restart_mode();
    std::this_thread::sleep_for(std::chrono::seconds(10));
    controller.shutdown();
    std::cout << "\n[完成] 已通过底层 send/receive 执行 STOP/RESTART 并完成状态打印。" << std::endl;
    return 0;
}
