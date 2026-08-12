#pragma once

#include "ethercat_types.hpp"

namespace motor_base {
struct RtEvent;
}

namespace myactua{

class EthercatAdapter
{
public:
    using RtEventSink = void (*)(void* context, const motor_base::RtEvent& event);

    virtual ~EthercatAdapter() = default;
    virtual bool init(const char* ifname) = 0;  // 初始化网口和ethercat网络
    virtual void set_event_sink(void* context, RtEventSink sink) {
        (void)context;
        (void)sink;
    }

    // 物理层收发：整个网络执行一次
    virtual void receive_physical() = 0;
    virtual void send_physical() = 0;

    // 逻辑层读写：每个电机对象调用一次（仅操作内存）
    virtual void send(int index, const TxPDO& pdo) = 0;
    virtual RxPDO receive(int index) = 0;
    virtual bool is_configured(int index) = 0;  // 检查电机是否已经配置完成
};

}
