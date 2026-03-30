#include "myactua/MotorTypes.hpp"
#include <iostream>
#include <iomanip>

using namespace myactua;

int main() {
    std::cout << "=== 电机状态字判断示例 ===" << std::endl;
    uint16_t test_cases[] = {
        0x0040, 
        0x0021, 
        0x0023, 
        0x0027, 
        0x0008, 
        0x0427  
    };
    
    for (uint16_t sw : test_cases) {
        std::cout << "\n状态字: 0x" << std::hex << std::setw(4) << std::setfill('0') << sw << std::dec << std::endl;
        std::cout << "  状态机检测:" << std::endl;
        std::cout << "    isFault: " << (isFault(sw) ? "是" : "否") << std::endl;
        std::cout << "    isSwitchOnDisabled: " << (isSwitchOnDisabled(sw) ? "是" : "否") << std::endl;
        std::cout << "    isReadyToSwitchOn: " << (isReadyToSwitchOn(sw) ? "是" : "否") << std::endl;
        std::cout << "    isSwitchedOn: " << (isSwitchedOn(sw) ? "是" : "否") << std::endl;
        std::cout << "    isOperationEnabled: " << (isOperationEnabled(sw) ? "是" : "否") << std::endl;
        
        std::cout << "  各位检测:" << std::endl;
        std::cout << "    Bit0 (Ready to switch on): " << (isReadyToSwitchOnBit(sw) ? "1" : "0") << std::endl;
        std::cout << "    Bit1 (Switched on): " << (isSwitchedOnBit(sw) ? "1" : "0") << std::endl;
        std::cout << "    Bit2 (Operation enabled): " << (isOperationEnabledBit(sw) ? "1" : "0") << std::endl;
        std::cout << "    Bit3 (Fault): " << (isFaultBit(sw) ? "1" : "0") << std::endl;
        std::cout << "    Bit4 (Voltage enabled): " << (isVoltageEnabledBit(sw) ? "1" : "0") << std::endl;
        std::cout << "    Bit5 (Quick stop): " << (isQuickStopBit(sw) ? "1" : "0") << std::endl;
        std::cout << "    Bit6 (Switch on disabled): " << (isSwitchOnDisabledBit(sw) ? "1" : "0") << std::endl;
        std::cout << "    Bit7 (Warning): " << (isWarningBit(sw) ? "1" : "0") << std::endl;
        std::cout << "    Bit9 (Remote): " << (isRemoteBit(sw) ? "1" : "0") << std::endl;
        std::cout << "    Bit10 (Target reached): " << (isTargetReachedBit(sw) ? "1" : "0") << std::endl;
        std::cout << "    Bit11 (Internal limit active): " << (isInternalLimitActiveBit(sw) ? "1" : "0") << std::endl;
        
        std::cout << "  建议控制字: 0x" << std::hex << std::setw(4) << std::setfill('0') << getNextControlWord(sw) << std::dec << std::endl;
    }
    
    std::cout << "\n=== 完成 ===" << std::endl;
    return 0;
}
