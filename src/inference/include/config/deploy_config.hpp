#pragma once

#include "robot/robot_config.hpp"

#include <string>

namespace inference {

struct DeployConfigLoadOptions {
    /* 相对路径（policy.model_path / recorder.directory）的解析基准。
       缺省使用编译宏 ROBOT_INFERENCE_ROOT_DIR（src/inference）。 */
    std::string root_dir;
};

/* 从 deploy.yaml 加载并完整校验机器人配置（唯一配置入口）。

   加载期一次性完成：解析 → 全键必填/未知键检查 → 模型序→电机序转换
   → 类型/数值检查。任何失败都通过 error 返回描述，不做回退。

   顺序约定：
   - YAML 中按 DOF 维度的参数一律按模型 DOF 顺序书写；
   - motor.kp/kd 仅写 8 个直驱项（模型 DOF 0..7），加载器经
     joint_ids_map 转为物理电机顺序，4 个脚踝电机槽位取
     ankle.torque.virtual_kp[0] / virtual_kd[0]（两轴当前等值）；
   - motor_to_model_direction 例外：按物理电机顺序书写，因为脚踝
     上下两电机的方向无法用 12 项模型序表达。 */
bool load_deploy_config(const std::string& yaml_path,
                        RobotInterfaceConfig& config,
                        std::string& error);

bool load_deploy_config(const std::string& yaml_path,
                        const DeployConfigLoadOptions& options,
                        RobotInterfaceConfig& config,
                        std::string& error);

}  // namespace inference
