#pragma once

#include "robot/robot_config.hpp"

#include <string>

namespace inference {

struct DeployConfigLoadOptions {
    /* 相对路径（policy.model_path / recorder.directory）的解析基准。
       缺省使用编译宏 ROBOT_INFERENCE_ROOT_DIR（src/inference）。 */
    std::string root_dir;
};

/* 从真实 deploy.yaml 格式加载并完整校验机器人配置（唯一配置入口）。

   加载期一次性完成：解析 → 必填/未知键检查 → 模型序/电机序
   边界校验 → 类型/数值检查。任何失败都通过 error 返回描述。

   顺序约定：
   - default_joint_pos / actions.JointPositionAction / observations 中
     按 DOF 维度的参数按模型 DOF 顺序书写；
   - stiffness / damping 按模型 DOF 顺序书写，加载器根据
     joint_ids_map 转换为物理电机顺序的 MIT 增益；
   - joint_ids_map 只用于生成模型 DOF → 物理电机下标的拓扑映射。 */
bool load_deploy_config(const std::string& yaml_path,
                        RobotInterfaceConfig& config,
                        std::string& error);

bool load_deploy_config(const std::string& yaml_path,
                        const DeployConfigLoadOptions& options,
                        RobotInterfaceConfig& config,
                        std::string& error);

}  // namespace inference
