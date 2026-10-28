# ONNX Runtime CPU 部署

策略运行时只使用 ONNX Runtime CPU 1.30.0，不需要 LibTorch、Python/PyTorch、
CUDA 或模型后端选择参数。Python/PyTorch 仍用于独立的 `chirp_test` 数据导出，
与策略部署无关。

从仓库根目录构建与验证：

```bash
cmake -S src/inference -B src/inference/build-ort-cpu -DCMAKE_BUILD_TYPE=Release
cmake --build src/inference/build-ort-cpu -j2
ctest --test-dir src/inference/build-ort-cpu --output-on-failure
python3 src/inference/src/recorder/plot_motor_tracking_test.py
```

本地依赖为 `third_party/onnxruntime-1.30.0` 的 Linux AArch64 CPU SDK、
`third_party/yaml-cpp` 和电机模块使用的 IgH EtherCAT SDK。第三方目录不受 Git
跟踪，干净 checkout 需自行准备。ORT SDK 路径可通过 `-DONNXRUNTIME_ROOT=...`
指定；版本和文件检查保持启用。yaml-cpp 命令行工具默认不构建。

模型默认路径相对于 `src/inference` 解析为 `model/policy.onnx`，不依赖启动时的
工作目录。加载器读取实际节点名称；当前模型是 `obs: float32[1,675]` →
`actions: float32[1,12]`。模型必须恰好有一个输入和一个输出。配置中启用
`observations.gait_phase` 时要求 705 维输入，需同时提供对应模型；当前自带模型
不支持直接打开此选项。允许模型元信息中的 batch 维动态，实际运行始终使用 batch 1。

观测字段顺序为角速度、重力投影、速度指令、可选步态、相对关节位置、关节速度、
历史动作；每个字段分别堆叠 15 帧，按从旧到新展平。首次观测填满全部历史帧。
只有准入并发布的策略输出才提交到历史动作，历史动作仍裁剪到 `[-1,1]`。
输出经过既有 raw action 裁剪、逐关节缩放、偏移限位及站立姿态叠加后交给命令线程。

`PolicyRuntime::load` 在电机/IMU 启动之前创建会话、校验端口、预分配 float
缓冲区和 Tensor，并执行 20 次预热。默认 intra-op=1、顺序执行、完整图优化，
关闭线程池 spinning。双线程实验仍检查指定 worker 的 CPU 和调度策略；初始化
阶段最多等待 100 ms，让异步创建的 worker 完成亲和性设置，失败仍拒绝启动。
`infer` 复用缓冲区，保留输入/输出有限性检查及推理时间戳；输出类型和形状由
加载校验及 ORT 对预分配输出的检查保证。`shutdown` 和析构通过 RAII 释放全部资源。
同一运行时由所有者线程串行访问。

`policy_runtime_test` 覆盖模型契约、加载/卸载/重载、异常输入输出、动态输出超限，
用独立 ORT 路径对比 100 组真实模型输出，并在有至少两个可用 CPU 时连续验证
30 次双线程加载。`observation_builder_test` 覆盖 675/705 两种布局、历史移位、
步态门控、动作提交/跳过和重置。错误模型夹具全部位于 `tests/fixtures`，总计约
3 KiB；它们不是机器人策略。仅在需要重新生成夹具时运行
`python3 src/inference/tests/make_policy_fixtures.py`，生成器只依赖 Python 标准库，
编码依据 [ONNX protobuf schema](https://github.com/onnx/onnx/blob/main/onnx/onnx.proto)。

硬件运行入口为 `build-ort-cpu/policy_test [deploy.yaml]`，会启动真实电机。
板端 CPU 布局、权限、内存锁定和长时间验收要求见
[RK3588 部署说明](../../deploy/rk3588/README.md)。20 ms 策略周期、1 ms 命令周期、
40 ms 结果准入及 60/10 ms 超时保持原值。

本次审计、文件变更、实测数据和未验证事项见
[瘦身验证记录](../../doc/review/onnx_cpu_slimming_2026-10-28.md)。
后续单次调用与纯转发函数的清理见
[一次性封装清理记录](../../doc/review/oneoff_wrapper_cleanup_2026-10-28.md)。
以 `policy_test` 为入口的初始化、函数调用、数据结构及跨线程链路见
[完整运行链路说明](../../doc/policy_test_call_and_data_flow.md)。
