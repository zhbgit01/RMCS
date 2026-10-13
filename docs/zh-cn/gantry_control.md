# 最简龙门架控制

## 设备与操作

CAN1：ID 1 横向 Yaw 丝杆，ID 2 左升降，ID 3 右升降。三台 M2006 使用默认 36:1 减速比，两侧同方向，假设两侧丝杆导程和额外传动比相同。不是发射架真实角度控制。

使用左摇杆：上下更新共同升降目标，左右更新横向目标，可以同时操作。右拨杆上挡使能；中/下挡关闭输出（不是位置保持）。启动先把摇杆回中、右拨杆放中/下挡，等三台反馈有效，再拨上挡。开机已经在上挡不会运动。回中保持目标位置。使用 SDK 的 CBoard 接口（USB VID:PID 为 a11c:d401），需要已刷兼容 RMCS 固件的 C 板，通过 USB 接入上位机，DR16 接 DBUS。

## 文件与数据流

- `hardware/gantry.cpp`：复用 DjiMotor 和 Dr16，CAN1 收取反馈、DBUS 接收遥控，发送伙伴组件在控制计算后发送 0x200。接收时间使用原子变量，DR16 包通过互斥锁交给执行器解析，不使用会关闭 DR16 超时的 RemoteControl 仲裁。
- `controller/gantry/gantry_logic.hpp`：不依赖 ROS/USB 的目标积分、同步修正、使能状态机；位置外环用比例控制。
- `controller/gantry/gantry_controller.cpp`：绑定 RMCS 接口，使用实际时间差，复用 PidCalculator 做三个速度内环。第一版速度环只启用 P，停机复位 PID。
- `rmcs_bringup/config/gantry.yaml`：仅新增龙门架配置，不修改其他机器人。

数据流：电机反馈 → 相对位置与目标误差 → 位置比例控制和左右同步修正 → 目标角速度 → 速度 PID → 力矩限制 → CAN1。

启用时记录各电机位置作为参考；共同升降目标从零开始，无自动回零。同步误差是两侧从参考位置开始的位移差。左侧走得多，则减少左侧速度并增加右侧速度。目标跟踪误差限制为 0.2 rad，防止卡住时目标持续积累；不是行程限位。

## 参数与保护

位置、速度是电机减速箱输出轴 rad、rad/s，力矩 N·m。默认目标速度 0.3 rad/s，最高速度 0.5 rad/s，输出限幅 0.15 N·m，均为待实物调节的初始值。左右同步阈值 0.3 rad，不代表实际允许高度差；须按导程换算并调整。上下/左右符号通过 pitch_sign/yaw_sign（仅允许 ±1）调整。现有 PID 的 I/D 按调用次数计算，此版本为 P 控制。

三台反馈缺失或超过 100 ms、DR16 超过 500 ms、非有限输入、周期超过 50 ms、同步差超过阈值，均关闭控制。恢复需要健康反馈、摇杆回中、拨杆先关闭再开启。重新使能重新建立当前位置参考，因此人工先确认两侧齐平，不能通过反复使能忽略歪斜。

此版本没有绝对行程限位、自动回零、丝杆卡滞识别、实际 Pitch 反馈或机械耦合补偿。断力矩可能下滑，必须实物确认自锁/制动。仅保持升降目标，不能证明单独 Yaw 时实际 Pitch 恒定。编码器数据更新不足时也不能可靠累计多圈。

## 编译、测试和运行

在仓库根目录：

```bash
bash .script/build-rmcs --packages-up-to rmcs_core rmcs_bringup
c++ -std=c++23 -Wall -Wextra -Werror -I rmcs_ws/src/rmcs_core/src rmcs_ws/src/rmcs_core/test/gantry_logic_test.cpp -o /tmp/gantry_logic_test
/tmp/gantry_logic_test /tmp/gantry.csv
```

测试覆盖启动互锁、共同升降目标、Yaw 软件解耦、回中保持、同步方向、歪斜停机、失联和异常反馈、恢复互锁、异常周期、参数验证，以及右侧响应较慢的简化动力学模型。CSV 可用表格工具查看；模型不代表实际机构，测试不需要硬件，也不会连接 USB。不要用 -DNDEBUG 编译此 assert 测试。

有硬件后：先确认固件/USB、CAN ID、DR16 数据和同方向，支撑机构确认不会下滑；短时间低速分别验证三个电机，再调速度环和升降同步，最后验证单独 Yaw 时真实 Pitch。没有行程限位，需要操作者观察，远离丝杆端部。

```bash
set-robot gantry
# 新开终端，让环境更新后：
launch-rmcs
```

如果脚本不可用，可 source ROS 与 rmcs_ws/install/setup.bash 后执行：

```bash
ros2 launch rmcs_bringup rmcs.launch.py robot:=gantry
```

## build-rmcs 保护模式

默认每次仅编译一个包，包内一个任务；MAKEFLAGS 和 CMAKE_BUILD_PARALLEL_LEVEL 都受限制。增加包内任务：

```bash
RMCS_BUILD_JOBS=2 build-rmcs
```

只接受 1–64 的整数。拒绝 --executor/--parallel-workers 覆盖，避免意外叠加并行。降低并行只降低内存峰值，不能保证单个编译任务不会耗尽内存。未修改交叉编译脚本。
