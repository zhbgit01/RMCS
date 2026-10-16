# C 车 yaw 直接力矩扫频

研究模型：`J * 角加速度 + B * 角速度 = 实际反馈力矩`。

控制流程只有：`正弦力矩扫频 → 限幅 → yaw 电机 → 记录反馈力矩和角度`。

- yaw 无角度 PID、速度 PID、前馈、保持或自动回中。
- 力矩幅值 ±0.8 N·m，频率按对数规律从 0.5 Hz 增至 1.5 Hz，持续 60 s。
- 已删除短脉冲波形、训练/验证分段、stage/packet 日志列，以及 20° 角度退出。角度可以超过 20°，需自行确认实际可转范围和线缆余量；本程序不限制累计转角。
- 保留 ±0.8 N·m 力矩限幅、编码器速度 90°/s 保护、非有限反馈锁停、离挡/双下/遥控失联停止。锁停必须双下复位。
- 底盘始终失能（轮零力矩、悬挂停止控制），摩擦轮和拨弹始终失能；pitch 仍由原控制器控制。失能不等于机械锁住，停止驱动也不等于主动刹停。

DR16：双下启动 → 双中准备 → 左保持中、右拨上，开始一次扫频。保持左中右上运行；离开该挡位停止。正常 60 s 后停止驱动、不重复。再次运行需回双中再右上。没有前置 2 s 等待段，也没有分段或自动回中。

主日志 `/tmp/swept_frequency_controller__gimbal_yaw_sweep_*.csv` 为名义 1 kHz，6 列：

`update_count, elapsed_s, /gimbal/yaw/control_torque, /gimbal/yaw/torque, /gimbal/yaw/velocity, /gimbal/yaw/angle`

时间 s、力矩 N·m、角速度 rad/s、角度 rad。正常结束的控制力矩为 NaN，表示停止驱动。角度拟合前需检查并展开绕回。

监控日志 `/tmp/yaw_sweep_monitor_<timestamp>.csv` 名义 200 Hz，包含 active/fault、编码器角度/速度、云台与底盘 IMU 速度、指令/反馈力矩，便于判断退出原因。日志不包含原生硬件采样时间戳。

收到数据后先检查反馈、运动和时序，再以实际反馈力矩为输入拟合 J、B，用未参与拟合的数据验证。当前不预设训练/验证波形；有数值参数不代表验证通过。

按目标架构编译并同步后，双下选择配置并重启：

```bash
ssh-remote "set-robot deformable-infantry-omni-c"
attach-remote -r
# 测完双下，再停止程序、查找并下载本次日志：
ssh-remote "service rmcs stop"
ssh-remote "ls -lt /tmp/swept_frequency_controller_*_sweep_*.csv /tmp/yaw_sweep_monitor_*.csv"
scp remote:/tmp/具体文件名.csv /workspaces/RMCS/
```

本地检查：

```bash
bash .script/build-rmcs --packages-select rmcs_core rmcs_bringup
ctest --test-dir rmcs_ws/build/rmcs_core -R '^direct_yaw_sweep$' --output-on-failure
```

模拟检查覆盖全部 16 种开关状态底盘失能、yaw 输出唯一、pitch 保留、触发/取消/到时/失联、超过 20°仍继续扫频、速度/非有限反馈锁停及复位。未在车上执行。
