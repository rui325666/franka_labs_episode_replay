# Tower of Babel: LABS 实机回放

数据集：`data/datasets/lerobot/tower_of_babel_20260915_174755`。
episode 0，2284 帧、20 Hz，原速动作时间 **114.2 秒**。双臂均有运动；左夹爪参与抓取，右夹爪在录制中始终打开。

## 运行

现已支持数字选择入口：在项目根目录运行 `bash replay.sh`，输入 `1`、`2` 分别回放新增的 19:52、19:25 轨迹，输入 `3` 回放 19:18 轨迹，输入 `4` 回放本文的旧轨迹。详见 [REPLAY_SELECT.md](REPLAY_SELECT.md)。

两台机械臂启用 FCI，LABS 机械臂、夹爪和 coordinator 服务正常。停止 LABS 页面中的遥操作，避免两个程序同时向 follower 话题发送目标。按录制第 0 帧恢复塔座、圆环与机器人/桌子的相对位置。数据集未记录底盘或升降柱，回放程序不会定位它们。

录制起点参考图：[start_head.png](../replay/tower_of_babel_20260915_174755/start_head.png)。

```bash
cd ~/franka_labs_episode_replay
# 只检查：不创建指令发布器，不切换控制器。
bash data/datasets/tools/replay_tower.sh check

# 完整实机回放：夹爪激活/开合验证 → 双臂归位 → 双臂与夹爪原速同步回放。
bash data/datasets/tools/replay_tower.sh run

# 单独恢复和验证两只夹爪，双臂保持停用：
bash data/datasets/tools/replay_tower.sh gripper-check

# 如只想初始化位置、到位后退出，可单独使用：
bash data/datasets/tools/replay_tower.sh home
```

`run` 自动完成以下步骤，初始化时间另计：

1. 读取双臂、夹爪和控制器状态，检查是否存在其他指令发布器。
2. 双臂停用时，依次调用两只夹爪的原生重新激活服务，硬件会执行自动校准。随后各自执行打开 → 80% 开度 → 打开，实际角度必须跟随指令并稳定 0.3 秒。夹爪应为空；验证失败会在启动双臂之前退出。
3. 保持当前实测姿态并激活双臂，再依次把左右臂的控制目标缓慢移至录制第 0 帧的 **action**，最大目标关节速度 0.15 rad/s。用录制第 0 帧的 **observation.state** 检查实际到位情况，每臂持续 0.5 秒保持在 0.05 rad 误差内后通过。
4. 保持相同关节指令约 1 秒，设置初始夹爪开度；随后原速同步回放全部 16 维动作，持续 114.2 秒。
5. 回放完成后保持目标、停用双臂控制器并核验停用状态。

`home` 只执行双臂初始化，到位后停用控制器并退出，不发送夹爪指令。`check` 只读检查通过不表示机器人已经在起点，也不证明夹爪能够实际运动。

跟随控制器是柔顺力矩控制，目标角与实测角可能不同。例如右臂第 7 轴的录制起点：action 为 0.760273 rad，实测 state 为 0.728544 rad。初始化直接发送 state 会丢失这段已记录的目标/实测差异。初始化日志现在逐轴显示误差，且保存 `home_*.csv`，失败时也保存；没有修改控制器增益或放宽 0.05 rad 到位门槛。

若保持录制目标 10 秒后仍有个别关节未到位，程序会尝试一次有限修正：只修正超差关节，目标变化速度 0.01 rad/s、相对录制指令偏移最多 0.06 rad、最长 8 秒。实际进入 0.035 rad 误差范围后，缓慢恢复**原始录制指令**，重新按 0.05 rad 门槛检查。修正期间无实际响应、超过范围或恢复后仍未到位都会退出。该步骤用于处理不同接近方向造成的静态偏差，不给回放轨迹叠加常量偏移。

Ctrl+C 请求停止；程序保持指令流，先请求停用控制器并核验实际状态，然后退出。如果服务故障使停止状态无法确认，它会明确报 `stop NOT VERIFIED` 并保持目标，需操作员用现场物理停止装置处理。不要同时在 LABS 页面重新启动遥操作。

## 数据与实现

- 原始数据保持不变。回放包在 `data/datasets/replay/tower_of_babel_20260915_174755/`，包含 `episode.npz`、`manifest.json`、`home_pose.json`、夹爪验证报告 `gripper_check_*.json`、归位记录 `home_*.csv` 和每次回放的 `trace_*.csv`。
- `prepare_replay.py` 从 parquet 原样取出 16 维 action；左右臂列分别为 `[0:7]`、`[8:15]`，两个夹爪列为 `7`、`15`，1 表示打开。起点取自 observation.state 的 `[0:7]` 和 `[28:35]`；录制夹爪实测角度取自列 `14`、`42`。
- `replay_labs.py` 通过当前 LABS 的 `/left|right/follower/gello/joint_states` 和对应夹爪话题发送；使用 `controller_coordinator` 管理 `joint_follower_controller`。这不是 Camelo 旧的 PTP/阻抗控制器入口。
- 原始动作时间间隔 0.05 秒；关节在相邻样本间线性插值，指令流约 100 Hz；夹爪保持前一条命令直到下一录制时间点。默认速度 1 倍，可显式用 `--speed 0.5` 半速回放。
- 实测反馈超过 0.2 秒未更新、跟随误差超过门槛、控制器退出 FOLLOWING、指令/调度间隔超过 0.15 秒或出现其他指令发布器，都会结束回放并走停止流程。本文旧轨迹的跟随误差门槛仍为 0.35 rad；新轨迹按关节、时刻参考录制跟随误差，具体规则见 [REPLAY_SELECT.md](REPLAY_SELECT.md)。这些是运行保护，不代表碰撞规划或任务成功验证。
- 回放还检查夹爪是否执行动作：持续收合指令下仍保持全开，或持续全开指令下仍明显闭合，超过 1.5 秒会停止。抓住物体后未达到完全闭合角度属于正常接触，不直接判定为故障。
- 终端每 5 秒显示进度、双臂最大跟随误差、左右夹爪指令和实测角度。`trace_*.csv` 同时保存夹爪指令、当前实测角度和同一录制时刻的参考角度，可用于核对执行效果。
- 默认使用现有 Humble coordinator 镜像和当前 LABS DDS 配置，不修改机器人增益或现有容器。

如需重新导出回放包：`bash data/datasets/tools/replay_tower.sh prepare`。查看数据摘要：`bash data/datasets/tools/replay_tower.sh inspect`。

## 发行版验证范围

原工作站已进行实机回放，本文控制行为来自该实现。本轻量发行版不携带原站点历史日志；接收方首次实机运行应以自己的设备反馈和现场观察为准。

运行 `bash scripts/test.sh` 可执行全部 11 项单元/ROS 集成测试。测试使用禁止网络的容器、ROS domain 93 和虚拟控制器，覆盖归位、跟随限幅、夹爪动作、控制器停用及指令发布冲突。测试通过不等于另一套实物机构已完成验证。

数据重新准备需要完整 LeRobot 数据和额外 dataset-builder 环境；仓库所附回放包无需重新 prepare 即可使用。
