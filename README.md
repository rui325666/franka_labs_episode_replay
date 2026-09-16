# Franka LABS Episode Replay

**双 Franka FR3 + Robotiq 夹爪的轨迹回放与场景实时叠图工具。**

包含中文操作说明、四条已录制轨迹的轻量回放包、双臂归位和夹爪同步动作、`cali` 参考图对齐，以及底层服务的启动和离线镜像搬运脚本。

- [完整中文操作说明](REPLAY_README_ZH.md)
- [国内网络与离线镜像部署](docs/OFFLINE_ZH.md)
- [轨迹编号及数据说明](data/datasets/tools/REPLAY_SELECT.md)
- [控制行为与停止机制](data/datasets/tools/REPLAY_TOWER.md)

## 1. 下载后还需要什么

**源码压缩包不包含 Docker 镜像。** 本仓库里的 `registry.localhost/labs/...:latest` 是原工作站使用的本地镜像标签，不是可以公开 `docker pull` 的镜像服务；配置镜像加速也不能凭空取得它们。

准备下列环境：

- Linux **x86_64/amd64** 工作站。当前验证来自 Ubuntu 24.04 主机；不直接适用于 Jetson/ARM、Windows Docker Desktop 或 macOS 的实机控制。
- Docker Engine、Docker Compose 插件、Bash、Python 3；`git` 用于克隆，或直接下载 GitHub ZIP 解压。
- 两台 Franka FR3 的 FCI 网络连接、两只 Robotiq 夹爪 USB/RS-485 连接。相机仅在使用 `cali` 时需要。
- 下表所列运行镜像。可使用离线包导入，也可在联网机器上从源码构建。
- 与录制匹配的机器人安装、负载、底盘/升降柱及桌面物体摆放。本工具没有自动碰撞规划或视觉纠偏。

| 镜像 | 用途 |
| --- | --- |
| `registry.localhost/labs/franka-robot:latest` | 双臂驱动/控制器 |
| `registry.localhost/labs/robotiq-gripper:latest` | 双夹爪驱动 |
| `registry.localhost/labs/controller-coordinator:latest` | 控制协调器及回放 Python 运行环境 |
| `registry.localhost/labs/zed-camera:latest` | 头部相机 CPU 采集 |
| `registry.localhost/labs/replay-gui:latest` | ROS + Python + Qt 实时叠图环境 |

镜像名中的 `registry.localhost` 不要求你搭建本地 registry。`docker load` 导入后，Docker 直接按这些标签使用本地镜像；运行入口禁止自动拉取镜像。

## 2. 首次安装

### 下载代码

```bash
cd ~
git clone https://github.com/rui325666/franka_labs_episode_replay.git
cd franka_labs_episode_replay
```

也可以在 GitHub 点击 **Code → Download ZIP**，解压后进入目录。脚本按仓库实际路径定位文件，无需创建 `/home/ebim/labs`。

### 准备镜像：选择一种方法

**方法 A：导入维护者提供的离线镜像包，适合下载依赖不稳定的网络。**

拿到完整镜像目录后运行（替换为自己的目录）：

```bash
bash scripts/images.sh import /path/to/franka-replay-images
bash scripts/images.sh list
```

导入前会先校验所有归档的 SHA256，导入后核对镜像 ID。只有仓库 ZIP、没有离线镜像目录时，这一步还不能完成；请取得单独传输的镜像包，或使用方法 B。镜像包内含开源第三方运行组件，不含模型权重。

**方法 B：在可以访问依赖源的网络中构建。**

```bash
bash scripts/build_images.sh all
```

构建会访问 Docker Hub、GHCR、GitHub、Ubuntu/ROS 软件源等。若遇到网络超时，见 [国内部署说明](docs/OFFLINE_ZH.md)。完整构建可能耗时较长，且上游包源状态会影响结果；原站点已有镜像可直接导出，能更准确复用已验证环境。

底层服务源码及构建上下文已附带，部分 Dockerfile 仍会在构建时下载上游依赖。`--pull=false` 不代表离线构建：本地缺少基础镜像时仍需要下载，`apt` 和 `git clone` 也需要网络。

### 修改自己机器的硬件配置

站点目录：[deployments/example_station](deployments/example_station)。随附配置记录的是原工作站数值，**新工作站必须先核对并修改**：

| 文件 | 必查项目 |
| --- | --- |
| `config_franka_robot.yml` | 左右 `robot_ip`、硬件型号、控制器参数、负载适配 |
| `config_robotiq_gripper.yml` | `ls -l /dev/serial/by-id/` 获取自己的左右串口，不能照抄序列号 |
| `config_zed_camera.yml` | 自己相机的 `expected_serial_number`、分辨率、选取左右眼 |
| `cyclonedds.xml` | `NetworkInterface` 的 `address` 改成这台电脑机器人网卡上的 IPv4 地址，原值为 `172.16.16.118` |
| `config_controller_coordinator.yml` | 左右命名空间和实际控制器名称匹配 |

查看本机地址：`ip -br addr`。机械臂 IP 与本机 DDS 网卡地址是不同配置，不能混为同一个值。

控制器镜像基于原站点的 Franka/ROS 驱动版本，必须与本地机器人系统、FCI 和 libfranka 兼容。不要通过绕过反馈/状态检查来适配另一套硬件。更换机器人型号或机构布局需要重新适配、录制与验证；不能仅凭相同 IP 直接播放。

## 3. 日常使用

在两台机械臂 Desk 中启用 FCI，连接并供电夹爪和相机。在其他控制程序中结束遥操作后：

```bash
# 启动双臂、夹爪、协调器和头部相机，并进行只读反馈检查。
bash replay_services.sh start

# 实时叠图：对齐桌面、塔座及芯片，完成后关闭窗口。
bash replay.sh cali

# 打开菜单，输入轨迹编号。
bash replay.sh
```

| 输入 | 轨迹/功能 |
| --- | --- |
| `1` | 19:52 录制，默认半速，动作时间 111.2 秒 |
| `2` | 19:25 录制，原速 70.1 秒 |
| `3` | 19:18 录制，原速 87.4 秒 |
| `4` | 17:47 录制，原速 114.2 秒 |
| `cali` | 实时图 / 轨迹 3 起点参考图 / 透明叠加图 |
| `start` | 启动底层服务并检查 |
| `status` | 查看底层服务容器状态 |
| `q` | 退出 |

四段均录于 2026-09-15（Europe/Berlin）。输入数字会产生双臂和夹爪运动，先清空夹爪并恢复场景；初始化时间另计。`cali` 本身仅订阅相机，不控制硬件。

直接回放和只读检查：

```bash
bash replay.sh 3 inspect   # 离线摘要和文件校验，仅需 Python 标准库
bash replay.sh 3 check     # 实际硬件状态检查，不发送运动指令
bash replay.sh 3           # 初始化 + 双臂和夹爪回放
```

`check` 通过但显示 `ready at starting pose: False`，表示尚未归位；`run` 会自动归位。容器显示 `Up` 不能代替实际反馈检查。

### `cali` 校准

下载版在 `replay-gui` 容器内运行，无需安装宿主机 ROS。需要 Linux 桌面 `DISPLAY` 和当前用户的 Xauthority；不需要 `xhost +`。无桌面或诊断时：

```bash
bash replay.sh cali --headless --seconds 10
```

默认参考图：[轨迹 3 第一帧](data/datasets/replay/tower_of_babel_20260915_191802/start_head.png)。窗口每秒刷新，拖动 **Live opacity** 调透明度，按 `Q` / `Esc` 退出。自定义 `--ref` / `--out` 使用仓库内的路径，以便容器访问。

## 4. 停止及故障排查

在回放终端按 `Ctrl+C`，等待 `Both arm controllers verified inactive.`，随后才停止底层服务：

```bash
bash replay_services.sh stop
```

若出现 `stop NOT VERIFIED`，按现场物理停止流程处理。服务停止命令不能替代运动中的紧急停止装置。

```bash
bash replay_services.sh status
bash replay_services.sh check
bash replay_services.sh logs franka-robot
bash replay_services.sh logs robotiq-gripper
bash replay_services.sh logs zed-camera-head
```

`no fresh joint feedback` 常见于底层驱动/FCI 未就绪；`no fresh gripper feedback` 先检查串口和供电；`Other command publishers exist` 应先结束另一个遥操作或运动程序。详细处理见 [中文操作说明第 8 节](REPLAY_README_ZH.md#8-常见故障排查)。

## 5. 数据范围、测试与来源

- 随附四个 `episode.npz`、清单、起点关节与参考照片，完整包含四条回放所需的关节/夹爪动作和反馈参考。
- 不包含完整 LeRobot 视频、原始 MCAP、采集数据库及原工作站历史运行日志。运行时新日志保存在各回放目录，Git 默认忽略。
- `prepare` 和数据导出代码是进阶入口，需要另外提供完整数据和数据处理环境；日常回放无需执行。
- 核心控制逻辑沿用原 LABS 工作站的实现；新站点仍须验证自己的机构、负载和场景。原站点成功不能证明另一台机器也能安全复现。
- 来源、依赖版本和许可证见 [NOTICE](NOTICE)、[LICENSE](LICENSE) 及各第三方目录的许可证。

隔离测试（虚拟控制器，无实机网络）：

```bash
bash scripts/test.sh
```

完整中文说明：[REPLAY_README_ZH.md](REPLAY_README_ZH.md)。发行包测试范围：[VALIDATION.md](docs/VALIDATION.md)。
