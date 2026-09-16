# 国内网络与离线镜像部署

## 先区分三件事

1. **Docker 运行镜像**：驱动、ROS、Python 等打包后的环境。这套程序在国内外运行都需要这些镜像。
2. **国内镜像源/加速服务**：帮助下载 Docker 基础镜像、Ubuntu/ROS 软件包或 Python 包的网络服务。是否需要取决于接收方的实际网络，不是程序的固定要求。
3. **离线镜像包**：维护者把已构建镜像保存成文件，接收方复制并导入。对于跨地域交付，能减少首次构建时反复下载依赖的工作。

本仓库没有可以公开拉取的 `registry.localhost` 服务。它只是镜像标签前缀；无论配置什么 Docker Hub 加速器，都不会自动得到原工作站的这些自定义镜像。

## 推荐交付内容

- 仓库源码：GitHub 克隆或 ZIP 下载。
- 镜像目录：五个 `.tar.gz`、`images.json` 和 `SHA256SUMS`，单独传输。
- 接收方自己的硬件配置：机器人 IP、串口、相机序列号和本机 DDS 网卡地址。

仓库已经包含四条轻量回放轨迹和参考图，无需另行下载视频。Docker Engine/Compose 仍需在目标主机上提前安装；本镜像包不包含宿主机操作系统或 Docker 安装包。

## 发送方：导出已验证的镜像

在具有五个本地镜像的 Linux x86_64 工作站，在仓库根目录运行：

```bash
bash scripts/images.sh list
bash scripts/images.sh export "$HOME/franka-replay-images"
```

输出目录必须不存在，防止不同批次混用。导出不会停止现有容器，也不会连接或控制机器人；会占用磁盘与 CPU，建议在没有时间敏感运动任务时执行。

输出目录结构：

```text
franka-replay-images/
├── franka-robot.tar.gz
├── robotiq-gripper.tar.gz
├── controller-coordinator.tar.gz
├── zed-camera.tar.gz
├── replay-gui.tar.gz
├── images.json
└── SHA256SUMS
```

`images.json` 保存镜像标签、ID、架构和大小。SHA256 用于检查文件传输完整性；请从可信维护者处取得镜像包。

把整个目录通过接收方可用的机构文件服务、网盘、移动硬盘或其他渠道传输。归档可能很大，不放入普通 Git 提交。如果要使用 GitHub Release，按 GitHub 当时的单文件大小限制处理，超限文件需要分卷并在接收端还原后再导入。

## 接收方：校验、导入、配置

```bash
cd ~/franka_labs_episode_replay
bash scripts/images.sh import /path/to/franka-replay-images
bash scripts/images.sh list
```

脚本会先检查全部文件的 SHA256 和平台，再调用 `docker image load`，最后核对镜像 ID。校验不通过不会开始导入。

**先按首页修改自己的站点配置**，然后启用 FCI 并运行：

```bash
bash replay_services.sh start
bash replay.sh cali
```

五个镜像导入完成后，日常启动和回放不需要连接 Docker Hub、GitHub 或 Python 包源，只需要与本地机器人和设备通信。应用层运行不代表机器可脱离机器人网络；Docker 的 `--network host` 和 DDS 网卡仍必须配置正确。

导入已有同名标签时会更新本机标签指向。不要在另一套正在执行运动任务的系统上替换镜像；先结束现有运动，再维护运行环境。

## 没有离线包，只能自己构建时

```bash
bash scripts/build_images.sh all
```

构建依赖多个独立来源：

| 来源 | 用途 | 单一 Docker Hub 加速器能否覆盖 |
| --- | --- | --- |
| Docker Hub | ROS/Ubuntu 等基础镜像 | 取决于加速服务支持情况 |
| GHCR | 某些 Dockerfile 使用的构建工具镜像 | 不应假定能覆盖 |
| GitHub | Franka、Robotiq、ZED 等源码 | 不能代替源码访问 |
| Ubuntu/ROS 软件源 | 系统包和 ROS 依赖 | 不能代替 apt 软件源 |

先看错误具体发生在哪一步。Docker `pull` 失败、`git clone` 失败、`apt-get` 失败需要分别处理；不要把所有网络错误归因于同一个源。

本项目不自动替换系统软件源、关闭 TLS 校验或写入未知公共加速站。可以使用机构提供的可信代理、镜像仓库和软件源，或在联网正常的机器上构建后导出。具体可达性需要由国内接收方实测。

## 官方参考

- [Docker image save](https://docs.docker.com/reference/cli/docker/image/save/)：导出镜像及标签，可配合 gzip 压缩。
- [Docker image load](https://docs.docker.com/reference/cli/docker/image/load/)：导入归档并恢复镜像标签。
- [Docker Hub mirror](https://docs.docker.com/docker-hub/image-library/mirror/)：Docker Hub 镜像缓存/镜像服务的范围。

离线运行能力与新站点硬件适配是两件独立的事。网络依赖准备好后，还要检查机器人系统版本、夹爪接线、安装几何和录制场景。
