# Ros2Go2Base 🦾

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**Ros2Go2Base** 是四足/双足机器人在 **ROS 2 Humble / Ubuntu 22.04** 上的 **底层集成仓库**，主要负责

* DDS ⇄ ROS 2 桥接
* Unitree SDK v2 运动控制
* 点云 → Scan、TF 树发布
* Launch/配置统一管理

配合生态中的其它仓库，即可快速搭建 **驱动 → 估计 → 感知 → 交互** 的完整闭环系统。

---

## ✨ 工程整体特性

| 类别                         | 说明                                                            |
| -------------------------- | ------------------------------------------------------------- |
| **高精度里程计**                 | 完全基于 **纯运动学**：IMU + 足端力传感器 + 关节角度/速度；无需相机 / LiDAR，可选融合进一步提高精度 |
| **双足 / 四足自适应**             | 不需要在估计算法里显式切模式，直接切步态即可                                        |
| **全 ROS 2 话题接口**           | 通过 `config.yaml` 可自由修改话题名；默认全部使用 `SMX/*` 命名空间                 |
| **可选 SLAM Toolbox / Nav2** | 提供现成 Launch，可在纯里程计地图、导航基础上快速扩展                                |
| **Ubuntu 20.04 Foxy 兼容**   | apt 依赖包名替换 `-foxy-` 即可在 Foxy 运行                               |

---

## 🏗️ 生态仓库一览

| 范畴       | 仓库                                                                                                   | 功能简介                             |
| -------- | ---------------------------------------------------------------------------------------------------- | -------------------------------- |
| **底层驱动** | **Ros2Go2Base (本仓库)**                                                                                | DDS 桥、Unitree SDK2 控制、点云→Scan、TF |
| 里程计      | [https://github.com/ShineMinxing/Ros2Go2Estimator](https://github.com/ShineMinxing/Ros2Go2Estimator) | 纯运动学多传感器融合                       |
| 语音 / LLM | [https://github.com/ShineMinxing/Ros2Chat](https://github.com/ShineMinxing/Ros2Chat)                 | 离线 ASR + OpenAI Chat + 语音合成      |
| 图像处理     | [https://github.com/ShineMinxing/Ros2ImageProcess](https://github.com/ShineMinxing/Ros2ImageProcess) | 相机、光点/人脸/无人机检测                   |
| 吊舱跟随     | [https://github.com/ShineMinxing/Ros2AmovG1](https://github.com/ShineMinxing/Ros2AmovG1)             | Amov G1 吊舱控制、目标跟踪                |
| 工具集      | [https://github.com/ShineMinxing/Ros2Tools](https://github.com/ShineMinxing/Ros2Tools)               | 蓝牙 IMU、手柄映射、吊舱闭环、数据采集            |

> ⚠️ 按需克隆：若只想驱动底盘，可 **仅使用本仓库**。其它仓库互不强依赖。

---

## 📂 本仓库结构

```text
Ros2Go2Base/
├── config.yaml            # 全局话题 & 网络配置
├── dds_rostopic/          # DDS → ROS2 桥
├── sport_control/         # 手柄 & 指令 → Unitree SDK2
├── message_handle/        # 点云→Scan、TF / Frame 处理
├── unitree_sdk2/          # 官方 C++ SDK (git submodule)
├── local_file/            # 数据录制 / 回放
└── other/                 # Launch、RViz 配置
```

### 主要包说明

* **dds\_rostopic** – 监听 Unitree DDS，发布 `/SMX/*` 传感器话题
* **sport\_control** – 订阅 `/SMX/SportCmd` & 手柄指令，将其映射为 SDK2 API（站立 / 坐 / 行走等）
* **message\_handle** – 将 `utlidar_lidar` 点云转换成 2D `/SMX/Scan`，并发布 `base_link*` TF
* **local\_file** – CSV / 视频录制、回放脚本

---

## ⚙️ 安装与编译

```bash
# 1. 安装依赖
sudo apt update && sudo apt install -y \
  ros-humble-rmw-cyclonedds-cpp ros-humble-pcl-ros \
  ros-humble-slam-toolbox ros-humble-nav2-bringup \
  ros-humble-joy python3-pip libeigen3-dev libpoco-dev
pip3 install openai httpx pydub vosk pygame pyttsx3

# 2. 创建工作空间并克隆
mkdir -p ~/ros2_ws/LeggedRobot/src && cd ~/ros2_ws/LeggedRobot/src

# ↓↓↓ 必要仓库 ↓↓↓
git clone --recursive https://github.com/ShineMinxing/Ros2Go2Base.git

# ↓↓↓ 可选功能仓库 ↓↓↓
git clone https://github.com/ShineMinxing/Ros2Go2Estimator.git
git clone https://github.com/ShineMinxing/Ros2Chat.git
git clone https://github.com/ShineMinxing/Ros2ImageProcess.git
git clone https://github.com/ShineMinxing/Ros2AmovG1.git
git clone https://github.com/ShineMinxing/Ros2Tools.git

# 3. 配置
#   - 搜索并替换 "~/ros2_ws/LeggedRobot" 为你的路径
#   - 将 config.yaml 内 network_interface 设置为本机网卡 (如 enxf8e43b808e06)

# 4. 编译
cd ~/ros2_ws/LeggedRobot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 启动示例

```bash
# (1) 基础驱动
ros2 launch sport_control go2_launch.py

# (2) 纯里程计建图
ros2 launch sport_control slam_launch.py

# (3) Nav2 导航
ros2 launch sport_control nav_launch.py

# (4) 吊舱跟随 (Amov G1)
ros2 launch sport_control g1_launch.py

# (5) 语音交互
ros2 run voice_chat voice_chat_node
```

> **RT+左摇杆**=行走、**RT+右摇杆**=转向。全部映射详见 `sport_control_node.cpp control_message_node.cpp`。

---

## 🎥 视频演示

| 主题               | 点击图片观看                                                                                                                                |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| 纯里程计建图 (站立/四足切换) | [![img](https://i1.hdslb.com/bfs/archive/4f60453cb37ce5e4f593f03084dbecd0fdddc27e.jpg)](https://www.bilibili.com/video/BV1UtQfYJExu)  |
| 室内行走误差 0.5 %‑1 %     | [![img](https://i1.hdslb.com/bfs/archive/10e501bc7a93c77c1c3f41f163526b630b0afa3f.jpg)](https://www.bilibili.com/video/BV18Q9JYEEdn/) |
| 爬楼梯高度误差 < 5 cm      | [![img](https://i0.hdslb.com/bfs/archive/c469a3dd37522f6b7dcdbdbb2c135be599eefa7b.jpg)](https://www.bilibili.com/video/BV1VV9ZYZEcH/) |
| 户外行走380m误差 3.3 %     | [![img](https://i0.hdslb.com/bfs/archive/481731d2db755bbe087f44aeb3f48db29c159ada.jpg)](https://www.bilibili.com/video/BV1BhRAYDEsV/) |
| 语音交互 + 地图导航        | [![img](https://i2.hdslb.com/bfs/archive/5b95c6eda3b6c9c8e0ba4124c1af9f3da10f39d2.jpg)](https://www.bilibili.com/video/BV1HCQBYUEvk/) |
| 人脸识别跟踪 + 光点跟踪     | [![img](https://i0.hdslb.com/bfs/archive/5496e9d0b40915c62b69701fd1e23af7d6ffe7de.jpg)](https://www.bilibili.com/video/BV1faG1z3EFF/) |
| AR眼镜头部运动跟随         | [![img](https://i1.hdslb.com/bfs/archive/9e0462e12bf77dd9bbe8085d0d809f233256fdbd.jpg)](https://www.bilibili.com/video/BV1pXEdzFECW) |
| YOLO无人机识别与跟随       | [![img](https://i1.hdslb.com/bfs/archive/a5ac45ec76ccb7c3fb18de9c6b8df48e8abe2b54.jpg)](https://www.bilibili.com/video/BV18v8xzJE4G) |
| 机器狗光电吊舱与固定相机协同 | [![img](https://i2.hdslb.com/bfs/archive/07ac6082b7efdc2e2d200e18fc8074eec1d9cfba.jpg)](https://www.bilibili.com/video/BV1fTY7z7E5T) |
| 多种神经网络位置预测        | [![img](https://i1.hdslb.com/bfs/archive/650062a4aeb28cb7bfdd15e658de1523f537efb7.jpg)](https://www.bilibili.com/video/BV1ytMizEEdG) |

---

## 📄 深入阅读

* 技术原理笔记：[https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22](https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22)
* ROS1 版本参考：[https://github.com/ShineMinxing/FusionEstimation](https://github.com/ShineMinxing/FusionEstimation)

---

## 📨 联系我们

| 邮箱                                          | 单位           |
| ------------------------------------------- | ------------ |
| [401435318@qq.com](mailto:401435318@qq.com) | 中国科学院光电技术研究所 |

> 📌 **本仓库仍在持续开发中** — 欢迎 Issue / PR 交流、贡献！
