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

## 🏗️ 生态仓库详细信息参见

[https://github.com/ShineMinxing/Ros2Go2Estimator](https://github.com/ShineMinxing/Ros2Go2Estimator)

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
## 📄 深入阅读

* 技术原理笔记：[https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22](https://www.notion.so/Ros2Go2-1e3a3ea29e778044a4c9c35df4c27b22)
* ROS1 版本参考：[https://github.com/ShineMinxing/FusionEstimation](https://github.com/ShineMinxing/FusionEstimation)

---

## 📨 联系我们

| 邮箱                                          | 单位           |
| ------------------------------------------- | ------------ |
| [401435318@qq.com](mailto:401435318@qq.com) | 中国科学院光电技术研究所 |

> 📌 **本仓库仍在持续开发中** — 欢迎 Issue / PR 交流、贡献！
