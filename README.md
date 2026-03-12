# fly-hector

一个面向无人机/小型飞行平台的综合工程仓库，包含飞控固件源码、ROS 导航与建图工作空间，以及凌霄飞控相关资料和原理图。

这个仓库当前主要由三部分组成：

1. `ANO_LX_FC/`
   匿名凌霄飞控源码工程，包含多种 MCU 工程模板和底层驱动。
2. `ros/`
   基于 ROS Noetic 的导航工作空间，集成 Hector SLAM、激光雷达驱动、串口控制、路径跟踪和颜色识别。
3. `凌霄/`
   配套资料目录，含用户手册、通信协议、原理图、开发环境包和参考 PDF。

## 目录结构

```text
fly/
├─ ANO_LX_FC/
│  └─ ANO_LX_FC/
│     ├─ FcSrc/                  # 飞控主逻辑与用户任务
│     ├─ DriversBsp/             # 板级驱动
│     ├─ DriversMcu/             # STM32F407 / TM4C123 / MSP432 驱动与库
│     ├─ ProjectSTM32F407/       # STM32F407 Keil 工程
│     ├─ ProjectTM4C123/         # TM4C123 Keil 工程
│     └─ ProjectMSP432/          # MSP432 Keil 工程
├─ ros/
│  ├─ src/color_detector/        # 摄像头颜色块识别节点
│  ├─ src/hector_slam-noetic/    # Hector SLAM 及自定义导航脚本
│  ├─ src/lsx10/                 # 激光雷达驱动相关包
│  └─ src/vision_opencv/         # cv_bridge 等视觉桥接代码
└─ 凌霄/
   ├─ 1.用户手册_通信协议/
   ├─ 3.开发环境安装/
   ├─ 5.飞控MCU源码工程/
   ├─ 6.原理图_PCB/
   ├─ 7.凌霄IMU固件/
   └─ 8.其他资料PDF/
```

## 仓库内容说明

### 1. 飞控固件工程

固件主入口位于：

- `ANO_LX_FC/ANO_LX_FC/FcSrc/main.c`
- `ANO_LX_FC/ANO_LX_FC/FcSrc/User_Task.c`
- `ANO_LX_FC/ANO_LX_FC/FcSrc/SysConfig.h`

根据当前源码可以看出：

- `main.c` 采用裸机任务调度方式，先执行 `All_Init()`，然后在主循环中持续调用 `Scheduler_Run()`。
- `User_Task.c` 中实现了串口接收解析逻辑，使用形如 `<x y>` 的文本帧接收控制命令。
- 当前任务逻辑中已经接入了解锁、起飞、降落等动作调用，例如：
  - `0x40` 对应解锁
  - `0x41` / `0x42` 组合用于一键起飞
  - `0x43` 对应降落
- `SysConfig.h` 中可见系统参数，例如 `PWM_FRE_HZ = 400`，以及 GPS 相关配置宏。

支持的 MCU/工程目录：

- `ProjectSTM32F407/`
- `ProjectTM4C123/`
- `ProjectMSP432/`

如果你当前硬件是 STM32F407 平台，通常优先从以下工程打开：

- `ANO_LX_FC/ANO_LX_FC/ProjectSTM32F407/ANO_LX_STM32F407.uvprojx`

### 2. ROS 导航工作空间

`ros/` 目录是一个 catkin 工作空间，包含自定义导航与视觉逻辑。

重点包和脚本如下：

- `ros/src/hector_slam-noetic/hector_slam-noetic-devel/hector_mapping/launch/drone_navigation.launch`
  单文件集成导航启动方案，包含雷达、IMU、Hector SLAM、EKF、move_base、路径跟踪和 RViz。
- `ros/src/hector_slam-noetic/hector_slam-noetic-devel/hector_mapping/launch/phase1.launch`
  先通过串口发送解锁/起飞命令，再自动启动 `phase2.launch`。
- `ros/src/hector_slam-noetic/hector_slam-noetic-devel/hector_mapping/launch/phase2.launch`
  启动建图、EKF、move_base 和路径跟踪任务，按预设航点飞行。
- `ros/src/hector_slam-noetic/hector_slam-noetic-devel/hector_mapping/launch/phase3.launch`
  启动颜色识别节点。
- `ros/src/hector_slam-noetic/hector_slam-noetic-devel/hector_mapping/scripts/serial_cmd_and_launcher.py`
  用串口向飞控发送控制命令，并串联后续 launch。
- `ros/src/hector_slam-noetic/hector_slam-noetic-devel/hector_mapping/scripts/path_follower_node.py`
  订阅 `move_base` 局部路径，把路径离散成串口速度命令，并按航点执行悬停和阶段切换。
- `ros/src/hector_slam-noetic/hector_slam-noetic-devel/hector_mapping/scripts/imu_serial_node.py`
  从串口读取 IMU 数据并发布到 `/imu/data`。
- `ros/src/color_detector/scripts/color_blob_raw_node.py`
  通过摄像头识别红/绿/蓝大面积色块并发布结果。

说明：

- `ros/src/vision_opencv` 当前直接保留在仓库内，便于完整保存你本地已经修改过的代码状态。

### 3. 凌霄资料目录

`凌霄/` 目录收集了大量配套材料，包括：

- 飞控手册
- 通信协议
- 原理图和 PCB 文件
- IMU 固件
- 参考资料 PDF
- 开发环境安装包

这个目录更偏向“资料备份”和“开发参考”，不一定全部参与编译。

## 推荐开发环境

### 固件侧

- Windows
- Keil MDK
- J-Link 驱动/下载器
- 对应芯片包与设备支持包

### ROS 侧

根据目录命名和脚本写法，当前 ROS 部分更适合以下环境：

- Ubuntu 20.04
- ROS Noetic
- Python 3
- OpenCV
- `pyserial`
- `robot_localization`
- `move_base`
- `dwa_local_planner`
- `rviz`

典型安装命令可参考：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-robot-localization \
  ros-noetic-navigation \
  python3-serial \
  python3-opencv
```

如果你的系统并非 Ubuntu 20.04 / ROS Noetic，请根据实际环境自行调整依赖。

## 固件编译方式

### STM32F407

1. 用 Keil 打开：

   `ANO_LX_FC/ANO_LX_FC/ProjectSTM32F407/ANO_LX_STM32F407.uvprojx`

2. 检查 `FcSrc/` 和 `DriversMcu/STM32F407/` 下的配置。
3. 连接 J-Link 或其他下载器。
4. 编译并烧录到飞控主板。

### 其他 MCU

如果你使用 TM4C123 或 MSP432，可切换到：

- `ProjectTM4C123/ANO_LX_TM4C123.uvprojx`
- `ProjectMSP432/ANO_LX_MSP432.uvprojx`

## ROS 工作空间使用方式

### 1. 编译工作空间

```bash
cd ros
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

说明：

- 本仓库不会跟踪 `ros/build`、`ros/devel`、`ros/logs` 等本地产物。
- 如果你习惯 `catkin_make`，也可以自行切换，但当前目录中保留了 `catkin_tools` 习惯用法。

### 2. 典型启动流程

完整流程可以从 `phase1.launch` 开始：

```bash
roslaunch hector_mapping phase1.launch
```

该流程大致分为三段：

1. `phase1.launch`
   通过 `serial_cmd_and_launcher.py` 发送解锁与起飞命令。
2. `phase2.launch`
   启动激光雷达、Hector SLAM、EKF、move_base 和路径跟踪。
3. `phase3.launch`
   在后续阶段启动颜色识别任务。

如果你只想直接进入导航链路，也可以尝试：

```bash
roslaunch hector_mapping drone_navigation.launch
```

## 串口与设备默认约定

从当前 launch 文件与脚本读取到的默认设备如下：

- 飞控控制串口：`/dev/ttyS7`
- IMU 串口：`/dev/ttyS3`
- 摄像头：`/dev/video0`

如果你的设备节点不同，请修改对应 launch 参数或脚本默认值。

## 航点与任务流程

`phase2.launch` 和 `path_follower_node.py` 中定义了一组任务航点，节点会：

1. 从 `move_base` 的局部路径中提取速度趋势。
2. 将二维速度转换为串口帧，例如 `<vx vy>`。
3. 到达指定航点后执行悬停。
4. 在特定航点调用颜色识别逻辑。
5. 任务完成后切换到下一阶段 launch。

这说明当前工程不仅是“建图”，还包含一套完整的飞行任务编排逻辑。

## 颜色识别模块

当前颜色识别逻辑使用 OpenCV 的 HSV 阈值法，识别：

- Red
- Green
- Blue

相关位置：

- `ros/src/color_detector/scripts/color_blob_raw_node.py`
- `ros/src/hector_slam-noetic/hector_slam-noetic-devel/hector_mapping/scripts/path_follower_node.py`
- `ros/src/hector_slam-noetic/hector_slam-noetic-devel/hector_mapping/launch/phase3.launch`

识别结果可用于任务阶段判断或后续控制扩展。

## 已忽略内容

为保证仓库可以正常推送到 GitHub，以下内容不会被纳入版本控制：

- ROS 编译产物和日志
- Keil / J-Link 生成的本地配置和日志文件
- 超过 GitHub 单文件限制的安装包：
  - `凌霄/3.开发环境安装/Keil.STM32F4xx_DFP.2.2.0.pack`

如果后续确实需要保存超大文件，建议改用：

- Git LFS
- Releases 附件
- 网盘/制品仓库

## 使用建议

- 若仓库计划公开，请先确认 `凌霄/` 目录中的第三方资料、安装包和文档是否允许公开分发。
- 串口号、航点坐标、悬停时间等参数建议在 launch 中统一管理，避免散落在脚本中。
- 如果后续准备长期维护，建议继续补充：
  - 硬件接线图
  - 飞控串口协议说明
  - 实测启动步骤
  - 任务演示视频或截图

## 当前状态

当前仓库已经具备：

- 飞控固件源码
- ROS 导航工作空间
- 激光雷达 / IMU / 串口控制逻辑
- 颜色识别任务节点
- 原理图和资料归档

适合作为一个“飞控 + ROS 自主导航 + 任务执行”综合工程仓库继续整理和迭代。
