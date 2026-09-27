# rl_sar

[![Ubuntu 20.04/22.04](https://img.shields.io/badge/Ubuntu-20.04/22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![macOS](https://img.shields.io/badge/macOS-Experimental-orange.svg?logo=apple)](https://www.apple.com/macos/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.2.7-orange.svg?logo=mujoco)](https://mujoco.org/)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2.0)

> 支持**IsaacGym**和**IsaacSim**
>
> **无 ROS 依赖**(主控程序内置 WebSocket 服务器与 Web 调试台)
>
> 支持**libtorch**和**onnxruntime**
>
> 支持**Linux**和**macOS**(只支持Mujoco仿真)
>
> 支持**Mujoco**仿真
>
> 支持**Locomotion**和**Dance**

## 准备

拉取仓库

```bash
git clone --recursive --depth 1 https://github.com/fan-ziqi/rl_sar.git
```

如需更新

```bash
git pull
git submodule update --init --recursive --recommend-shallow --progress
```

## 依赖

安装必要的依赖：

```bash
# Ubuntu
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev

# macOS
brew install boost lcm yaml-cpp tbb libomp pkg-config glfw
```

本项目**不依赖 ROS**。编译产物直接在宿主机运行，无需安装任何 ROS 发行版。

## 编译

在项目根目录中执行下面的脚本编译整个项目

```bash
./build.sh
```

若想删除构建产物，可以使用下列命令

```bash
./build.sh -c  # or ./build.sh --clean
```

若想使用Mujoco仿真器

```bash
./build.sh -mj  # or ./build.sh --mujoco
```

详细的使用说明可以通过`./build.sh -h`查看

```bash
Usage: ./build.sh [OPTIONS]

Options:
  -c, --clean    Clean build artifacts (cmake_build/)
  -m, --cmake    Build using CMake (default, for hardware deployment)
  -mj,--mujoco   Build with MuJoCo simulator support (CMake only)
  -h, --help     Show this help message
```

## 运行

下文中使用 **\<ROBOT\>/\<CONFIG\>** 代替表示不同的环境，如 `JXG/amp` 、 `JXGw/himloco`。

运行前请将训练好的pt模型文件拷贝到`policy/<ROBOT>/<CONFIG>`中，并配置`<ROBOT>/<CONFIG>/config.yaml`和`<ROBOT>/base.yaml`中的参数。

### 仿真

#### Mujoco

```bash
./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>
```

### Web 调试台 (内置,无需额外服务)

主控程序 `rl_real_JXG` 内置 WebSocket 服务器,与 Web 调试台静态文件同端口托管,无需 rosbridge / web_video_server 等任何额外服务。

启动主程序后,浏览器访问 `http://<机器人IP>:8088` 即可打开调试台;端口与静态目录可通过 `--web-port` / `--web-root` 修改。调试台提供 3D 模型跟随、电机/IMU/里程计遥测、速度遥控、行为命令、通知与作业交底播报、通道调试等功能。

### 使用手柄或键盘控制

|手柄控制|键盘控制|功能描述|
|---|---|---|
|**基础**|||
|A|Num0|让机器人从程序开始运行时的姿态以位控插值运动到`base.yaml`中定义的`default_dof_pos`|
|B|Num9|让机器人从当前位置以位控插值运动到程序开始运行时的姿态|
|X|N|切换导航模式 (导航模式屏蔽手柄速度命令，接收 Web 下发的`cmd_vel`通道)|
|Y|N/A|N/A|
|**仿真**|||
|RB+Y|R|重置Gazebo环境 (让摔倒的机器人站起来)|
|RB+X|Enter|切换Gazebo运行/停止 (默认为运行状态)|
|**电机**|||
|LB+A|M|N/A (推荐设置为电机使能)|
|LB+B|K|N/A (推荐设置为电机失能)|
|LB+X|P|电机Passive模式 (`kp=0, kd=8`)|
|LB+RB|N/A|N/A (推荐设置为急停保护)|
|**技能**|||
|RB+DPadUp|Num1|基础Locomotion|
|RB+DPadDown|Num2|技能2|
|RB+DPadLeft|Num3|技能3|
|RB+DPadRight|Num4|技能4|
|LB+DPadUp|Num5|技能5|
|LB+DPadDown|Num6|技能6|
|LB+DPadLeft|Num7|技能7|
|LB+DPadRight|Num8|技能8|
|**移动**|||
|LY轴|W/S|前后移动 (X轴)|
|LX轴|A/D|左右移动 (Y轴)|
|RX轴|Q/E|偏航旋转 (Yaw)|
|N/A(松开摇杆)|Space|将所有控制指令设置为零|

### 真实机器人

#### JXG / JXGw

JXG 通过自定义 JXG SDK（USB2CAN）连接电机和 IMU。

启动控制程序（内置 Web 调试台 `http://<机器人IP>:8088` 随主程序一同启动）。如果控制 JXGw（轮腿模式），使用 `wheel` 参数。

```bash
# 手动启动(推荐用脚本: 自动等待 CAN 接口 UP 后拉起主程序)
scripts/start.sh            # wheel 模式
scripts/start.sh --help     # 查看全部参数(--web-port / --web-root 等)
```

**可用策略配置：**

| 机器人 | 配置 | 说明 |
|---|---|---|
| JXG | `JXG/amp` | AMP locomotion |
| JXG | `JXG/dwaq` | DreamWaQ |
| JXG | `JXG/himloco_amp` | HimLoco with AMP |
| JXG | `JXG/np3o` | NP3O |
| JXGw | `JXGw/himloco` | HimLoco (轮腿) |

#### 开机自启动

仓库根目录已提供现成的服务文件 `rl-real-jxg.service`（裸机直跑，含 CAN 等待与实时性调度参数），直接安装即可：

```bash
sudo cp rl-real-jxg.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rl-real-jxg.service
```

禁用开机自动启动：

```bash
sudo systemctl disable rl-real-jxg.service
```

启动/停止/重启服务：

```bash
sudo systemctl start rl-real-jxg.service
sudo systemctl stop rl-real-jxg.service
sudo systemctl restart rl-real-jxg.service
```

查看服务日志：

```bash
sudo journalctl -u rl-real-jxg.service -f
```

## 添加你的机器人

下面使用 **\<ROBOT\>/\<CONFIG\>** 代替表示你的机器人环境。你只需要创建或修改下述文件，命名必须跟下面一样。（你可以参考JXGw对应的文件）

```yaml
# 你的机器人description（仿真用）
rl_sar/src/rl_sar_zoo/<ROBOT>_description/CMakeLists.txt
rl_sar/src/rl_sar_zoo/<ROBOT>_description/package.ros1.xml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/package.ros2.xml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/xacro/robot.xacro
rl_sar/src/rl_sar_zoo/<ROBOT>_description/xacro/gazebo.xacro
rl_sar/src/rl_sar_zoo/<ROBOT>_description/config/robot_control.yaml
rl_sar/src/rl_sar_zoo/<ROBOT>_description/config/robot_control_ros2.yaml

# 你训练的policy
policy/<ROBOT>/base.yaml  # 此文件中必须遵守实物机器人的关节顺序
policy/<ROBOT>/<CONFIG>/config.yaml
policy/<ROBOT>/<CONFIG>/<POLICY>.pt  # libtorch使用，注意导出jit
policy/<ROBOT>/<CONFIG>/<POLICY>.onnx  # onnxruntime使用

# 机器人的fsm
src/rl_sar/fsm_robot/fsm_<ROBOT>.hpp
src/rl_sar/fsm_robot/fsm_all.hpp

# 你实物机器人的代码
rl_sar/src/rl_sar/src/rl_real_<ROBOT>.cpp  # 可以按需自定义forward()函数以适配您的policy
```

## 贡献

衷心欢迎社区的贡献，以使这个框架更加成熟和对所有人有用。贡献可以是bug报告、功能请求或代码贡献。

[贡献者名单](CONTRIBUTORS.md)

## 引用

如果您使用此代码或其部分内容，请引用以下内容：

```
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```

## 致谢

本项目使用了以下开源代码库中的部分代码：

- [unitreerobotics/unitree_sdk2-2.0.0](https://github.com/unitreerobotics/unitree_sdk2/tree/2.0.0)
- [unitreerobotics/unitree_legged_sdk-v3.2](https://github.com/unitreerobotics/unitree_legged_sdk/tree/v3.2)
- [unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)
- [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
- [google-deepmind/mujoco-3.2.7](https://github.com/google-deepmind/mujoco)
- [mertgungor/unitree_model_control](https://github.com/mertgungor/unitree_model_control)
- [Improbable-AI/walk-these-ways](https://github.com/Improbable-AI/walk-these-ways)
- [ccrpRepo/RoboMimic_Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy)
- [Deeprobotics/Lite3_Motion_SDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)
- [chengyangkj/ROS_Flutter_Gui_App](https://github.com/chengyangkj/ROS_Flutter_Gui_App)
