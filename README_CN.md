# ATOM01 ROS2 Deploy

[![ROS2](https://img.shields.io/badge/ROS2-Humble-silver)](https://docs.ros.org/en/humble/index.html)
![C++](https://img.shields.io/badge/C++-17-blue)
[![Linux platform](https://img.shields.io/badge/platform-linux--x86_64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Linux platform](https://img.shields.io/badge/platform-linux--aarch64-orange.svg)](https://releases.ubuntu.com/22.04/)

[English](README.md) | [中文](README_CN.md)

## 概述

本仓库提供了使用 ROS2 作为中间件的部署框架，并具有模块化架构，便于无缝定制和扩展。

开源地址：[https://github.com/zerojuhao/rpo_deploy_depth](https://github.com/zerojuhao/rpo_deploy_depth)

**维护者**: 刘志浩
**联系方式**: <ZhihaoLiu_hit@163.com>

**主要特性:**

- **易于上手**: 提供全部细节代码，便于学习并允许修改代码。
- **隔离性**: 不同功能由不同包实现，支持加入自定义功能包。
- **长期支持**: 本仓库将随着训练仓库代码的更新而更新，并将长期支持。

## 主控连接

部署框架在 **Orange Pi 5 Plus** 与 **RDK X5** 上经过了充分验证。

- **Orange Pi 5 Plus**: 系统为 `Ubuntu 22.04`，内核版本为 `5.10`
- **RDK X5**: 系统为 `Ubuntu 22.04`，内核版本为 `6.1.83`

关于主控的连接方法和相关资料，参见 [Orange Pi 5 Plus Wiki](http://www.orangepi.cn/orangepiwiki/index.php/Orange_Pi_5_Plus) 与 [RDK X5 Doc](https://d-robotics.github.io/rdk_doc/Quick_start/hardware_introduction/rdk_x5)。

## 环境配置

1. 首先安装 ROS2 Humble，参考 [ROS 官方](https://docs.ros.org/en/humble/Installation.html) 进行安装。

2. 部署还依赖 `ccache`、`fmt`、`spdlog`、`eigen3`、`screen` 等库，在主控中执行指令进行安装：

   ```bash
   sudo apt update && sudo apt install -y ccache libfmt-dev libspdlog-dev libeigen3-dev screen
   ```

3. 若需使用手柄控制，还需安装 ROS2 的 `joy` 包：

   ```bash
   sudo apt install -y ros-humble-joy
   ```

4. 若需使用仓库中的 Python 脚本（如 `scripts/set_zero.py`），还需安装对应 Python 依赖：

   ```bash
   sudo apt install -y python3-yaml python3-numpy
   ```

5. 接着拉取部署代码：

   ```bash
   git clone https://github.com/zerojuhao/rpo_deploy_depth.git
   cd rpo_deploy_depth
   ```

6. 如果使用 Orange Pi 5 Plus，执行下面的指令为其安装 **5.10 实时内核**：
   
   > **注意**：RDK X5 无需执行此步骤，请直接烧录我们提供的、已预装实时内核的镜像。

   ```bash
   cd assets
   sudo apt install ./*.deb
   cd ..
   ```

7. 接下来为用户授予实时优先级设置权限：

   ```bash
   sudo nano /etc/security/limits.conf
   ```

   在文件末尾添加以下两行（**请务必将 `orangepi` 替换为你的实际用户名**，例如 RDK X5 的默认用户名为 `sunrise`）：

   ```bash
   # Allow user 'orangepi' to set real-time priorities
   orangepi   -   rtprio   98
   orangepi   -   memlock  unlimited
   ```

   重启设备使配置生效，随后通过以下指令验证：

   ```bash
   ulimit -r
   ```

   > **提示**：输出为 **98** 即代表配置成功。

## AP配置（可选）

为方便脱离网线和显示器调试，可以为主控板开启 WiFi 热点（AP）。配置相关文件在 `tools/create_ap` 目录中。

> **注意**：由于单网卡限制，开启 AP 模式后，主控板自带的 WiFi 将难以连接家用路由器等其他外部网络。
>
> - **如需连接外网加载包或环境**，请为主控板**接入有线网络**。
> - **如想暂时恢复无线上网**，可以通过以下命令停止服务（需要外接显示器或网线登录）：
>   ```bash
>   sudo systemctl stop create_ap.service
>   ```

1. 在项目根目录下执行，安装并赋予权限：

   ```bash
   sudo cp tools/create_ap/create_ap /usr/bin/
   sudo chmod +x /usr/bin/create_ap
   ```

2. 部署 systemd 服务文件：

   ```bash
   sudo cp tools/create_ap/create_ap.service /etc/systemd/system/
   ```

3. 根据你的主控板复制配置文件：

   **Orange Pi 5 Plus** 请使用该配置：

   ```bash
   sudo cp tools/create_ap/create_ap_orangepi.conf /etc/create_ap.conf
   ```

   **RDK X5** 请使用该配置：

   ```bash
   sudo cp tools/create_ap/create_ap_sunrise.conf /etc/create_ap.conf
   ```

   > **说明**：默认配置下的热点名称（`SSID`）为 **`atom`**，连接密码（`PASSPHRASE`）为 **`jujujuju`**。如需自定义热点名称或密码，可编辑 `/etc/create_ap.conf` 文件并修改对应的字段。

4. 开启开机自启并立即启动热点：

   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable create_ap.service
   sudo systemctl start create_ap.service
   ```

## 硬件配置

在连接之前，请先完成对电机 ID 和 IMU 波特率以及频率的设置。

对于 **电机 ID**，请参见 [产品安装手册](https://roboparty.feishu.cn/wiki/OiO2wF4NiiE08Yk1yJjcgnumnUw) 中对电机 ID 的定义，并使用达妙上位机工具进行设置，使用教程参见 [达妙科技文档](https://gitee.com/kit-miao/damiao-document)。

对于 **IMU**，我们默认使用 **`921600` 波特率** 与 **`500HZ` 频率**。如何使用上位机进行修改参见 [HiPNUC 产品手册](https://www.hipnuc.com/resource_hi14.html)。
> **提示**：也可以使用其他波特率，但请 **保证频率大于 200HZ**。若使用其他波特率，请同步修改 `src/inference/config/robot.yaml` 中的 IMU 配置。

## 硬件连接

电机驱动的默认 CAN 映射关系如下（按照 USB 转 CAN 插入主控的顺序编号，先插的为 `can0`）：
- **`can0`** 对应 **左腿**
- **`can1`** 对应 **右腿加腰**
- **`can2`** 对应 **左手**
- **`can3`** 对应 **右手**

> **建议**：将 USB 转 CAN 插在主控的 **USB 3.0 接口**上。如果使用 USB 扩展坞，也请使用 3.0 接口的扩展坞并插在 3.0 接口上；IMU 和手柄插在 USB 2.0 接口即可。具体可参见 [走线说明](https://roboparty.feishu.cn/wiki/QeY2wozbiiIivlkBfdccvqVlnog)。

### 方式一：手动配置（不推荐）
如果不配置 udev 规则，则需要严格按照上文顺序插入 USB 转 CAN，并插入 IMU 后手动配置 CAN 和 IMU 串口：

```bash
# CAN 配置
sudo ip link set canX up type can bitrate 1000000
sudo ip link set canX txqueuelen 1000
# canX 为 can0 can1 can2 can3，需要为每个 can 都输入一遍上面两个指令

# IMU 配置
sudo chmod 666 /dev/ttyUSB0
```

### 方式二：使用 udev 规则自动绑定（推荐）
编写 udev 规则将 USB 接口与对应设备物理绑定，这样就**不需要按顺序插入设备**。示例提供了 `99-auto-up-devs-orangepi.rules` 与 `99-auto-up-devs-sunrise.rules`。如果连线方式与 [走线说明](https://roboparty.feishu.cn/wiki/QeY2wozbiiIivlkBfdccvqVlnog) 完全一致，可直接使用。

接线不一致则需要修改文件中的 `KERNELS` 项，将其对应到实际绑定的 USB 接口。在主控输入以下指令以监视 USB 事件：

```bash
sudo udevadm monitor
```

在 USB 接口插入设备时，终端就会显示该 USB 接口的 `KERNELS` 属性项，如 `/devices/pci0000:00/0000:00:14.0/usb3/3-8`，在匹配 `KERNELS` 属性项时使用 `3-8` 即可。如果绑定在该 USB 接口上的扩展坞上的 USB 口，则会有 `3-8.x` 出现，此时使用 `3-8.x` 匹配扩展坞上的 USB 口即可。

编写完成后在项目根目录下执行：

```bash
# RDK X5 使用 assets/99-auto-up-devs-sunrise.rules
sudo cp assets/99-auto-up-devs-orangepi.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
```

重启主控即可生效。

该 udev 规则还包括 IMU 串口配置。如果规则正常生效，CAN 接口应该全部自动配置完毕并使能，可以在主控中输入 `ip a` 指令查看结果。

## 软件使用

### 电机标零（首次使用/零点丢失时）

> **说明**：电机标零通常只需要在首次使用时执行一次；如果电机经过检修、更换，或出现零点丢失，也需要重新执行标零。

仓库内提供了两种零点标定方式，适用于不同场景：

- `ros2 service call /set_zeros std_srvs/srv/Trigger`
  用于在机器人软件已经启动、电机已经初始化且推理未运行时，将当前关节位置写入电机零点。
- `python3 scripts/set_zero.py`
  用于逐个电机进行人工摆位标零，更适合首次装机、检修后重标定或只想对部分电机重新标零的场景。

使用 `/set_zeros` 服务时，建议按以下顺序操作：

1. 在当前终端先 source ROS2 环境和工作空间环境。
2. 运行 `./tools/start_robot.sh` 启动软件。
3. 调用 `/init_motors` 初始化电机。
4. 确认机器人处于目标零位，且此时没有运行推理。
5. 调用 `/set_zeros` 写入当前零点。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./tools/start_robot.sh
ros2 service call /init_motors std_srvs/srv/Trigger
ros2 service call /set_zeros std_srvs/srv/Trigger
```

使用脚本 `scripts/set_zero.py` 时：

1. 先完成工作空间编译，并确保 `install/setup.bash` 已生成。
2. 在当前终端 source ROS2 环境和工作空间环境。
3. 确认 CAN 接口与 udev 映射已正常生效。
4. 按需检查或修改 `scripts/config/set_zero.yaml`，确认电机 ID、CAN 接口、电机型号与实际硬件一致。
5. 在可交互终端中运行脚本后，按提示将当前电机手动摆到零位。
6. 按 `Enter` 写入该电机零点，按空格跳过当前电机。

```bash
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/set_zero.py
```

`scripts/set_zero.py` 会按 `scripts/config/set_zero.yaml` 中的顺序依次标定各电机，并在标定过程中将电机切换到阻尼模式，便于手动调整姿态。

### 启动软件

> **警告**：启动机器人前，确保机器人完成零点标定，**请务必阅读 [安全操作指南](https://roboparty.feishu.cn/wiki/ZGtnwpHCjii2XykBYMGchoBBnSl)！**

此外，请特别注意 `src/inference/config/robot.yaml` 中的零点偏移配置：

```yaml
motor_zero_offset: 
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.093,
     0.0, 0.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 0.0, 0.0]
```

- 如果是**将腰部 yaw 转至限位块处**进行标定：保留 `2.093`。
- 如果是**使用打印件固定腰部 yaw** 进行标定：将 `2.093` 改为 `0.0`。

一切准备就绪后，运行脚本启动软件：

```bash
./tools/start_robot.sh
```

`./tools/start_robot.sh` 会自动执行 `colcon build --symlink-install` 编译工作空间，并在后台启动以下两个 `screen` 会话：

- `inference_session`：推理节点
- `joy_session`：手柄节点

可使用以下命令查看后台输出：

```bash
screen -r inference_session
screen -r joy_session
```

可使用以下命令停止对应后台组件：

```bash
screen -S inference_session -X quit
screen -S joy_session -X quit
```

### Parkour 部署

本仓库支持基于深度相机的 Parkour 部署功能。启动 Parkour 程序：

```bash
./tools/start_robot_depth.sh
```

配置文件位于 `src/inference/config/` 目录下。Parkour 部署的配置文件为 `src/inference/config/inference_depth.yaml`。

如果需要切换不同的策略模型，可以先修改 `src/inference/launch/inference.launch.py` 中加载的配置文件：

```python
configs = [
    os.path.join(
        get_package_share_directory("inference"),
        "config",
        "inference.yaml",
    ),
]
```

将其中的 `inference.yaml` 替换为目标配置文件名即可，例如：

- `inference_amp.yaml`
- `inference_attn_enc.yaml`
- `inference_beyondmimic.yaml`
- `inference_getup.yaml`
- `inference_interrupt.yaml`

修改完成后，重新运行 `./tools/start_robot.sh`，启动时就会加载对应配置，从而使用不同策略。

### 手柄控制

- **X 键**: 使能 / 失能电机
- **A 键**: 复位电机
- **B 键**: 开始 / 暂停推理
- **Y 键**: 切换手柄控制 / cmd_vel 指令控制
- **LB 键**: 切换策略模式（在 beyondmimic / interrupt 模式下可用）
- **RB 键**: 切换运动序列（在 beyondmimic 模式下可用）
- **右摇杆**: 控制前后左右移动
- **LT/RT**: 控制转向（左 / 右旋转）

### 服务接口

可以通过命令行调用 ROS2 服务来控制机器人：

- **初始化电机**:
  
  ```bash
  ros2 service call /init_motors std_srvs/srv/Trigger
  ```

- **去初始化电机**:

  ```bash
  ros2 service call /deinit_motors std_srvs/srv/Trigger
  ```

- **开始推理**:

  ```bash
  ros2 service call /start_inference std_srvs/srv/Trigger
  ```

- **停止推理**:

  ```bash
  ros2 service call /stop_inference std_srvs/srv/Trigger
  ```

- **清除错误**:

  ```bash
  ros2 service call /clear_errors std_srvs/srv/Trigger
  ```

- **设置零点**:

  ```bash
  ros2 service call /set_zeros std_srvs/srv/Trigger
  ```

  该服务会将机器人当前姿态写入电机零点。调用前请确保当前终端已 source ROS2 和工作空间环境，且电机已初始化、机器人已摆到目标零位、当前没有运行推理。

- **重置关节**:

  ```bash
  ros2 service call /reset_joints std_srvs/srv/Trigger
  ```

- **刷新关节状态**:

  ```bash
  ros2 service call /refresh_joints std_srvs/srv/Trigger
  ```

- **读取关节状态**:

  ```bash
  ros2 service call /read_joints std_srvs/srv/Trigger
  ```

- **读取 IMU 状态**:

  ```bash
  ros2 service call /read_imu std_srvs/srv/Trigger
  ```

## Python SDK

本仓库提供了 Python SDK，方便用户使用 Python 脚本控制硬件。

> **注意**：`imu_py`、`motors_py`、`robot_py` 这三个模块来自工作空间编译产物。运行任何 Python SDK 示例或脚本前，请先完成工作空间编译，并 source ROS2 环境和本工作空间的 `install/setup.bash`。

```bash
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

> **提示**：详细 Python 脚本示例请参考 `scripts/` 目录。

### 1. IMU SDK (`imu_py`)

#### 静态方法

- `create_imu(imu_id: int, interface_type: str, interface: str, imu_type: str, baudrate: int = 0) -> IMUDriver`: 创建 IMU 驱动实例。

#### 成员方法

- `get_imu_id() -> int`: 获取 IMU ID。
- `get_ang_vel() -> List[float]`: 获取角速度 [x, y, z]。
- `get_quat() -> List[float]`: 获取四元数 [w, x, y, z]。
- `get_lin_acc() -> List[float]`: 获取线加速度 [x, y, z]。
- `get_temperature() -> float`: 获取温度。

#### 使用示例

```python
import imu_py
imu = imu_py.IMUDriver.create_imu(8, "serial", "/dev/ttyUSB0", "HIPNUC", 921600)
quat = imu.get_quat()
```

### 2. 电机 SDK (`motors_py`)

提供了 `MotorControlMode` 枚举：`NONE`, `MIT`, `POS`, `SPD`。

#### 静态方法

- `create_motor(motor_id: int, interface_type: str, interface: str, motor_type: str, motor_model: int, master_id_offset: int = 0, motor_zero_offset: double = 0.0) -> MotorDriver`: 创建电机驱动实例。

#### 成员方法

- `init_motor()`: 初始化电机。
- `deinit_motor()`: 去初始化电机。
- `set_motor_control_mode(mode: MotorControlMode)`: 设置控制模式。
- `motor_mit_cmd(pos: float, vel: float, kp: float, kd: float, torque: float)`: MIT 模式控制指令。
- `motor_pos_cmd(pos: float, spd: float, ignore_limit: bool = False)`: 位置模式控制指令。
- `motor_spd_cmd(spd: float)`: 速度模式控制指令。
- `lock_motor() / unlock_motor()`: 锁定/解锁电机。
- `set_motor_zero()`: 设置当前位置为零点。
- `clear_motor_error()`: 清除错误。
- `get_motor_pos() -> float`: 获取位置 (rad)。
- `get_motor_spd() -> float`: 获取速度 (rad/s)。
- `get_motor_current() -> float`: 获取电流 (A)。
- `get_motor_temperature() -> float`: 获取温度 (°C)。
- `get_error_id() -> int`: 获取错误码。
- `get_motor_id() -> int`: 获取电机 ID。
- `get_motor_control_mode() -> int`: 获取控制模式。
- `get_response_count() -> int`: 获取响应计数。
- `refresh_motor_status()`: 刷新电机状态。

#### 使用示例

```python
import motors_py
motor = motors_py.MotorDriver.create_motor(1, "can", "can0", "DM", 0, 16)
motor.init_motor()
motor.set_motor_control_mode(motors_py.MotorControlMode.MIT)
motor.motor_mit_cmd(0.0, 0.0, 5.0, 1.0, 0.0)
```

### 3. 机器人 SDK (`robot_py`)

`RobotInterface` 类用于统一控制整个机器人，读取配置文件自动加载电机和 IMU。

#### 构造函数

- `RobotInterface(config_file: str)`: 根据配置文件路径创建实例。

#### 成员方法

- `init_motors()`: 初始化所有电机。
- `deinit_motors()`: 去初始化所有电机。
- `reset_joints(joint_default_angle: List[float])`: 将所有关节重置到默认角度。
- `apply_action(action: List[float])`: 应用控制动作 (关节目标位置/力矩等，取决于内部实现)。
- `refresh_joints()`: 刷新所有关节状态。
- `set_zeros()`: 将当前所有关节位置设为零点。
- `clear_errors()`: 清除所有电机错误。
- `get_joint_q() -> List[float]`: 获取所有关节位置。
- `get_joint_vel() -> List[float]`: 获取所有关节速度。
- `get_joint_tau() -> List[float]`: 获取所有关节力矩。
- `get_quat() -> List[float]`: 获取 IMU 四元数 [w, x, y, z]。
- `get_ang_vel() -> List[float]`: 获取 IMU 角速度。

#### 属性

- `is_init`: (只读) 机器人是否已初始化。

#### 使用示例

```python
import robot_py
robot = robot_py.RobotInterface("config/robot.yaml")
robot.init_motors()
robot.apply_action([0.0] * 23)
```
