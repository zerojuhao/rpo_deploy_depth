# ATOM01 ROS2 Deploy (Parkour)

[![ROS2](https://img.shields.io/badge/ROS2-Humble-silver)](https://docs.ros.org/en/humble/index.html)
![C++](https://img.shields.io/badge/C++-17-blue)
[![Linux platform](https://img.shields.io/badge/platform-linux--x86_64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Linux platform](https://img.shields.io/badge/platform-linux--aarch64-orange.svg)](https://releases.ubuntu.com/22.04/)

[English](README.md) | [中文](README_CN.md)

## 概述

本仓库在 [Roboparty/roboparty_deploy](https://github.com/Roboparty/roboparty_deploy) 的基础上，增加了基于深度相机的 **Parkour** 部署功能。

详细的环境配置、硬件连接、电机标零、手柄控制、Python SDK 等通用部署说明，请参考上游仓库：  
**[https://github.com/Roboparty/roboparty_deploy](https://github.com/Roboparty/roboparty_deploy)**

开源地址：[https://github.com/zerojuhao/rpo_deploy_depth](https://github.com/zerojuhao/rpo_deploy_depth)

**维护者**: 刘志浩
**联系方式**: <ZhihaoLiu_hit@163.com>

## Parkour 部署

本仓库增加了基于深度相机的 Parkour 部署功能。启动 Parkour 程序：

```bash
./tools/start_robot_depth.sh
```

## 配置文件

配置文件位于 `src/inference/config/` 目录下。主要文件：

| 文件 | 说明 |
|---|---|
| `inference_depth.yaml` | Parkour 部署配置 |
| `robot.yaml` | 硬件配置（电机、IMU、CAN 等） |
| `realsense.yaml` | RealSense 深度相机设置 |
