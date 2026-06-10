# ATOM01 ROS2 Deploy (Parkour)

[![ROS2](https://img.shields.io/badge/ROS2-Humble-silver)](https://docs.ros.org/en/humble/index.html)
![C++](https://img.shields.io/badge/C++-17-blue)
[![Linux platform](https://img.shields.io/badge/platform-linux--x86_64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Linux platform](https://img.shields.io/badge/platform-linux--aarch64-orange.svg)](https://releases.ubuntu.com/22.04/)

[English](README.md) | [中文](README_CN.md)

## Overview

This repository extends [Roboparty/roboparty_deploy](https://github.com/Roboparty/roboparty_deploy) with depth-camera-based **Parkour** deployment support.

For detailed environment setup, hardware connection, motor calibration, gamepad control, Python SDK, and other general deployment instructions, please refer to the upstream repository:  
**[https://github.com/Roboparty/roboparty_deploy](https://github.com/Roboparty/roboparty_deploy)**

Open-source repository: [https://github.com/zerojuhao/rpo_deploy_depth](https://github.com/zerojuhao/rpo_deploy_depth)

**Maintainer**: Zhihao Liu
**Contact**: <ZhihaoLiu_hit@163.com>

## Parkour Deployment

This repository adds depth-camera-based Parkour deployment. To start the Parkour program:

```bash
./tools/start_robot_depth.sh
```

## Configuration

Configuration files are located in `src/inference/config/`. Key files:

| File | Description |
|---|---|
| `inference_depth.yaml` | Parkour deployment configuration |
| `robot.yaml` | Hardware configuration (motors, IMU, CAN, etc.) |
| `realsense.yaml` | RealSense depth camera settings |
