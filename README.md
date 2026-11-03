<div align="center">

# Autonomous RF-Mapping & Cognitive Radio Rover

**High-resolution electromagnetic spatial analysis powered by ROS2, SDR, and Edge AI.** <br>
*Developed by 54Lab for **Jugend forscht 2026/2027***

<!-- Badges -->
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?style=flat-square&logo=ros)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=flat-square&logo=python&logoColor=white)](https://www.python.org/)
[![Edge AI](https://img.shields.io/badge/Edge_AI-Google_Coral-F9AB00?style=flat-square&logo=google)](https://coral.ai/)


<br>

<!-- Hero Images: Render and 360 GIF side by side -->
<img src="docs/assets/render.png" alt="Rover CAD Render" width="48%" />
<img src="docs/assets/360-spin.gif" alt="Rover 360 Spin" width="48%" />

</div>

## Overview

This repository contains the hardware configurations, ROS2 workspaces, and machine learning pipelines for an autonomous, SLAM-navigated rover designed to map indoor radio frequency (RF) environments. 

Instead of relying on basic signal strength (RSSI), the system utilizes a **HackRF One** Software Defined Radio (SDR) and a **Google Coral Edge TPU** (M.2) to capture, process, and semantically classify broadband RF protocols (Wi-Fi, BLE, Zigbee, LoRa etc.) in real-time via a custom INT8-quantized Convolutional Neural Network (CNN).

## Key Features

- **Autonomous SLAM:** Mapping using **Cartographer ROS** driven by Direct-Time-of-Flight LiDAR scans and BNO085 IMU data.
- **Precision Kinematics:** Custom multi-deck chassis driven by TMC2209-stepped NEMA motors.
- **Cognitive Radio Pipeline:** Real-time extraction of Short-Time Fourier Transform (STFT) spectrograms from raw IQ data.
- **Edge AI Inference:** Protocol classification using a custom TensorFlow Lite model accelerated by a Google Coral Edge TPU.
- **Spatial Heatmapping:** Overlaying classified RF protocol data directly onto 2D LiDAR occupancy grids.

## System Architecture

### Hardware Stack
| Component | Description |
| :--- | :--- |
| **Compute** | Raspberry Pi 5 (1GB) |
| **Edge Accelerator** | Google Coral M.2 TPU (PCIe) |
| **RF Frontend** | HackRF One + UWB Omni-Antenna |
| **Spatial Sensors** | RPLidar C1 + BNO085 (9-DOF IMU) |
| **Motor Control** | Custom ESP32S3 + TMC2209 Driver board |

### Software Stack
- **OS & Middleware:** Ubuntu 24.04 LTS, ROS2 Jazzy
- **SLAM & Navigation:** Cartographer ROS, Nav2
- **Signal Processing:** GNU Radio, SciPy, NumPy (SigMF standard)
- **Machine Learning:** TensorFlow 2, Keras, Edge TPU Compiler

### Sponsors & Partners
This project is being developed for the Jugend forscht competition. We are actively looking for hardware partners. A huge thank you to the companies supporting the next generation of engineers:

## Contact

- **Ben Bluhm** – Lead Developer
- **Laura Übele** – Developer

**Email:** [hello@54lab.dev](mailto:hello@54lab.dev)
