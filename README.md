<div align="center">

# Autonomous RF-Mapping & Cognitive Radio Rover

**High-resolution electromagnetic spatial analysis powered by ROS2, SDR, and Edge AI.** <br>
*Developed for **Jugend forscht 2026/2027***

<!-- Badges -->
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?style=flat-square&logo=ros)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=flat-square&logo=python&logoColor=white)](https://www.python.org/)
[![Edge AI](https://img.shields.io/badge/Edge_AI-Google_Coral-F9AB00?style=flat-square&logo=google)](https://coral.ai/)
[![Hardware](https://img.shields.io/badge/Revision-r0.2-2ea44f?style=flat-square)]()

<br>

<!-- Rev 0.2 Hero Render -->
<img src="assets/render_v02.png" alt="Rover Rev 0.2 CAD Render with OAK-D Lite and TPU Bumper" width="97%" />

<br><br>

<!-- Rev 0.1 Render and 360 GIF side by side -->
<img src="assets/render.png" alt="Rover Rev 0.1 CAD Render" width="48%" />
<img src="assets/360-spin.gif" alt="Rover 360 Spin" width="48%" />

</div>

## Overview

Instead of relying on basic signal strength (RSSI), the system utilizes a **HackRF One** Software Defined Radio (SDR) and a **Google Coral Edge TPU** (M.2) to capture, process, and semantically classify broadband RF protocols (Wi-Fi, BLE, Zigbee, LoRa, etc.) in real-time via a custom INT8-quantized Convolutional Neural Network (CNN).

With **Revision 0.2 (r0.2)**, the rover integrates a **Luxonis OAK-D Lite** stereo depth camera for 3D obstacle avoidance and visual-spatial sensor fusion, protected by a custom 3D-printed **TPU 95A shock-absorbing front bumper**.

## Key Features

- **Autonomous 2D/3D SLAM:** Mapping using **Cartographer ROS** driven by Direct-Time-of-Flight LiDAR scans, **OAK-D Lite** stereo depth perception, and BNO085 IMU data.
- **Precision Kinematics & Chassis Protection:** Custom multi-deck chassis driven by TMC2209-stepped NEMA 14 motors with zero-backlash metal GT2 pulleys and a flexible **TPU 95A front bumper** to protect the optical array.
- **Cognitive Radio Pipeline:** Real-time extraction of Short-Time Fourier Transform (STFT) spectrograms from raw IQ data, combined with 360° in-situ antenna radiation pattern calibration.
- **Edge AI Inference:** Protocol classification using a custom TensorFlow Lite model accelerated by a Google Coral Edge TPU (and on-device VPU depth processing via OAK-D Lite).
- **Spatial Heatmapping:** Overlaying classified RF protocol data and synthetic Angle-of-Arrival (AoA) vectors directly onto 2D/3D occupancy grids.

## System Architecture

### Current Hardware Stack (Rev. 0.2)
| Component | Description |
| :--- | :--- |
| **Compute** | Raspberry Pi 5 (Target upgrade: NVIDIA Jetson Orin Nano Super) |
| **Edge Accelerators** | Google Coral M.2 TPU (PCIe) + Intel Movidius Myriad X VPU (on OAK-D Lite) |
| **RF Frontend** | HackRF One + UWB Omni-Antenna |
| **Spatial & Vision Sensors** | RPLidar C1 + Luxonis OAK-D Lite (Stereo Depth) + BNO085 (9-DOF IMU) |
| **Drive & Motor Control** | Custom ESP32-S3 + TMC2209 Driver Board, NEMA 14 Steppers with Metal GT2 Pulleys |
| **Chassis & Protection** | Custom Multi-Deck Frame with 3D-Printed **TPU 95A Front Impact Bumper** |

### Software Stack
- **OS & Middleware:** Ubuntu 24.04 LTS, ROS2 Jazzy
- **SLAM, Vision & Navigation:** Cartographer ROS, Nav2, DepthAI ROS
- **Signal Processing:** GNU Radio, SciPy, NumPy (SigMF standard)
- **Machine Learning:** TensorFlow 2, Keras, Edge TPU Compiler

### Project Status & Roadmap (Work in Progress)

| Component / Milestone | Progress | Current Status & Next Steps |
| :--- | :--- | :--- |
| **Rev. 0.1 Physical Prototype** | ![100%](https://geps.dev/progress/100) | **Completed:** Initial chassis build and mechanical validation. |
| **Rev. 0.2 Physical Prototype** | ![85%](https://geps.dev/progress/85) | **Finalizing Build:** Integrated OAK-D Lite mount, custom TPU 95A camera bumper, and upgraded to metal GT2 pulleys to eliminate backlash. |
| **Autonomous Navigation (Nav2 & SLAM)** | ![95%](https://geps.dev/progress/95) | **Finalizing:** 2D LiDAR SLAM, DepthAI integration & sub-0.2ms telemetry link validated; fine-tuning odometry with metal pulleys. |
| **RF Signal Classification (CNN & Calibration)** | ![20%](https://geps.dev/progress/20) | **In Progress:** Pipeline architecture set up; 360° in-situ antenna calibration routine using 2.4 GHz CW reference source underway. |

## Sponsors & Partners
This project is being developed for the **Jugend forscht** competition. We are actively looking for hardware and compute partners (specifically for our edge-compute upgrade). A huge thank you to the companies supporting the next generation of engineers:

## Contact

**Team:**
- **Ben Bluhm** – Lead Developer
- **Laura Übele** – Developer
- **Miriam Hohenberger** – Developer
- **Email:** [hello@54lab.dev](mailto:hello@54lab.dev)

**Get in Touch & Follow the Build:**

[![Email](https://img.shields.io/badge/Email-hello%4054lab.dev-blue?style=for-the-badge&logo=minutemailer&logoColor=white)](mailto:hello@54lab.dev)
[![Instagram](https://img.shields.io/badge/Instagram-%4054lab.dev-E4405F?style=for-the-badge&logo=instagram&logoColor=white)](https://www.instagram.com/54lab.dev/)

*Follow [@54lab.dev](https://www.instagram.com/54lab.dev/) on Instagram for regular build logs, 3D printing updates, and live testing videos!*
