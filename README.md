# 🤖 Chopper-Bot (C1-10P) - Utility Droid

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![Hardware](https://img.shields.io/badge/Hardware-NVIDIA%20Jetson%20Orin%20Nano-76b900.svg)](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
[![License](https://img.shields.io/badge/License-Apache_2.0-orange.svg)](https://opensource.org/licenses/Apache-2.0)

Chopper-Bot is an open-source, Star Wars-inspired autonomous utility droid designed for household clutter retrieval. It leverages the **NVIDIA Jetson Orin Nano** for high-performance edge AI and **ROS 2 Humble** for modular robotics control.

> _"A grumpy droid for a messy house."_

---

## 🛠 Project Vision

This project bridges the gap between hobbyist RC builds and professional "Physical AI" robotics.

- **Phase 1:** Core logic, Spektrum DSMX integration, and 2020 Aluminum skeleton.
- **Phase 2:** Vision-Language Model (VLM) implementation for "clutter" recognition.
- **Phase 3:** Full C1-series aesthetic shell and 6-DOF manipulator arm printed on a **Bambu Lab H2C**.

---

## 🏗 Tech Stack

- **Brain:** NVIDIA Jetson Orin Nano (8GB)
- **Software:** ROS 2 Humble inside Docker (ARM64)
- **Vision:** Luxonis OAK-D Lite (Depth + AI)
- **Control:** Spektrum DSMX Satellite (UART)
- **Actuation:** Feetech STS3215 Serial Bus Servos
- **Fabrication:** 2020 Aluminum Extrusion + Bambu Lab H2C Production

---

## 🚀 Getting Started

### Prerequisites

- **Docker** and **Docker Compose** installed.
- (Recommended) **Cursor AI** or **VS Code** with the Dev Containers extension.

### Quick Start (Dev Container)

1. Clone the repository:
   ```bash
   git clone https://github.com/jameymcelveen/chopper-bot.git
   cd chopper-bot
   ```
