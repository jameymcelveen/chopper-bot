# Chopper-Bot (Public)
- **Architecture:** ROS 2 Humble / Jetson Orin Nano
- **GPU Runtime:** Enabled

## 📍 Resume Here: Hardware Hookup
- **Spektrum Satellite Receiver:** Connect to J41 Header.
    - **VCC:** Pin 1 (3.3V) — **CRITICAL: DO NOT USE 5V**
    - **GND:** Pin 6 or 14.
    - **RX:** Pin 10 (UART2_RX / `/dev/ttyTHS1`).
- **NVIDIA Pinout Reference:** [Official Jetson Orin Nano Header Guide](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/howto.html#id1)

## Current Nodes
- heartbeat_node
- spektrum_node
- motor_node

## Shop TODO List
- [ ] Install NVIDIA Container Toolkit on Orin Nano
- [ ] Solder/Pin-out Spektrum Satellite
- [ ] Run 'make build' on Jetson
