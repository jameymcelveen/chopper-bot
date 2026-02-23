# Chopper-Bot Project Context (Seed File)
- **Target Hardware:** NVIDIA Jetson Orin Nano (8GB)
- **Architecture:** ROS 2 Humble / Docker (ARM64) / Mac Dev Host
- **Naming Convention:** Dash-case for paths/files, underscore for Python packages.
- **Spektrum Logic:** DSMX Satellite via UART (/dev/ttyTHS1), 115200 baud, 16-byte frames.
- **Safety:** Killswitch on Ch5 (Aux1).
- **Current Status:** Spektrum parsing implemented, Motor Node scaffolded.
