# 🐢 ez-turtlebot3

> **Enhanced TurtleBot3 with analog sensing and multimedia streaming capabilities**

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-brightgreen)](https://docs.ros.org/en/humble/index.html)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20LTS-orange)](https://releases.ubuntu.com/jammy/)
[![Python](https://img.shields.io/badge/Python-3.8+-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## 🌎 Overview

The **ez-turtlebot3** project extends the standard TurtleBot3 with advanced capabilities:

- 🔧 **Enhanced Navigation**: Autonomous navigation and obstacle avoidance optimized for tight spaces
  > ⚠️ **Note**: Navigation is currently under development (WIP)
- 📊 **Analog Sensing**: Publishes ROS 2 topics with measurements from analog pins A0-A5 on the TurtleBot OpenCR microcontroller
- 📹 **Multimedia Streaming**: Streams analog data, audio, and video to remote PCs or cloud services (AWS/YouTube)

---

## 🛠️ Hardware Requirements

| Component | Model | Link |
|-----------|-------|------|
| **Robot Platform** | [TurtleBot3 Burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) | [📖 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) |
| **Single Board Computer** | [Raspberry Pi 4B (8GB)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/) | [🛒 Buy](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/) |
| **Camera Options** | • [Raspberry Pi Camera Module v2](https://www.raspberrypi.com/products/camera-module-v2/)<br>• [Raspberry Pi AI Camera](https://www.raspberrypi.com/products/ai-camera/) | [📷 Camera Module v2](https://www.raspberrypi.com/products/camera-module-v2/)<br>[🤖 AI Camera](https://www.raspberrypi.com/products/ai-camera/) |

## 💻 Software Stack

### Operating System
- **For Camera Module v2**: [Ubuntu 22.04 LTS](https://releases.ubuntu.com/jammy/) (recommended for easy ROS 2 installation)
- **For AI Camera**: [Raspberry Pi OS x64](https://www.raspberrypi.com/software/)

### Core Software
- [**ROS 2 Humble**](https://docs.ros.org/en/humble/index.html) - Robot Operating System
- [**Nav2**](https://docs.nav2.org/index.html) - Navigation framework

---

## 👷 Setup Instructions

The ez-turtlebot3 project consists of four repositories. Follow them in this order:

| Repository | Purpose |
|------------|-------------|
| [**OpenCR**](https://github.com/ez-turtlebot3/OpenCR) | Flash analog-enabled firmware to the OpenCR board |
| [**turtlebot3**](https://github.com/ez-turtlebot3/turtlebot3) | Build the analog-enabled turtlebot3 package |
| [**ez_analog_processor**](https://github.com/ez-turtlebot3/ez_analog_processor) | Install the analog data processing node |
| [**rpi-av-stream-scripts**](https://github.com/ez-turtlebot3/rpi-av-stream-scripts) | Set up audio/video streaming environment |

Each repository contains detailed setup instructions in its README file.

---

## 🎮 Example Use Case: Teleoperation with Live Data

This example demonstrates broadcasting analog data and object detection video to a remote PC while teleoperating the TurtleBot3.

### 🤖 Raspberry Pi Setup

1. **Power on and connect** to the TurtleBot3's Raspberry Pi via SSH
2. **Launch TurtleBot3 bringup**:
   ```bash
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
3. **Start analog processor** (new terminal):
   ```bash
   ros2 launch ez_analog_processor analog_processor.launch.py
   ```
4. **Configure streaming** (new terminal):
   ```bash
   export REMOTE_PC_IP=192.168.1.100
   python3 ~/rpi-av-stream-scripts/streaming_scripts/pi/stream_object_detection_video_to_pc.py
   ```

### 💻 Remote PC Setup

5. **Open live data visualization**:
   ```bash
   ros2 run plotjuggler plotjuggler
   ```
   > **Tip**: Drag desired ROS topics into the chart area

6. **Start video feed** (new terminal):
   ```bash
   cd ~/rpi-av-stream-scripts/streaming_scripts/pc
   ./open_video_stream.sh
   ```

7. **Begin teleoperation** (new terminal):
   ```bash
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

8. **Drive and explore**! 🎯

9. **Cleanup**: Use `Ctrl+C` to stop all processes on both Pi and PC

---

## 📚 Documentation

- [**OpenCR Setup**](https://github.com/ez-turtlebot3/OpenCR) - Analog firmware installation
- [**TurtleBot3 Build**](https://github.com/ez-turtlebot3/turtlebot3) - Robot platform setup
- [**Analog Processor**](https://github.com/ez-turtlebot3/ez_analog_processor) - Data processing node
- [**Streaming Scripts**](https://github.com/ez-turtlebot3/rpi-av-stream-scripts) - Multimedia streaming

---

## 🤝 Contributing

We welcome contributions! Please see individual repository READMEs for contribution guidelines.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

<div align="center">

**Made with ❤️ for the robotics community**

[![GitHub stars](https://img.shields.io/github/stars/ez-turtlebot3/ez-turtlebot3?style=social)](https://github.com/ez-turtlebot3/ez-turtlebot3)
[![GitHub forks](https://img.shields.io/github/forks/ez-turtlebot3/ez-turtlebot3?style=social)](https://github.com/ez-turtlebot3/ez-turtlebot3)

</div>
