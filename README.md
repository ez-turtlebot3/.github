# Overview
This repo stands as the front page to the [ez-turtlebot3 project](https://github.com/ez-turtlebot3).

The ez-turtlebot3 project achieves the following:
* Optimized autonomous navigation and obstacle avoidance
  * Navigation is currently broken... WIP
* Publishes a ROS 2 topic of measurements from analog pins A0-A5 from the TurtleBot OpenCR microcontroller board
* Streams analog data, audio and video to a remote pc or to a cloud service (AWS or YouTube)

# Context
The following is a list of hardware and software used in developing this project.
## Hardware
* [TurtleBot3 Burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview)
* [Raspberry Pi 4b (8GB)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/specifications/)
* [Raspberry Pi Camera Module v2](https://www.raspberrypi.com/products/camera-module-v2/) and [Raspberry Pi AI Camera](https://www.raspberrypi.com/products/ai-camera/)
## Software
* OS options:
  * For Camera Module v2 and easy, reliable ROS 2 installation, use [Ubuntu 22 LTS](https://releases.ubuntu.com/jammy/)
  * For AI Camera use [Raspberry Pi OS x64](https://www.raspberrypi.com/software/)
* [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
* [Nav2](https://docs.nav2.org/index.html)

# Project composition
The ez-turtlebot3 project is composed of the following repos
1. An analog-enabled fork of ROBOTIS OpenCR: https://github.com/ez-turtlebot3/OpenCR
2. An analog-enabled fork of ROBOTIS turtlebot3: https://github.com/ez-turtlebot3/turtlebot3
3. A ROS 2 node to process the analog data: https://github.com/ez-turtlebot3/ez_analog_processor
4. A collection of scripts to stream audio, video, and video with object detection overlays from the Raspberry Pi: https://github.com/ez-turtlebot3/rpi-av-stream-scripts

Each of these repos contains setup instructions in the README. Therefore, setup instructions for the ez-turtlebot3 project are to visit each of these repos in the order listed above and follow the setup instructions in their READMEs.

# High-level set up instructions
1. Flash the OpenCR board with [analog-enabled firmware](https://github.com/ez-turtlebot3/OpenCR).
2. Build the [analog-enabled turtlebot3](https://github.com/ez-turtlebot3/turtlebot3).
3. Build the [analog processor node](https://github.com/ez-turtlebot3/ez_analog_processor).
4. Prepare your environment to execute [audio and video streaming scripts](https://github.com/ez-turtlebot3/rpi-av-stream-scripts).

# Example use case
In this example we broadcast analog data and object detection video to a remote PC while we teleoperate the TurtleBot3.
## Raspberry Pi setup
1. Turn on the TurtleBot3 and remote access its Raspberry Pi. I use SSH and then I start a tmux session.
2. Launch the TurtleBot3 bringup.
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
3. In a new terminal window, launch the analog processor node.
```bash
ros2 launch ez_analog_processor analog_processor.launch.py
```
4. In a new terminal window, export the remote PC IP address and start the video stream
```bash
export REMOTE_PC_IP=192.168.1.100
python3 ~/rpi-av-stream-scripts/streaming_scripts/pi/stream_object_detection_video_to_pc.py 
```
## Remote PC setup
5. Open the live analog data plot. You'll need to drag the ROS topics you want into the chart area.
```bash
ros2 run plotjuggler plotjuggler
```
6. Open a new terminal window and then open the video feed.
```bash
cd ~/rpi-av-stream-scripts/streaming_scripts/pc
./open_video_stream.sh
```
7. Open a new terminal window and then start teleoperation.
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
8. Drive around.
9. Profit.
10. To exit, CTR+C all the terminal windows this procedure opened on the remote PC and pi.

# Contents of this repo
The purpose of this repo is to stand as the front page for the ez-turtlebot3 project. Aside from this README, there are some other bits and pieces in here that may be useful to other members of the TurtleBot community.
* commands_and_tips: Commands, tips, and notes to copy-paste or reference frequently
* Humble_install_steps: The steps I took to install ROS 2 Humble to Raspberry Pi OS x64
* nav2_params: The Nav2 parameters file for optimized navigation in tight labs and office spaces. The file invokes the MPPI controller and the Smac Hybrid A* Planner.