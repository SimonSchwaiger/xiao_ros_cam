# xiao_ros_cam
Micro-ROS node that publishes the camera stream of a Seeed Studio XIAO ESP32 embedded board. Uses PlattformIO and the built-in micro-ROS library.

## Getting Started
This project uses PlattformIO [(link)](https://platformio.org/) for the convenience of using their already packaged micro-ROS binaries.

1. Install VSCode
2. Install PlattformIO extension
3. Load this PlattformIO project. This should install the board-level dependencies and libraries for micro-ROS and ESPCam.
4. Set Wifi credentials and the IP of the PC running the micro-ROS daemon in *src/main.cpp*
5. Start the micro-ROS daemon (required to connect the MCU to ROS 2). The easiest way is with the official Docker container: `docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6`
6. Firewall may be blocking the traffic. Disable that temporarily: `ufw disable` (don't forget to re-enable later: `ufw enable`).
7. Flash the ESP. It should connect and start publishing the image stream right away.
