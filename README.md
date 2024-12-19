# ROS2-RobotFz-Nix
This is an incompleted code of a robot that Moves and overcomes obstacles, detect ball ... using ROS2 in nixos and micro-ros
My project aimed to create an autonomous robot capable of detecting and interacting with balls, using the ROS2 system. The robot was based on the following architecture:

- Camera and Perception: I used an ESP32-CAM board equipped with a micro-RTSP library for transmitting the video stream. For integration with ROS2, I implemented micro-ROS on the ESP32-CAM, allowing it to act as a publisher of data (video stream, potential sensor data).

- Data Processing: My computer, running ROS2 (Robot Operating System 2 - https://www.ros.org/), received this data via micro-ROS. A Python script, based on computer vision libraries (OpenCV - https://opencv.org/), was responsible for ball detection. Here's a link to a tutorial on how to get started with ROS2: https://docs.ros.org/en/humble/Tutorials.html

- Robot Control: After processing the data, ROS2 created a publisher for movement commands. These commands were then received by the ESP32-CAM, which acted as a subscriber, and controlled the robot's movements.

- Micro-ROS: The implementation of micro-ROS for microcontrollers was based on this documentation: https://micro-ros.github.io/docs/

I chose to use ROS2 to take advantage of its flexibility and tools for robotics, as well as Python for its rapid development speed. The Arduino IDE was used for programming the ESP32 board. I also used the micro-RTSP library for video streaming from the esp32-cam https://github.com/loboris/Micro-RTSP
