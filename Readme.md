# TurtleSCAN 3D XYZRGB Reconstruction

This repository contains the description of a system based on Turtlebot platform and a 2D Hokuyo LiDAR  to map an indoor navigation environment, generating a Point Cloud reconstruction of  with six dimensions of information: X, Y, Z, R, G and B.

## Folders and files

Repository folders and files:

```
/yourRoot    -path
  |--  Readme.md
  |-- src # source codes
	|-- Python source codes     # main source codes
		|-- fixed_no_color.py # Point cloud generation: fixed without color
		|-- scan_and_go_no_color.py # Dynamic reconstruction without color for the scanning mode: "scan and go"
		|-- stop_and_scan_no_color.py # Dynamic reconstruction without color for the scanning mode: "stop and scan"
		|-- scan_and_go_color.py # Dynamic color reconstruction for the scanning mode: "scan and go"
		|-- stop_and_scan_color.py # Dynamic color reconstruction for the scanning mode: "stop and scan"
		|-- send_command.py # Manual sending of Command
  |-- data
    |-- pointcloud_1.txt # Database file for the "scan and go" point cloud without color
    |-- pointcloud_2.txt # Database file for the "stop and scan" point cloud without color
    |-- pointcloud_3.txt # Database file for color point cloud "scan and go"
    |-- pointcloud_4.txt # Database file for color point cloud "stop and scan"
```

## Hardware requirements

- Turtlebot 2
- Dynamixel AX12A servomotor
- LOGITECH C920S HD PRO RGB Camera
- HOKUYO RangeFinder URG-04LX-UG01

## Software requirements

- Ubuntu 16.04.6 LTS (Xenial Xerus)
- ROS distribution: ROS Kinetic Kame [http://wiki.ros.org/kinetic](http://wiki.ros.org/kinetic)
- ROS nodes/drivers libraries:
	-   [Turtlebot](http://wiki.ros.org/turtlebot_bringup?distro=kinetic)
	-  [Dynamixel servomotor](http://wiki.ros.org/dynamixel)
	-  [Hokuyo](http://wiki.ros.org/urg_node)
	-  [Camera](http://wiki.ros.org/usb_cam)
- Python 3.6 in this case I used [Visual Studio Code](https://code.visualstudio.com/)
-   OpenCV 4

## ROS Installing.

For ROS install you must follow the next steps:

-   [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)

or follow the video:

-   [https://www.youtube.com/watch?v=36O6OGOJG1E](https://www.youtube.com/watch?v=36O6OGOJG1E)

## Use ROS

In a terminal you should run the ROS kernel with

`roscore`

in other tabs the following commands are executed, each in a separate terminal

- `roslaunch turtlebot_bringup minimal.launch`
- `roslaunch control_dynamixel_ax12 controller_manager.launch`
- `roslaunch control_dynamixel_ax12 start_motor_controller.launch`
- `roslaunch usb_cam usb_cam test.launch`
- `rosrun urg_node urg_node`

To execute each program:

`rosrun my_pack source_code`

## Authors


-   Julián R. Cháux - ([jchaux@misena.educo](mailto:jchaux@misena.educo))
-  Harold F. Murcia - ([www.haroldmurcia.com](http://www.haroldmurcia.com/))

** **
### License

This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or distribute this software, either in source code form or as a compiled binary, for any purpose, commercial or non-commercial, and by any means.

In jurisdictions that recognize copyright laws, the author or authors of this software dedicate any and all copyright interest in the software to the public domain. We make this dedication for the benefit of the public at large and to the detriment of our heirs and successors. We intend this dedication to be an overt act of relinquishment in perpetuity of all present and future rights to this software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to  [http://unlicense.org](http://unlicense.org/)
