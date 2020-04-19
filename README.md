# Robonaldo

## Installation

To build everything other than robonaldo_vision just run:
```
sudo ./install.sh
```
Restart the terminal for the changes to take effect. This should let you run catkin_make with no problems.

### Camera Installation

1. If this is on a Jetson with Jetpack 4.3 that was freshly installed, run
```
sudo ln -s /usr/include/opencv4 /usr/include/opencv
```
2. Install the ZED camera software here: [https://www.stereolabs.com/developers/release/](https://www.stereolabs.com/developers/release/).
3. Make the sub ports not require root: [https://github.com/LairdCP/UwTerminalX/wiki/Granting-non-root-USB-device-access-(Linux)](https://github.com/LairdCP/UwTerminalX/wiki/Granting-non-root-USB-device-access-(Linux))

## Nodes
* Camera Node
* Arduino Node
* Decision Node
* Vision Processing Node
* Simulation Node
* Odometry Node
* Desktop Node

# Running ROS

`cd` into repository and run:
```
catkin_make
```


In one Terminal window: `roscore`

In other Terminal windows:

```
source devel/setup.bash 
rosrun robonaldo <node_name>
```

For example, run `roscore` on one terminal, and in another terminal run:
```
source devel/setup.bash 
rosrun robonaldo vision_processing.py
```
(under the `/robonaldo_vision` directory)

# Running on Arduino


http://wiki.ros.org/rosserial_arduino/Tutorials/Blink


`roscore`

`rosrun rosserial_python serial_node.py /dev/ttyACM0`

`rostopic pub toggle_led std_msgs/Empty --once` -- To toggle once

Run `rosrun robonaldo talker` to simulate ROS broadcast


# Running Gaze-bo

Launch Gaze-bo
```
source devel/setup.bash
roslaunch ronobaldo_gazebo robonaldo_gazebo.launch
```


# Other

`rospack list`
