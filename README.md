# Robonaldo

## Installation
1. Install ros melodic with these instructions: [http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. Install the remaining dependencies by running:
```
sudo apt-get install libsdl2-dev ros-melodic-rosserial-arduino ros-melodic-rosserial
sudo ln -s /usr/include/opencv4 /usr/include/opencv
```
3. Make the sub ports not require root: [https://github.com/LairdCP/UwTerminalX/wiki/Granting-non-root-USB-device-access-(Linux)](https://github.com/LairdCP/UwTerminalX/wiki/Granting-non-root-USB-device-access-(Linux))
4. (Optional) Install the ZED camera software here: [https://www.stereolabs.com/developers/release/](https://www.stereolabs.com/developers/release/).

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
caikin_make
```


In one Terminal window: `roscore`

In other Terminal windows:

```
source devel/setup.bash 
rosrun robonaldo <node_name>
```

# Running on Arduino


http://wiki.ros.org/rosserial_arduino/Tutorials/Blink


`roscore`

`rosrun rosserial_python serial_node.py /dev/ttyACM0`

`rostopic pub toggle_led std_msgs/Empty --once` -- To toggle once

Run `rosrun robonaldo talker` to simulate ROS broadcast


# Other

`rospack list`
