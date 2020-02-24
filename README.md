# Robonaldo

## Nodes
* Camera Node
* Arduino Node
* Decision Node
* Vision Processing Node
* Simulation Node
* Odometry Node
* Desktop Node

# Running ROSS

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
