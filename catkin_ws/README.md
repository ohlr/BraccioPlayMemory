### ROS connection to Braccio

first thing to do:

Source your ROS environment:
```bash
source /opt/ros/kinetic/setup.bash
```

Run 
```bash
catkin_make
```

Than source this workspace
```bash
source devel/setup.bash
```

## Terminal 1:

```bash
roscore
```


### Terminal 2:
```ruby
	source devel/setup.bash
	cd src/detect_and_collect
	rosrun detect_and_collect parse_and_publish
```

Converts the joint angles to degrees and reduces the message size 

### Terminal 4:
```ruby
    rosrun rosserial_python serial_node.py /dev/ttyACM0
```

Starts the communication between Arduino and Pc.
Change "/dev/ttyACM0" to the port the Port of your Arduino. 
You find this Information in the Arduino IDE, ArduinoIDE>Tools>Port.

### Terminal 5 (optional/Debugging):
```ruby
	rostopic echo joint_array
```

view what is published to the Arduino

```ruby
	rostopic echo joint_states
```

view what is published by the joint_state_publisher (GUI)

```ruby
	rqt_graph
```
