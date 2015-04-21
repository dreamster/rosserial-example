# rosserial-example
Arduino Example using rosserial

## Installing

This example uses the [rosserial](http://wiki.ros.org/rosserial) package for arduino. 

After installing the package using 

```
apt-get install ros-indigo-rosserial-arduino ros-indigo-rosserial-python
```

you need to configure your Arduino IDE following [this tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup).

## Running the example

In order to run this example you need to launch the roscore in a new terminal window:

```
roscore
```

Next, run the rosserial client application that forwards your Arduino messages to the rest of ROS. Make sure to use the correct serial port:

```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
```

At this point, you can look at the distance measures from your Dreamster by launching a new terminal window and entering :

```
rostopic echo /ultrasound
```

Or you can move your Dreamster by publishing velocity commands, for example moving it forward with:

```
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0 }}' -r 1
```

## Visualizing ultrasonic sensor

To visualize the ultrasonic sensor using RViz, first, run RViz

```
rosrun rviz rviz
```

Then run a static transform between ```/map``` and ```/ultrasound``` using:

```
rosrun tf static_transform_publisher 0 0 0.2 0 0 0 nav ultrasound 100
```

This will allow for the tf tree to be correctly populated. Then, on RViz, click on ```Add``` -> ```By Topic``` and select ```Range```. Now you can see the ultrasonic beam on RViz. 




