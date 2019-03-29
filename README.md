# Usings ROS for Robotics

## The fundamentals

### Start the master process
Open a terminal and issue the command:

```
roscore
```

### Run a node from a package
Open a new terminal tab and follow:

```
rosrun turtlesim turtlesim_node
```
This command will create a new node representing the simulation.

To control the simulation, lets create a new node with teleoperation properties:

```
rosrun turtlesim turtle_teleop_key
```

### List of nodes
Issue the following command in a terminal:

```
rosnode list
```

You will see something like this:
```
/rosout
/teleop_turtle
/turtlesim
```

### List of topics

```
rostopic list
```

### Information about the topics

```
rostopic info /turtle1/cmd_vel
```

An example of the output on the terminal promt is as follows:

```
Type: geometry_msgs/Twist

Publishers:
 * /teleop_turtle (http://fauxtales:34597/)

Subscribers:
 * /turtlesim (http://fauxtales:44061/)
```

### Information about the type of message

Lets explore more about the type of message used in `/turtle1/cmd_vel` rostopic. In the terminal prompt issue the following command:
```
rosmsg info geometry_msgs/Twist
```

You will see something like the following output:
```
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

Sometime the information you will see here won't be enough to understand the associations and purpose of the topics. You can also explore the original
message file using `rosed`.

### Exploring message files with rosed

First you need to set up the editor. By default the one used is VIM. Use the following instruction to add a different text editor (nano in this example). Open 
the `~/.bashrc` file and add `export EDITOR='nano -w'` at the end of the file. Now you can source the bash file `source ~/.bashrc` and verify that the editor variable has been defined accordingly: `echo $EDITOR`. You should get the output `nano -w`.

Once the setup has been completed, you can issue the following command to see contents of the message:

```
rosed geomtry_msgs Twist.msg
```

In this case, the output is:
```
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
```
 Which gives additional info with the comment on the first line.


### Echo messages on a topic

Sometimes is useful to see the message being published in real time. Use the following command to do that:

```
rostopic echo /turtle1/cmd_vel
```





