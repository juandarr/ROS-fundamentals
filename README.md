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

## Information about the type of message


  519  rostopic info /turtle1/cmd_vel 
  520  rosmsg info geometry_msgs/Twist
  521  rosed geometry_msgs Twist.msg 
  522  echo $EDITOR
  523  nano ~/.bashrc
  524  source ~/.bashhrc
  525  source ~/.bashrc
  526  echo $EDITOR
  527  rosed geometry_msgs Twist.msg 
  528  rosmsg info geometry_msgs/Vector3
  529  rosed geometry_msgs Vector3
  530  rosed geometry_msgs Vector3.msg
  531  clear
  532  ls
  533  history 10
  534  history 20
  535  history 30
  536  history 30 >> commands.md
