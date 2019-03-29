# Usings ROS for Robotics: the fundamentals

## start the master process
Open a terminal and issue the command:

```
roscore
```

## Run a node from a package
Open a new terminal tab and follow:

```
rosrun turtlesim turtlesim_node
```
This command will create a new node representing the simulation.

To control the simulation, lets create a new node with teleoperation properties:

```
rosrun turtlesim turtle_teleop_key
```

  515  rosnode list 
  516  rosrun turtlesim turtle_teleop_key
  517  rosnode list
  518  rostopic list
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
