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

## Packages and Catkin workspaces

A catkin workspace is an upper level directory where a series of nodes, services, dependencies and more are stored, normally as subdirectories.

### How to create a catkin workspace
First of all, create the root directory. This will work as the workspace for the packages you intend to create:

```bash
mkdir -p ~/catkin_ws/src
```

Go to the source directory,
```bash
cd src
```

and initialize the catkin workspace with:
```bash
catkin_init_workspace
```

This will create a symlink to the master cmakelist file in `/opt/ros/DISTRIBUTION/share/catkin/cmake/toplevel.cmake`, where DISTRIBUTION is your ROS distribution. In my case I am working on Melodic with Ubuntu 18.04.

Now go back to the top level directory, and run the final command

```bash
catkin_make
```

This completes the process of creation of a catkin workspace. 

### Add packages

Lets use an already created package to speed up the process.
Change directory to the source folder and clone the sample package:
```bash
cd ~/catkin_ws/src

git clone https://github.com/udacity/simple_arm_01.git simple_arm
```

Now all we have to do is to go back to the top level directory and run catkin make:

```bash
cd ~/catkin_ws

catkin_make
```

### Manage inter-package dependencies

Catkin packages have two types of dependencies: build and run dependencies. Sometimes when you execute the package you'll find dependecies errors in runtime. One way to solve this is to check the log file and see what is the missing package. Then you can install it in your distro. We are assuming the missing package is controller manager.

```bash
sudo apt-get install ros-melodic-controller-manager
```

Another way is to source the workspace, and run `rosdep` to install any required dependency that is yet to be installed. First, check for missing dependencies:

```bash
rosdep check simple_arm
```

If there are any dependencies missing, install them:

```bash
rosdep install -i simple_arm
```

And you are all set!

### How to succesfully compile

Make sure you have compiled the whole workspace:

 ```bash
cd ~/catkin_ws
catkin_make
```

Then source the workspace,

```bash
source devel/setup.bash
```

Now we can run the package:

```bash
roslaunch simple_arm robot_spawn.launch
```

## Writing ROS Nodes

The following descriptions will include information about how to implement a publisher, service and subscriber using Python. Links about where to get additional information will be provided.

### ROS publish
Publishers allow a node to send messages to a topic so that data from the node can be used in different parts of ROS. In python the instructions used to create a node are as follows:

```python
pub1 = rospy.Publisher("/topic_name", message_type, queue_size=size)
```

* `/topic_name` is the name of the topic the node will be publishing to.
* `message_type` is the type of message sent to the topic
* `queue_size` is used to define whether the publisher will be syncronous or asyncronous. When `queue_size` is not defined or none, the publisher is syncronous, this means that when the topic is being published to a different publisher the current publisher will be blocked. When a `queue_size` value is defined, it will store messages in a buffer until it is full, then will try to free the buffer as soon as the topic is available to receive messages. This is also know as the asyncronous mode of the publisher.

Once the publisher has been created, the following command can be used to publish a `message`:

```python
pb1.publish(message)
```

[Here](http://docs.ros.org/kinetic/api/rospy/html/rospy.topics.Publisher-class.html) you will find more info about ROS publishers.


#### Basic steps

##### Add the scripts directory

In this example we have a package named `simple_arm`. To create nodes in python we have to create a folder called `scripts` in its root. 

```bash
cd ~/catkin_ws/src/simple_arm
mkdir scripts
```

##### Creating a new script
Now go to the scripts folder, and create a new node file:

```bash
cd scripts
touch arm_mover
```

Now you need to give it executable permissions:

```bash
chmod u+x arm_mover
```  

Now define the personalized python code inside the file. Lets assume we already wrote the code.

##### Make, source file and rosrun

Go back to the workspace upper level:

```bash
cd ~/catkin_ws
catkin_make
```
Source the setup.bash script and finally you can run the node:

```bash
source devel/setup.bash
rosrun simple_arm arm_mover
```

### ROS services

A ROS service allows request/responde communication between nodes. The basic instruction used to define a service in Python is as follows:

```python
service = rospy.Service('service_name', serviceClassName, handler)
```

* `service_name` is the name of the service.
* `serviceClassName` is the name of the class of service, this corresponds to a `srv` file that will define the kind of message(s) sent in the `request` and the `response` separated by three consecutive dashes (`---`).
* `handler` is a method or function used to handle a request. It must return a messageClass compatible with the expected one as response. 

A service can be called directly from console or defining a proxy service as follows:

```python
service_proxy = rospy.ServiceProxy('service_name', serviceClassName)
```

Once the ServiceProxy has been created we can send requests as follows:

```python
msg = serviceClassNameRequest()
#update msg attributes here to have correct data
response = service_proxy(msg)
```

#### Basic steps

##### Creating a new service definition

We need to create the `srv` folder at the root of the package:

```bash
cd ~/catkin_ws/src/simple_arm
mkdir srv
cd srv
touch GoToPosition.srv
```

Now lets create the definition of the service class. Put the following content inside `GoToPosition.srv`:

```bash
float64 joint_1
float64 joint_2
---
duration time_elapsed
```

##### Modify CMakeLists.txt

In this case we want to add the custom services files in `srv` and add `std_msgs` as dependency:

First of all ensure that the following section is uncommented and complete:

```C
find_package(catkin REQUIRED COMPONENTS
        std_msgs
        message_generation
)
```

Uncomment the `add_services_files()` section:

```C
## Generate services in the 'srv' folder
add_service_files(
   FILES
   GoToPosition.srv
)
```
and finally make sure that `generate_messages()` is uncommented:

```C
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)
```

##### Modify package.xml

Uncomment/add the following sections to the file:

```xml
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>
```

##### Build the package

Once these configurations have been defined, build the package:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

##### Create the node

```bash
cd ~/catkin_ws/src/simple_arm/scripts
touch arm_mover
chmod u+x arm_mover
```

Finally add the contents, and now you can run the node.

##### Launching the project with the new service

Edit the launcher file in the folder `launch` of the package and add the following block of instructions:

```xml
<!-- The arm mover node -->
<node name="arm_mover" type="arm_mover" pkg="simple_arm">
  <rosparam>
    min_joint_1_angle: 0
    max_joint_1_angle: 1.57
    min_joint_2_angle: 0
    max_joint_2_angle: 1.57
  </rosparam>
</node>
```

## ROS subscribe

A subscriber allows a node to receive messages from a topic. Here is the instruction used in Python to define one:

```python
sub1 = rospy.Subscriber("/topic_name", message_type, callback_function)
```

* `/topic_name` is the topic name the node will subscribe to.
* `message_type` is the type of message expected
* `callback_function` is a function that does something when messages are received from the topic. In contrast with the hanlder function in a service, this calllback function doesn't have to return
any specific info.

#### Basic steps

##### Create node look_away

```bash
cd ~/catkin_ws/src/simple_arm/scripts
touch look_away
chmod u+x look_away
```

##### Update the launch file

We want the subscriber to do something when receiving messages. Lets change the launch file to launch the node from the start of a process:

```xml
<!-- The look away node -->
<node name="look_away" type="look_away" pkg="simple_arm"/>
```

After this you can build the project, source the setup file and launch the project. Check that the service/publisher/action are active using `rosnode`, `rostopic`, `rosservice` and their subcommands.