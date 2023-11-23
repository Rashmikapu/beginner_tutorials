----READ ME-----

## 1.) Install docker 

For native Ubuntu :
```
sudo apt install docker.io 

sudo usermod  -aG docker $USER   # add yourself to the "docker" group
newgrp docker  # or logout and re-login completely (not just opening a new terminal)
```

# Verify installation (for both native and WSL2 Ubuntus):

```
sudo service docker start
sudo service docker status
```

# For Ubuntu > 18

```
sudo apt install python3-rocker
```

# Verify by :

```
rocker --version
rocker alpine echo "hello-world"
```

## 2.) Install and modify ROS2 Humble Docker image

# terminal 1:

# download ROS 2 image
```
docker pull osrf/ros:humble-desktop-full

# Run ROS 2 container
rocker osrf/ros:humble-desktop-full bash

# inside ROS 2 container, install packages
apt update;
apt -y install terminator
apt -y install ros-humble-gazebo-ros-pkgs 
apt -y install ros-humble-turtlebot3*
apt -y install python3-colcon-common-extensions
apt -y install ros-humble-turtlebot4-desktop

apt clean all
```

# terminal 2:
Open another terminal and run the command below to save a snapshot of the container.

```
# get the ID of the container running on terminal 1
# assume there is only on container running
CONTAINER_ID=$(docker ps --format {{.ID}})
echo $CONTAINER_ID

# Save the Docker image snapshot 
docker commit $CONTAINER_ID my-docker2-humble
```

The modified docker image is now called my-docker2-humble.


## Tutorials
- Follow these ![tutorial](http://docs.ros.org/en/humble/Tutorials.html) to understand the basics of ROS2. If you have installed the ROS2 from source, the ros2 setup path will be your 
```
. <installation directory>/install/setup.bash
```
- Source installation installs turtlesim and other packages. No need to use the commands listed in the above tutorial. 
- `colcon` can be installed by running the command:
```
sudo apt install python3-colcon-common-extensions
```

## Clone and Build the package
- Clone this repository in your ROS2 Workspace /src folder. It should create beginner_tutorials folder in your /src.
```
git clone git@github.com:Rashmikapu/beginner_tutorials.git
```

Make sure that your terminal is sourced
- Run the below commands:
```
cd beginner_tutorials
rosdep install -i --from-path src --rosdistro humble -y
cd ../.. # It should take your present working directory to <ROS2_workspace>
colcon build
. install/setup.bash
```

# Commands to source a new Terminal window
- When a new terminal is opened, please source your terminal using
```
. <ros2_installation_directory>/install/setup.bash
. install/setup.bash
```

# Run Publisher-Subscriber
Follow the below instructions to run the simple publisher and subscriber package.
- Run the publisher
```
ros2 run beginner_tutorials talker
```
- Open a new terminal
- Source it
- Run the subscriber
```
ros2 run beginner_tutorials listener
```


# Run Server, Client-Publisher, Subscriber

Follow the below instructions to run the simple server, client, publisher and subscriber.

## Run the publisher
```
ros2 run beginner_tutorials server
```

Open a new terminal
Source it
Run the client-publisher

```
ros2 run beginner_tutorials client
```

Open a new terminal
Source it
Run the subscriber

```
ros2 run beginner_tutorials listener
```
These nodes use /topic as a topic
Service name is change_string


# Run Publisher and Subscriber using Parameters

Follow the below instructions to run the simple publisher and subscriber that uses parameters to change publishing frequency with different log levels.


# Run the publisher
```
ros2 run beginner_tutorials param_pub
```
This node uses /chatter topic with a parameter name "freq" of type double. Default value is 1.0 (Hz).
To change the frequency, use the below command in a new terminal:

```
ros2 param set /param_publisher freq <frequency value (double)>
```
Example:
```
ros2 param set /param_pub freq 5.0
```
    
Open a new terminal
Source it
Run the Subscriber

```
ros2 run beginner_tutorials param_sub
```
# Using a launch file to run the publisher, subscriber

## Type the below command to launch publisher and subscriber with default frequency (2Hz)

```
ros2 launch beginner_tutorials launch.py
```

## To set a different frequency, open a new terminal and run the below command:

```
ros2 param set /param_pub freq <frequency value (double)>
```

Example:
```
ros2 param set /param_pub freq 5.0
```
## Type the below command to launch publisher and subscriber with user defined frequency

```
ros2 launch beginner_tutorials launch.py freq:=<user defined frequency (double)>
```

Example:
```
ros2 launch beginner_tutorials launch.py freq:=201.0
```

# Using rqt_console to visualize the log messages:

## Run the below command in a new terminal

```
ros2 run rqt_console rqt_console
```


# Publish a static transform using ROS2 TF2

Run the static transform publisher that publishes the data to /topic and also converts the static transformation between world frame and child frame.
   
## To run the node, use the below command

```
ros2 run beginner_tutorials tf_pub talk 0.5 0.8 1 2 3 1
```

## Use the tf2_ros to output the transformation to the console.
```
ros2 run tf2_ros tf2_echo world talk
```
or

```
ros2 topic echo /tf_static

```

## To save the frames and their relation in the pdf, run the below command
```
ros2 run tf2_tools view_frames
```

## run the test and look at the output: (after building package)

```
colcon test --packages-select beginner_tutorials
cat log/latest_test/beginner_tutorials/stdout_stderr.log
```

check the return status:

```
colcon test-result --verbose --test-result-base build/beginner_tutorials
echo $?
```

## Static Code Analysis
# cpplint
Run the below command from inside the package folder 'beginner_tutorials'
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp &> ./results/cpplint.txt
```
# cppcheck
Run the below command from the package folder 'beginner_tutorials'
```
cppcheck --enable=all --std=c++17 src/*.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ./results/cppcheck.txt
```
