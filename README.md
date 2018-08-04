## Getting Started With Bobble Balance Sim

### Install Packages

```shell
sudo apt-get install ros-lunar-ros-control
sudo apt-get install ros-lunar-ros-controllers
sudo apt-get install ros-lunar-gazebo-ros-pkgs
sudo apt-get install ros-lunar-gazebo-ros-control
```

#### Special note
I had to also do this in order to get the BobbleControlNode to compile:
```shell
sudo ln -s /opt/ros/lunar/include/pluginlib/class_loader.h /opt/ros/lunar/include/pluginlib/class_loader.hpp
```

### Add Bobble Sim and Controller Packages
Clone the following two repositories into your BobbleBot workspace.

```shell
cd BobbleBot/src
git clone ssh://git@hardworkin-man.com:23/bots/bobble_bot/bobble_controllers.git
git clone ssh://git@hardworkin-man.com:23/bots/bobble_bot/bobble_description.git
catkin build
```

###  Configure System For Real-Time
Create a PAM file. See reference section 4
[RealTimeJointControllers.pdf](/uploads/d3596e5602a8ce6ba3f4cb146c7b1120/RealTimeJointControllers.pdf)

```shell
sudo vi /etc/security/limits.d/bobble_controllers.conf
```
Paste in these contents (substitute in your linux username)

```shell
username soft cpu      unlimited
username -    rtprio   99
username -    nice     -20
username -    memlock  unlimited
```

Log out and then log back in for the above settings to take effect.

###  Run the Sim
```shell
catkin clean
catkin build
source devel/setup.bash
roslaunch bobble_controllers bobble_balance.launch
```
