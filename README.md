# BobbleBot Simulator
> A Gazebo simulation of the self-balancing ROS robot, BobbleBot.

One to two paragraph statement about your product and what it does.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=hS7kfhN-8V8" 
target="_blank"><img src="imgs/BobbleBotGazebo.png" 
alt="BobbleBot Simulation" width="840" height="380" border="10" /></a>


## Quick Start

TODO debian based install instructions.

```sh
sudo apt-get install bobble-sim
```

### Keyboard Control
Here's how to control BobbleBot with the keyboard.

Launch the simulation using the command below.
```sh
roslaunch bobble_description run_sim.launch
```

In a separate terminal (with the ROS environment sourced) launch the keyboard control node using the 
command below.
```sh
rosrun bobble_description KeyboardControl
```

The controls are summarized below. The terminal used to launch the keyboard control node must 
have the active focus.
```sh
BobbleBot Keyboard Controller
---------------------------
Activate/Deactivate command:
    Activate/Shutdown: space bar
Moving around:
    Forward : w
    Backward : s
    Left : a
    Right : d
Speed Up/Down: 
    15% Increase: q
    15% Decrease: e
CTRL-C to quit
```

### Joystick Control
The bobble_description package comes with a Joystick control node that is defaulted with a mapping 
that is suitable for an Xbox 1 controller. To use joystick control, follow 
[these instructions](https://www.maketecheasier.com/set-up-xbox-one-controller-ubuntu/) 
to setup your Xbox 1 controller. Next, make sure you have the [ROS joy package installed](http://wiki.ros.org/joy). 
With those two steps out of the way, you can then launch the simulator using the command below.

```sh
roslaunch bobble_description run_sim_with_joystick.launch
```

The default controls are depicted below:

![Joystick Controls](imgs/JoystickControls.png)


## Development setup

TODO section.
Describe how to install all development dependencies and how to run an automated test-suite of some kind. 
Potentially do this for multiple platforms via docker containers.

```sh
mkdir -p bobble_workspace/src
cd bobble_workspace/src
catkin init
git clone https://github.com/super-owesome/bobble_controllers.git
git clone https://github.com/super-owesome/bobble_description.git
catkin build
catkin run_tests
```

## Build Your Own BobbleBot
BobbleBot is a real robot built by the robotics team at SOE. The controller provided in 
this repository is currently in use by the real BobbleBot. See video below for this 
controller in action. 

<a href="http://www.youtube.com/watch?feature=player_embedded&v=bg6ksWbVXSk" 
target="_blank"><img src="http://img.youtube.com/vi/bg6ksWbVXSk/0.jpg" 
alt="BobbleBot Testing" width="840" height="380" border="10" /></a>

Check out the [parts list](https://soe/bobble-parts) to learn how to build your own.

## Release History

* 0.2.1
    * CHANGE: Update docs (module code remains unchanged)
* 0.2.0
    * CHANGE: Remove `setDefaultXYZ()`
    * ADD: Add `init()`
* 0.1.1
    * FIX: Crash when calling `baz()` (Thanks @GenerousContributorName!)
* 0.1.0
    * The first proper release
    * CHANGE: Rename `foo()` to `bar()`
* 0.0.1
    * Work in progress

## Meta

Your Name – [@YourTwitter](https://twitter.com/dbader_org) – YourEmail@example.com

Distributed under the BSD license. See ``LICENSE`` for more information.

[https://github.com/super-owesome](https://github.com/super-owesome/)

## Contributing

1. Fork [bobble_controllers](<https://github.com/super-owesome/bobble_controllers/fork>) and [bobble_description](<https://github.com/super-owesome/bobble_description/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

<!-- Markdown link & img dfn's -->
[wiki]: https://github.com/super-owesome/bobble_controllers/wiki
