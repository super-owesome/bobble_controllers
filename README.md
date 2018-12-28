# BobbleBot Simulator
> A Gazebo simulation of the self-balancing ROS robot, BobbleBot.

TODO section.
One or two paragraphs about BobbleBot.

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
Potentially do this for multiple platforms via docker containers. CI tests should test these steps.

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
BobbleBot is a real robot built by the robotics team at SOE. Check out the 
[parts list](https://soe/bobble-parts) to learn how to build your own. The controller 
provided in this repository is currently in use by the real BobbleBot. Check out the video 
below to see the controller in action on the real system.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=bg6ksWbVXSk" 
target="_blank"><img src="http://img.youtube.com/vi/bg6ksWbVXSk/0.jpg" 
alt="BobbleBot Testing" width="840" height="380" border="10" /></a>

## Contributing

1. Fork [bobble_controllers](<https://github.com/super-owesome/bobble_controllers/fork>) and [bobble_description](<https://github.com/super-owesome/bobble_description/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

<!-- Markdown link & img dfn's -->
[wiki]: https://github.com/super-owesome/bobble_controllers/wiki
