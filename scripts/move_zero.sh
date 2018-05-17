#!/bin/bash

rostopic pub /wam/computed_torque_controller/command \
trajectory_msgs/JointTrajectoryPoint \
"[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" \
"[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" \
"[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" \
"[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" \
"[0.0, 0.0]" -1
