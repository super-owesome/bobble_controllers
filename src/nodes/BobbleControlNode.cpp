#include <ros/ros.h>
#include <bobble_controllers/BobbleBotHw.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "bobble_bot_control_node");
  ros::NodeHandle nh;

  BobbleBotHw bobble_bot;
  controller_manager::ControllerManager cm(&bobble_bot);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Control loop
  double loop_rate;
  ros::NodeHandle pnh("~");
  pnh.param("LoopRate", loop_rate, 1000.0);

  ros::Time prev_time = ros::Time::now();
  ros::Rate rate(loop_rate);

  // Initialize the robot hw
  bobble_bot.init();

  // Run the controller manager
  while (ros::ok())
  {
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    prev_time = time;
    bobble_bot.read();
    ROS_INFO("Looping with period: %f s", period.toSec());
    cm.update(time, period);
    bobble_bot.write();
    rate.sleep();
  }
  return 0;
}

