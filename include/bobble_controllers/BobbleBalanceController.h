/******************************************************************************
 * Document this if it actually works...
 * Mike
*******************************************************************************/

#ifndef BOBBLE_CONTROLLERS_BOBBLE_BALANCE_CONTROLLER_H
#define BOBBLE_CONTROLLERS_BOBBLE_BALANCE_CONTROLLER_H

#include <cstddef>
#include <vector>
#include <string>

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Core>

namespace bobble_controllers
{
	class BobbleBalanceController: public controller_interface::
		Controller<hardware_interface::EffortJointInterface>
	{
		ros::NodeHandle node_;

		hardware_interface::EffortJointInterface *robot_;
		std::vector<hardware_interface::JointHandle> joints_;

		ros::Subscriber sub_imu_sensor_;
		ros::Subscriber sub_command_;

		Eigen::MatrixXd Kp;
		Eigen::MatrixXd Kd;

		double Pitch;
		double PitchDot;
		double LeftWheelPosition;
		double LeftWheelVelocity;
		double RightWheelPosition;
		double RightWheelVelocity;

		void imuCB(const sensor_msgs::Imu::ConstPtr &imuData);

		public:
		BobbleBalanceController(void);
		~BobbleBalanceController(void);
		
		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time,const ros::Duration& duration);
	};
}
#endif
