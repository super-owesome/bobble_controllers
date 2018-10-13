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
#include <bobble_controllers/ControlCommands.h>
#include <executive/BobbleBotStatus.h>
#include <tf/transform_datatypes.h>

#include "bobble_controllers/MadgwickAHRS.h"

namespace bobble_controllers
{
	class BobbleBalanceController: public controller_interface::
		Controller<hardware_interface::EffortJointInterface>
	{
		ros::NodeHandle node_;
		hardware_interface::EffortJointInterface *robot_;
		std::vector<hardware_interface::JointHandle> joints_;
		ros::Publisher pub_bobble_status;
		ros::Subscriber sub_imu_sensor_;
		ros::Subscriber sub_command_;

		enum ControlModes{
			IDLE,
			BALANCE,
			DRIVE,
            SPIN_MOTORS
		};

		/// Config params
		double MotorEffortMax;
		// Gains
		double PitchGain;
		double PitchDotGain;
		double YawGain;
		double YawDotGain;
		double WheelGain;
		double WheelDotGain;
		double WheelIntegralGain;
		double WheelIntegralSaturation;
		// Filters
		double PendulumStateAlpha;
		double WheelStateAlpha;
		double EffortPendulumAlpha;
		double EffortWheelAlpha;
		/// State Vars
		int ActiveControlMode;
        double Roll;
		double Pitch;
		double Yaw;
		double RollDot;
		double PitchDot;
		double YawDot;
		double LeftWheelPosition;
		double LeftWheelPositionBias;
		double LeftWheelVelocity;
		double LeftMotorEffortCmd;
		double RightWheelPosition;
		double RightWheelPositionBias;
		double RightWheelVelocity;
		double RightMotorEffortCmd;
		double LeftWheelErrorAccumulated;
		double RightWheelErrorAccumulated;
        double DesiredLeftWheelPosition;
        double DesiredRightWheelPosition;

        // Offsets
		double RollOffset;
		double PitchOffset;
		double YawOffset;
		double WheelVelocityAdjustment;

		// Sim specific parameters
		double MotorEffortToTorqueSimFactor;

        // Control commands
		double DesiredPitch;
		double DesiredYaw;

		// Previous Commands
		double _EffortRightWheelPrevious;
		double _EffortLeftWheelPrevious;
		double _EffortPendulumPrevious;
		double _PitchPrevious;
		double _RollPrevious;
		double _PitchDotPrevious;
		double _RollDotPrevious;
		double _LeftWheelVelocityPrev;
		double _RightWheelVelocityPrev;

		// Safety
		bool _isSafe;
		double MaximumPitch;

		void imuCB(const sensor_msgs::Imu::ConstPtr &imuData);
		void commandCB(const bobble_controllers::ControlCommands::ConstPtr &cmd);
		void write_controller_status_msg();

		void unpackParameter(std::string parameterName, double &referenceToParameter, double defaultValue);

		public:
		BobbleBalanceController(void);
		~BobbleBalanceController(void);
		
		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time,const ros::Duration& duration);
	};
}
#endif
