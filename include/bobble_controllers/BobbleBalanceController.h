/******************************************************************************
 * Document this if it actually works...
 * Mike
*******************************************************************************/

#ifndef BOBBLE_CONTROLLERS_BOBBLE_BALANCE_CONTROLLER_H
#define BOBBLE_CONTROLLERS_BOBBLE_BALANCE_CONTROLLER_H

#include <cstddef>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <bobble_controllers/ControlCommands.h>
#include <bobble_controllers/PidControl.h>
#include <generic_filter/Filter.h>
#include <executive/BobbleBotStatus.h>
#include <tf/transform_datatypes.h>

namespace bobble_controllers {

    class BobbleBalanceController : public controller_interface::
    Controller<hardware_interface::EffortJointInterface> {
        ros::NodeHandle node_;
        hardware_interface::EffortJointInterface *robot_;
        std::vector <hardware_interface::JointHandle> joints_;
        ros::Publisher pub_bobble_status;
        ros::Subscriber sub_imu_sensor_;
        ros::Subscriber sub_command_;

        enum ControlModes {
            IDLE,
            STARTUP,
            BALANCE,
            DRIVE,
            DIAGNOSTIC
        };

        /// Controller config
        double StartingTiltSafetyLimitDegrees;
        double MaxTiltSafetyLimitDegrees;
        double MotorEffortMax;
        double MotorEffortToTorqueSimFactor;
        double MotorEffortFilterGain;
        double VelocityControlKp;
        double VelocityControlKi;
        double VelocityControlAlphaFilter;
        double VelocityControlMaxIntegralOutput;
        double VelocityControlOutputLimitDegrees;
        double TiltControlKp;
        double TiltControlKd;
        double TiltControlAlphaFilter;
        double HoldHeadingControlKp;
        double HoldHeadingControlKd;
        double TurningControlKp;
        double TurningControlKi;
        double TurningControlKd;

        /// Measured state
        double MeasuredTilt;
        double MeasuredHeading;
        double MeasuredRoll;
        double MeasuredTiltDot;
        double MeasuredTurnRate;
        double MeasuredRightMotorPosition;
        double MeasuredRightMotorVelocity;
        double MeasuredLeftMotorPosition;
        double MeasuredLeftMotorVelocity;
        double ForwardVelocity;

        /// Control commands (desired states)
        bool StartupCmd;
        bool IdleCmd;
        double DesiredVelocity;
        double DesiredTurnRate;
        double DesiredHeading;

        /// Filters
        LPFilter MeasuredTiltFilter;
        LPFilter MeasuredTiltDotFilter;
        LPFilter LeftWheelVelocityFilter;
        //LPFilter RightMotorEffortLPFilter;

        /// Filter Gains
        double MeasuredTiltFilterGain;
        double MeasuredTiltDotFilterGain;
        double LeftWheelVelocityFilterGain;

        /// PID Controllers
        PidControl VelocityControlPID;
        PidControl TiltControlPID;
        PidControl HoldHeadingControlPID;
        PidControl TurningControlPID;

        /// Controller state
        int ActiveControlMode;
        double DesiredTilt;
        double Tilt;
        double TiltDot;
        double Heading;
        double TurnRate;
        double LastTiltError;
        double LastHeadingError;
        double LastTurningError;

        /// Controller outputs
        double TiltEffort;
        double HeadingEffort;
        double LeftMotorEffortCmd;
        double RightMotorEffortCmd;

        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

        void starting(const ros::Time &time);

        void imuCB(const sensor_msgs::Imu::ConstPtr &imuData);

        void commandCB(const bobble_controllers::ControlCommands::ConstPtr &cmd);

        void update(const ros::Time &time, const ros::Duration &duration);

        void perform_velocity_control();

        void perform_tilt_control();

        void perform_heading_control();

        void write_controller_status_msg();

        void unpackParameter(std::string parameterName, double &referenceToParameter, double defaultValue);

        double limitEffort(double effort_cmd);

    public:
        BobbleBalanceController(void);

        ~BobbleBalanceController(void);

    };
}
#endif
