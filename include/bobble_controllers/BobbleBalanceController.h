/******************************************************************************
 * Document this if it actually works...
 * Mike
*******************************************************************************/

#ifndef BOBBLE_CONTROLLERS_BOBBLE_BALANCE_CONTROLLER_H
#define BOBBLE_CONTROLLERS_BOBBLE_BALANCE_CONTROLLER_H

#include <bobble_controllers/BobbleControllerBase.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <bobble_controllers/ControlCommands.h>
#include <bobble_controllers/PidControl.h>
#include "bobble_controllers/MadgwickAHRS.h"
#include <bobble_controllers/Filter.h>
#include <bobble_controllers/BobbleBotStatus.h>
#include <tf/transform_datatypes.h>

namespace bobble_controllers {

class BobbleBalanceController : public bobble_controllers::BobbleControllerBase<BobbleBotStatus>
    {
        /// Subscriber for simulation
        ros::Subscriber sub_imu_sensor_;

        /// Define control Modes
        enum ControlModes {
            IDLE,
            STARTUP,
            BALANCE,
            DIAGNOSTIC
        };

        /// Define possible commands
        typedef struct {
            bool StartupCmd;
            bool IdleCmd;
            bool DiagnosticCmd;
            double DesiredVelocity;
            double DesiredTurnRate;
        } CommandStruct;


        void subscriberCallBack(const bobble_controllers::ControlCommands::ConstPtr &cmd);
        /**
         * \brief Velocity command callback
         * \param command Velocity command message (twist)
        */
        void cmdVelCallback(const geometry_msgs::Twist& command);

        /// Controller config
        bool InSim; // Temporary hack. Need to re-orient IMU model.
        double StartingTiltSafetyLimitDegrees;
        double MaxTiltSafetyLimitDegrees;
        double MotorEffortMax;
        double MotorEffortToTorqueSimFactor;
        double WheelVelocityAdjustment;
        double WheelRadiusMeters;
        double MaxVelocityCmd;
        double MaxTurnRateCmd;
        double VelocityCmdScale;
        double TurnCmdScale;
        double VelocityControlKp;
        double VelocityControlKi;
        double VelocityControlAlphaFilter;
        double VelocityControlMaxIntegralOutput;
        double VelocityControlOutputLimitDegrees;
        double TiltControlKp;
        double TiltControlKd;
        double TiltControlAlphaFilter;
        double TurningControlKp;
        double TurningControlKi;
        double TurningControlKd;
        double TurningOutputFilter;

        /// Measured state
        double MeasuredTilt;
        double MeasuredHeading;
        double MeasuredRoll;
        double MeasuredRollDot;
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
        bool DiagnosticCmd;
        double DesiredVelocityRaw;
        double DesiredTurnRateRaw;
        double DesiredVelocity;
        double DesiredTurnRate;

        /// Filters
        double MadgwickFilterGain;
        LPFilter MeasuredTiltFilter;
        LPFilter MeasuredTiltDotFilter;
        LPFilter MeasuredHeadingFilter;
        LPFilter MeasuredTurnRateFilter;
        LPFilter LeftWheelVelocityFilter;
        LPFilter RightWheelVelocityFilter;
        LPFilter DesiredForwardVelocityFilter;
        LPFilter DesiredTurnRateFilter;
        //LPFilter RightMotorEffortLPFilter;

        /// Filter Gains
        double MeasuredTiltFilterGain;
        double MeasuredTiltDotFilterGain;
        double MeasuredHeadingFilterGain;
        double MeasuredTurnRateFilterGain;
        double LeftWheelVelocityFilterGain;
        double RightWheelVelocityFilterGain;
        double DesiredForwardVelocityFilterGain;
        double DesiredTurnRateFilterGain;

        /// PID Controllers
        PidControl VelocityControlPID;
        PidControl TiltControlPID;
        PidControl TurningControlPID;

        /// Controller state
        double DesiredTilt;
        double Tilt;
        double TiltDot;
        double TiltOffset;
        double TiltDotOffset;
        double RollDotOffset;
        double YawDotOffset;
        double Heading;
        double TurnRate;
        double LeftWheelVelocity;
        double RightWheelVelocity;

        /// Controller outputs
        double TiltEffort;
        double HeadingEffort;
        double LeftMotorEffortCmd;
        double RightMotorEffortCmd;

        /// Imu Name
        std::string ImuName;

        void starting(const ros::Time &time);

        void imuCB(const sensor_msgs::Imu::ConstPtr &imuData);

        void update(const ros::Time &time, const ros::Duration &duration);

        void publish_status_message();

        void populateImuData();
        void populateCommands();
        void populateOdometry();

        void applyFilters();
        void applySafety();


    protected:
        virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                                 ros::NodeHandle&             root_nh,
                                 ros::NodeHandle&             controller_nh,
                                 ClaimedResources&            claimed_resources) override;
    public:
        BobbleBalanceController(void);

    };
}
#endif
