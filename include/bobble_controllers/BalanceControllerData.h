//
// Created by mike on 5/11/19.
//

#ifndef SRC_BALANCE_CONTROLLER_DATA_H
#define SRC_BALANCE_CONTROLLER_DATA_H

#include <bobble_controllers/ControlCommands.h>
#include <bobble_controllers/BobbleBotStatus.h>
#include <bobble_controllers/PidControl.h>
#include "bobble_controllers/MadgwickAHRS.h"
#include <bobble_controllers/Filter.h>
#include <bobble_controllers/LowPassFilter.h>

namespace bobble_controllers {

    /// Controller real-time data.
    /// HW interface data.
    class BalanceControllerRtData {
    public:
        BalanceControllerRtData() :
	 AccelX(0.0), AccelY(0.0), AccelZ(0.0),
     RollDot(0.0), TiltDot(0.0), TurnDot(0.0),
	 LeftMotorPosition(0.0), LeftMotorVelocity(0.0), LeftMotorEffort(0.0),
	 RightMotorPosition(0.0), RightMotorVelocity(0.0), RightMotorEffort(0.0){};
        double AccelX;
        double AccelY;
        double AccelZ;
        double RollDot;
        double TiltDot;
        double TurnDot;
        double LeftMotorPosition;
        double LeftMotorVelocity;
        double LeftMotorEffort;
        double RightMotorPosition;
        double RightMotorVelocity;
        double RightMotorEffort;
    };

    /// Controller configuration parameters.
    /// These are set at init via yml
    class BalanceControllerConfig {
    public:
        double ControlLoopFrequency;
        double StartingTiltSafetyLimitDegrees;
        double MaxTiltSafetyLimitDegrees;
        double ControllerEffortMax;
        double MotorEffortToTorqueSimFactor;
        double WheelVelocityAdjustment;
        double MadgwickFilterGain;
        double MeasuredTiltFilterFrequency;
        double MeasuredTiltDotFilterFrequency;
        double MeasuredHeadingFilterFrequency;
        double MeasuredTurnRateFilterFrequency;
        double LeftWheelVelocityFilterFrequency;
        double RightWheelVelocityFilterFrequency;
        double DesiredForwardVelocityFilterFrequency;
        double DesiredTurnRateFilterFrequency;
        double MaxVelocityCmd;
        double MaxTurnRateCmd;
        double WheelRadiusMeters;
        double VelocityCmdScale;
        double TurnCmdScale;
        double VelocityControlKp;
        double VelocityControlKd;
        double VelocityControlKi;
        double VelocityControlAlphaFilter;
        double VelocityControlMaxIntegralOutput;
        double VelocityControlOutputLimitDegrees;
        double TiltControlKp;
        double TiltControlKd;
        double TiltControlAlphaFilter;
        double TiltOffset;
        double TiltDotOffset;
        double RollDotOffset;
        double YawDotOffset;
        double TurningControlKp;
        double TurningControlKi;
        double TurningControlKd;
        double TurningOutputFilter;
    };

    /// Controller commands.
    /// These are sent in via external command.
    class BalanceControllerCommands {
    public:
        bool StartupCmd;
        bool IdleCmd;
        bool DiagnosticCmd;
        double DesiredVelocityRaw;
        double DesiredTurnRateRaw;
        double DesiredVelocity;
        double DesiredTurnRate;
    };

    /// Controller outputs.
    class BalanceControllerOutputs {
    public:
        double TiltEffort;
        double HeadingEffort;
        double LeftMotorEffortCmd;
        double RightMotorEffortCmd;
    };

    /// Controller state parameters.
    /// These are updated continually at run-time
    class BalanceControllerState {
    public:
        int    ActiveControlMode;
        double MeasuredTilt;
        double MeasuredHeading;
        double MeasuredRoll;
        double MeasuredRollDot;
        double MeasuredTiltDot;
        double MeasuredTurnRate;
        double FilteredRollDot;
        double FilteredTiltDot;
        double FilteredTurnRate;
        double MeasuredRightMotorPosition;
        double MeasuredRightMotorVelocity;
        double MeasuredLeftMotorPosition;
        double MeasuredLeftMotorVelocity;
        double ForwardVelocity;
        double DesiredTilt;
        double Tilt;
        double TiltDot;
        double Heading;
        double TurnRate;
        double LeftWheelVelocity;
        double RightWheelVelocity;
        BalanceControllerCommands Cmds;
    };

    /// Controller modes
    enum ControlModes {
        IDLE = 0,
        STARTUP = 1,
        BALANCE = 2,
        DIAGNOSTIC = 3
    };

    /// Controller filters
    class BalanceControllerFilters {
    public:
        LowPassFilter MeasuredTiltFilter;
        LowPassFilter MeasuredTiltDotFilter;
        LowPassFilter MeasuredHeadingFilter;
        LowPassFilter MeasuredTurnRateFilter;
        LowPassFilter LeftWheelVelocityFilter;
        LowPassFilter RightWheelVelocityFilter;
        LowPassFilter DesiredForwardVelocityFilter;
        LowPassFilter DesiredTurnRateFilter;
    };


    /// PID Controllers
    class BalancePIDControllers {
    public:
        BalancePIDControllers() :
          VelocityControlPID(0.0, 0.0, 0.0),
          TiltControlPID(0.0, 0.0, 0.0),
          TurningControlPID(0.0, 0.0, 0.0)
        {}
        PidControl VelocityControlPID;
        PidControl TiltControlPID;
        PidControl TurningControlPID;
    };
}

#endif //SRC_BALANCE_CONTROLLER_DATA_H
