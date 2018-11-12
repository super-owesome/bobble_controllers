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
#include <thread>
#include <mutex>

#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <controller_interface/controller.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <bobble_controllers/ControlCommands.h>
#include <bobble_controllers/PidControl.h>
#include "bobble_controllers/MadgwickAHRS.h"
#include <generic_filter/Filter.h>
#include <executive/BobbleBotStatus.h>
#include <tf/transform_datatypes.h>

namespace bobble_controllers {

    class BobbleBalanceController : public controller_interface::
    Controller<hardware_interface::EffortJointInterface> {
        ros::NodeHandle node_;
        hardware_interface::RobotHW *robot_;
        std::vector <hardware_interface::JointHandle> joints_;
        hardware_interface::ImuSensorInterface *imu_;
        realtime_tools::RealtimePublisher<executive::BobbleBotStatus>* pub_bobble_status;
        ros::Subscriber sub_imu_sensor_;
        ros::Subscriber sub_command_;
        hardware_interface::ImuSensorHandle imuData;
        enum ControlModes {
            IDLE,
            STARTUP,
            BALANCE,
            DIAGNOSTIC
        };

        typedef struct {
            bool StartupCmd;
            bool IdleCmd;
            bool DiagnosticCmd;
            double DesiredVelocity;
            double DesiredTurnRate;
        } CommandStruct;

        std::mutex control_command_mutex;

	    std::thread* subscriberThread;
        CommandStruct commandStruct;
        CommandStruct commandStructTmp;
	    bool runThread;
        void runSubscriber();
        void subscriberCallBack(const bobble_controllers::ControlCommands::ConstPtr &cmd);

        /// Controller config
        bool InSim; // Temporary hack. Need to re-orient IMU model.
        double StartingTiltSafetyLimitDegrees;
        double MaxTiltSafetyLimitDegrees;
        double MotorEffortMax;
        double MotorEffortToTorqueSimFactor;
        double WheelVelocityAdjustment;
        double MotorEffortFilterGain;
        double MaxVelocityCmd;
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
        int ActiveControlMode;
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

        void write_controller_status_msg();


        void unpackParameter(std::string parameterName, double &referenceToParameter, double defaultValue);
        void unpackParameter(std::string parameterName, std::string &referenceToParameter, std::string defaultValue);

        void unpackFlag(std::string parameterName, bool &referenceToFlag, bool defaultValue);

        double limit(double cmd, double max);

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

        ~BobbleBalanceController(void);

    };
}
#endif
