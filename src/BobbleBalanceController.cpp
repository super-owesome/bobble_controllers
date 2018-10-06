/******************************************************************************
 * Document this if it actually works
 * MMM
*******************************************************************************/

#include <sys/mman.h>

#include <bobble_controllers/BobbleBalanceController.h>
#include <pluginlib/class_list_macros.h>

namespace bobble_controllers {
    BobbleBalanceController::BobbleBalanceController(void)
            :
            VelocityControlPID(0.0, 0.0, 0.0),
            TiltControlPID(0.0, 0.0, 0.0),
            HoldHeadingControlPID(0.0, 0.0, 0.0),
            TurningControlPID(0.0, 0.0, 0.0) {
    }

    BobbleBalanceController::~BobbleBalanceController(void) {
        sub_command_.shutdown();
    }

    bool BobbleBalanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) {
        node_ = n;
        robot_ = robot;

        XmlRpc::XmlRpcValue joint_names;
        if (!node_.getParam("joints", joint_names)) {
            ROS_ERROR("No 'joints' in controller. (namespace: %s)",
                      node_.getNamespace().c_str());
            return false;
        }

        if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("'joints' is not a struct. (namespace: %s)",
                      node_.getNamespace().c_str());
            return false;
        }

        for (int i = 0; i < joint_names.size(); i++) {
            XmlRpc::XmlRpcValue &name_value = joint_names[i];
            if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("joints are not strings. (namespace: %s)",
                          node_.getNamespace().c_str());
                return false;
            }

            hardware_interface::JointHandle j = robot->getHandle((std::string) name_value);
            joints_.push_back(j);
        }

        unpackParameter("StartingTiltSafetyLimitDegrees", StartingTiltSafetyLimitDegrees, 4.0);
        unpackParameter("MaxTiltSafetyLimitDegrees", MaxTiltSafetyLimitDegrees, 20.0);
        unpackParameter("MotorEffortMax", MotorEffortMax, 0.4);
        unpackParameter("MotorEffortToTorqueSimFactor", MotorEffortToTorqueSimFactor, 0.832);
        unpackParameter("MeasuredTiltFilterGain", MeasuredTiltFilterGain, 0.05);
        unpackParameter("MeasuredTiltDotFilterGain", MeasuredTiltDotFilterGain, 0.05);
        unpackParameter("LeftWheelVelocityFilterGain", LeftWheelVelocityFilterGain, 0.05);
        unpackParameter("VelocityControlKp", VelocityControlKp, 1.0);
        unpackParameter("VelocityControlKi", VelocityControlKi, 0.01);
        unpackParameter("VelocityControlAlphaFilter", VelocityControlAlphaFilter, 0.05);
        unpackParameter("VelocityControlMaxIntegralOutput", VelocityControlMaxIntegralOutput, 0.6);
        unpackParameter("VelocityControlOutputLimitDegrees", VelocityControlOutputLimitDegrees, 5.0);
        unpackParameter("TiltControlKp", TiltControlKp, 1.0);
        unpackParameter("TiltControlKd", TiltControlKd, 0.01);
        unpackParameter("TiltControlAlphaFilter", TiltControlAlphaFilter, 0.05);
        unpackParameter("HoldHeadingControlKp", HoldHeadingControlKp, 1.0);
        unpackParameter("HoldHeadingControlKd", HoldHeadingControlKd, 0.01);
        unpackParameter("TurningControlKp", TurningControlKp, 1.0);
        unpackParameter("TurningControlKi", TurningControlKi, 0.01);
        unpackParameter("TurningControlKd", TurningControlKd, 0.01);

        // Setup Measured State Filters
        MeasuredTiltFilter.setGain(MeasuredTiltFilterGain);
        MeasuredTiltDotFilter.setGain(MeasuredTiltDotFilterGain);
        LeftWheelVelocityFilter.setGain(LeftWheelVelocityFilterGain);

        // Setup PID Controllers
        // TODO: Expose the important tunable constants to param file
        VelocityControlPID.setPID(VelocityControlKp, VelocityControlKi, 0.0);
        VelocityControlPID.setOutputFilter(VelocityControlAlphaFilter);
        VelocityControlPID.setMaxIOutput(VelocityControlMaxIntegralOutput);
        VelocityControlPID.setOutputLimits(VelocityControlOutputLimitDegrees * (M_PI / 180.0),
                                           VelocityControlOutputLimitDegrees * (M_PI / 180.0));
        VelocityControlPID.setDirection(true);

        TiltControlPID.setPID(TiltControlKp, 0.0, TiltControlKd);
        TiltControlPID.setExternalDerivativeError(&TiltDot);
        TiltControlPID.setOutputFilter(TiltControlAlphaFilter);
        TiltControlPID.setOutputLimits(-MotorEffortMax, MotorEffortMax);
        TiltControlPID.setDirection(true);
        TiltControlPID.setSetpointRange(20.0 * (M_PI / 180.0));

        HoldHeadingControlPID.setPID(HoldHeadingControlKp, 0.0, HoldHeadingControlKd);
        HoldHeadingControlPID.setOutputFilter(0.05);
        HoldHeadingControlPID.setOutputLimits(-MotorEffortMax / 2.0, MotorEffortMax / 2.0);
        HoldHeadingControlPID.setDirection(false);
        HoldHeadingControlPID.setSetpointRange(90.0 * (M_PI / 180.0));

        TurningControlPID.setPID(TurningControlKp, TurningControlKi, TurningControlKd);
        TurningControlPID.setOutputFilter(0.05);
        TurningControlPID.setMaxIOutput(20.0 * (M_PI / 180.0));
        TurningControlPID.setOutputLimits(-MotorEffortMax / 2.0, MotorEffortMax / 2.0);
        TurningControlPID.setDirection(false);
        TurningControlPID.setSetpointRange(45.0 * (M_PI / 180.0));

        // Setup publishers and subscribers
        pub_bobble_status = n.advertise<executive::BobbleBotStatus>("bb_controller_status", 1);
        sub_imu_sensor_ = node_.subscribe("/imu_bosch/data", 1, &BobbleBalanceController::imuCB, this);
        sub_command_ = node_.subscribe("/bobble/bobble_balance_controller/bb_cmd", 1,
                                       &BobbleBalanceController::commandCB, this);

        return true;
    }


    void BobbleBalanceController::starting(const ros::Time &time) {
        ActiveControlMode = ControlModes::IDLE;
        StartupCmd = false;
        DesiredVelocity = 0.0;
        DesiredTilt = 0.0;
        DesiredTurnRate = 0.0;
        DesiredHeading = 0.0;
        TiltEffort = 0.0;
        HeadingEffort = 0.0;
        LeftMotorEffortCmd = 0.0;
        RightMotorEffortCmd = 0.0;
        ForwardVelocity = 0.0;
        Tilt = 0.0;
        TiltDot = 0.0;
        Heading = 0.0;
        TurnRate = 0.0;
        // Setup the Real-Time thread
        struct sched_param param;
        // set the priority high, but not so high it overrides the comm
        param.sched_priority = 95;
        if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            ROS_WARN("Failed to set real-time scheduler.");
            return;
        }
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
            ROS_WARN("Failed to lock memory.");
        }
    }

    void BobbleBalanceController::imuCB(const sensor_msgs::Imu::ConstPtr &imuData) {
        tf::Quaternion q(imuData->orientation.x, imuData->orientation.y, imuData->orientation.z,
                         imuData->orientation.w);
        tf::Matrix3x3 m(q);
        MeasuredTiltDot = 1.0 * imuData->angular_velocity.y;
        MeasuredTurnRate = imuData->angular_velocity.z;
        m.getRPY(MeasuredRoll, MeasuredTilt, MeasuredHeading);
    }

    void BobbleBalanceController::commandCB(const bobble_controllers::ControlCommands::ConstPtr &cmd) {
        StartupCmd = cmd->StartupCmd;
        IdleCmd = cmd->IdleCmd;
        DesiredVelocity = 2.0 * cmd->DesiredVelocity;
        DesiredTurnRate = cmd->DesiredTurnRate;
        DesiredHeading = cmd->DesiredHeading;
    }

    void BobbleBalanceController::update(const ros::Time &time, const ros::Duration &duration) {
        // Get odometry information. Not using these quite yet.
        MeasuredRightMotorPosition = joints_[0].getPosition();
        MeasuredRightMotorVelocity = joints_[0].getVelocity();
        MeasuredLeftMotorPosition = joints_[1].getPosition();
        MeasuredLeftMotorVelocity = joints_[1].getVelocity();
        //ForwardVelocity = LeftWheelVelocityFilter.filter(MeasuredLeftMotorVelocity);
        ForwardVelocity = 0.05*(MeasuredRightMotorVelocity + MeasuredLeftMotorVelocity)/2; // R * (vR + vL)/2
        // TODO Compute velocity
        // Velocity = ;
        // TODO Apply filters to measured state values
        Tilt = MeasuredTiltFilter.filter(MeasuredTilt);
        TiltDot = MeasuredTiltDotFilter.filter(MeasuredTiltDot);
        //Heading = MeasuredHeadingFilter.filter(MeasuredHeading);
        //TurnRate = MeasuredTurnRate.filter(MeasuredTurnRate);
        Tilt = MeasuredTilt;
        TiltDot = MeasuredTiltDot;
        Heading = MeasuredHeading;
        TurnRate = MeasuredTurnRate;
        // TODO Apply filters to desired values. Filter stick inputs

        /////////////////////////////////////////////////////////////////////////////////////////
        /// Perform the desired control depending on BobbleBot controller state
        /////////////////////////////////////////////////////////////////////////////////////////
        if (ActiveControlMode == ControlModes::IDLE) {
            TiltEffort = 0.0;
            HeadingEffort = 0.0;
            if (StartupCmd) {
                ActiveControlMode = ControlModes::STARTUP;
            }
        } else if (ActiveControlMode == ControlModes::STARTUP) {
            // Wait until we're safe to proceed to balance mode
            if (abs(Tilt) >= StartingTiltSafetyLimitDegrees * (M_PI / 180.0)) {
                TiltEffort = 0.0;
                HeadingEffort = 0.0;
            } else {
                ActiveControlMode = ControlModes::BALANCE;
            }
        } else if (ActiveControlMode == ControlModes::BALANCE) {
            DesiredHeading = Heading;
            DesiredTilt = VelocityControlPID.getOutput(DesiredVelocity, ForwardVelocity);
            DesiredTilt = DesiredTilt*-1.0;
            TiltEffort = TiltControlPID.getOutput(DesiredTilt, Tilt);
            perform_heading_control();
            if (IdleCmd) {
                ActiveControlMode = ControlModes::IDLE;
            }
            if (abs(DesiredVelocity) >= 0.1 || abs(DesiredTurnRate) >= 0.1) {
                ActiveControlMode = ControlModes::DRIVE;
            }
        } else if (ActiveControlMode == ControlModes::DRIVE) {
            DesiredTilt = VelocityControlPID.getOutput(DesiredVelocity, ForwardVelocity);
            TiltEffort = TiltControlPID.getOutput(DesiredTilt, Tilt);
            perform_heading_control();
            if (IdleCmd) {
                ActiveControlMode = ControlModes::IDLE;
            }
        } else {
            TiltEffort = 0.0;
            HeadingEffort = 0.0;
        }

        /////////////////////////////////////////////////////////////////////////////////////////
        /// Filter the motor effort commands
        /////////////////////////////////////////////////////////////////////////////////////////
        RightMotorEffortCmd = TiltEffort + HeadingEffort;
        LeftMotorEffortCmd = TiltEffort - HeadingEffort;
        // TODO
        //RightMotorEffortCmd = (double) RightMotorLowPassEffortFilter.filter(TiltEffort + HeadingEffort);
        //LeftMotorEffortCmd = (double) LeftMotorLowPassEffortFilter.filter(TiltEffort - HeadingEffort);

        /////////////////////////////////////////////////////////////////////////////////////////
        /// Apply safety checks
        /////////////////////////////////////////////////////////////////////////////////////////
        // No effort when tilt angle is way out of whack.
        // You're going down. Don't fight it... just accept it.
        if (abs(Tilt) >= MaxTiltSafetyLimitDegrees * (M_PI / 180.0)) {
            RightMotorEffortCmd = 0.0;
            LeftMotorEffortCmd = 0.0;
        }
        // Apply motor effort limits
        RightMotorEffortCmd = limitEffort(RightMotorEffortCmd);
        LeftMotorEffortCmd = limitEffort(LeftMotorEffortCmd);

        /////////////////////////////////////////////////////////////////////////////////////////
        /// Send our motor commands
        /////////////////////////////////////////////////////////////////////////////////////////
        // Send the limited effort commands
        joints_[0].setCommand(RightMotorEffortCmd * MotorEffortToTorqueSimFactor);
        joints_[1].setCommand(LeftMotorEffortCmd * MotorEffortToTorqueSimFactor);

        /////////////////////////////////////////////////////////////////////////////////////////
        /// Report our status
        /////////////////////////////////////////////////////////////////////////////////////////
        write_controller_status_msg();
    }

    void BobbleBalanceController::perform_heading_control() {
        if (DesiredTurnRate == 0) {
            HeadingEffort = HoldHeadingControlPID.getOutput(Heading, DesiredHeading);
        } else {
            HeadingEffort = TurningControlPID.getOutput(TurnRate, DesiredTurnRate);
        }
    }

    void BobbleBalanceController::write_controller_status_msg() {
        executive::BobbleBotStatus sim_status_msg;
        sim_status_msg.ControlMode = ActiveControlMode;
        sim_status_msg.DeltaT = 0.0;
        sim_status_msg.Tilt = Tilt * (180.0 / M_PI);
        sim_status_msg.TiltRate = TiltDot * (180.0 / M_PI);
        sim_status_msg.Heading = Heading * (180.0 / M_PI);
        sim_status_msg.TurnRate = TurnRate * (180.0 / M_PI);
        sim_status_msg.ForwardVelocity = ForwardVelocity * (180.0 / M_PI);
        sim_status_msg.DesiredVelocity = DesiredVelocity;
        sim_status_msg.DesiredTilt = DesiredTilt * (180.0 / M_PI);
        sim_status_msg.DesiredTurnRate = DesiredTurnRate * (180.0 / M_PI);
        sim_status_msg.DesiredHeading = DesiredHeading * (180.0 / M_PI);
        sim_status_msg.LeftMotorPosition = MeasuredLeftMotorPosition * (180.0 / M_PI);
        sim_status_msg.LeftMotorVelocity = MeasuredLeftMotorVelocity * (180.0 / M_PI);
        sim_status_msg.RightMotorPosition = MeasuredRightMotorPosition * (180.0 / M_PI);
        sim_status_msg.RightMotorVelocity = MeasuredRightMotorVelocity * (180.0 / M_PI);
        sim_status_msg.TiltEffort = TiltEffort;
        sim_status_msg.HeadingEffort = HeadingEffort;
        sim_status_msg.LeftMotorEffortCmd = LeftMotorEffortCmd;
        sim_status_msg.RightMotorEffortCmd = RightMotorEffortCmd;
        pub_bobble_status.publish(sim_status_msg);
    }

    void BobbleBalanceController::unpackParameter(std::string parameterName, double &referenceToParameter,
                                                  double defaultValue) {
        if (!node_.getParam(parameterName, referenceToParameter)) {
            referenceToParameter = defaultValue;
            ROS_ERROR("%s not set for (namespace: %s) using %f.",
                      parameterName.c_str(),
                      node_.getNamespace().c_str(),
                      defaultValue);
        }
    }

    double BobbleBalanceController::limitEffort(double effort_cmd) {
        if (effort_cmd < -MotorEffortMax) {
            return -MotorEffortMax;
        } else if (effort_cmd > MotorEffortMax) {
            return MotorEffortMax;
        }
        return effort_cmd;
    }

}

PLUGINLIB_EXPORT_CLASS(bobble_controllers::BobbleBalanceController, controller_interface::ControllerBase
)
