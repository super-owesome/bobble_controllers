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

        unpackFlag("InSim", InSim, true);
        unpackParameter("StartingTiltSafetyLimitDegrees", StartingTiltSafetyLimitDegrees, 4.0);
        unpackParameter("MaxTiltSafetyLimitDegrees", MaxTiltSafetyLimitDegrees, 20.0);
        unpackParameter("MotorEffortMax", MotorEffortMax, 0.4);
        unpackParameter("MotorEffortToTorqueSimFactor", MotorEffortToTorqueSimFactor, 0.832);
        unpackParameter("WheelVelocityAdjustment", WheelVelocityAdjustment, 0.0);
        unpackParameter("MeasuredTiltFilterGain", MeasuredTiltFilterGain, 0.0);
        unpackParameter("MeasuredTiltDotFilterGain", MeasuredTiltDotFilterGain, 0.0);
        unpackParameter("MeasuredHeadingFilterGain", MeasuredHeadingFilterGain, 0.0);
        unpackParameter("MeasuredTurnRateFilterGain", MeasuredTurnRateFilterGain, 0.0);
        unpackParameter("LeftWheelVelocityFilterGain", LeftWheelVelocityFilterGain, 0.0);
        unpackParameter("RightWheelVelocityFilterGain", RightWheelVelocityFilterGain, 0.0);
        unpackParameter("VelocityCmdScale", VelocityCmdScale, 1.0);
        unpackParameter("TurnCmdScale", TurnCmdScale, 1.0);
        unpackParameter("VelocityControlKp", VelocityControlKp, 1.0);
        unpackParameter("VelocityControlKi", VelocityControlKi, 0.01);
        unpackParameter("VelocityControlAlphaFilter", VelocityControlAlphaFilter, 0.05);
        unpackParameter("VelocityControlMaxIntegralOutput", VelocityControlMaxIntegralOutput, 0.6);
        unpackParameter("VelocityControlOutputLimitDegrees", VelocityControlOutputLimitDegrees, 5.0);
        unpackParameter("TiltControlKp", TiltControlKp, 1.0);
        unpackParameter("TiltControlKd", TiltControlKd, 0.01);
        unpackParameter("TiltControlAlphaFilter", TiltControlAlphaFilter, 0.05);
        unpackParameter("TurningControlKp", TurningControlKp, 1.0);
        unpackParameter("TurningControlKi", TurningControlKi, 0.01);
        unpackParameter("TurningControlKd", TurningControlKd, 0.01);
        unpackParameter("TiltOffset", TiltOffset, 0.0);

        // Setup Measured State Filters
        MeasuredTiltFilter.setGain(MeasuredTiltFilterGain);
        MeasuredTiltDotFilter.setGain(MeasuredTiltDotFilterGain);
        MeasuredHeadingFilter.setGain(MeasuredHeadingFilterGain);
        MeasuredTurnRateFilter.setGain(MeasuredTurnRateFilterGain);
        LeftWheelVelocityFilter.setGain(LeftWheelVelocityFilterGain);
        RightWheelVelocityFilter.setGain(RightWheelVelocityFilterGain);

        // Setup PID Controllers
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

        TurningControlPID.setPID(TurningControlKp, TurningControlKi, TurningControlKd);
        TurningControlPID.setOutputFilter(0.05);
        TurningControlPID.setMaxIOutput(1.0);
        TurningControlPID.setOutputLimits(-MotorEffortMax / 5.0, MotorEffortMax / 5.0);
        TurningControlPID.setDirection(false);
        TurningControlPID.setSetpointRange(45.0 * (M_PI / 180.0));

        // Setup publishers and subscribers
        pub_bobble_status = n.advertise<executive::BobbleBotStatus>("bb_controller_status", 1);
        sub_imu_sensor_ = node_.subscribe("/imu_bosch/data_raw", 1, &BobbleBalanceController::imuCB, this);
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
        TiltEffort = 0.0;
        HeadingEffort = 0.0;
        LeftMotorEffortCmd = 0.0;
        RightMotorEffortCmd = 0.0;
        ForwardVelocity = 0.0;
        Tilt = 0.0;
        TiltDot = 0.0;
        Heading = 0.0;
        TurnRate = 0.0;
        LeftWheelVelocity = 0.0;
        RightWheelVelocity = 0.0;
        // Reset Madgwick Q on start is a sim only thing. The sim
        // resets the orientation on transition from Idle to Balance
        if (InSim) {
            q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
        }
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
        // Set Angular Velocities
        MeasuredTiltDot = imuData->angular_velocity.y;
        MeasuredTurnRate = imuData->angular_velocity.z;
        // Use Madgwick orientation filter.
        MadgwickAHRSupdateIMU(imuData->angular_velocity.x, imuData->angular_velocity.y, imuData->angular_velocity.z,
							  imuData->linear_acceleration.x, imuData->linear_acceleration.y,
							  imuData->linear_acceleration.z);
        // Construct a DCM matrix from the quaternion
        tf::Quaternion q(q0, q1, q2, q3);
		tf::Matrix3x3 m(q);
		// Set Angles
		if(InSim)
		{
			m.getRPY(MeasuredHeading, MeasuredTilt, MeasuredRoll);
			MeasuredTilt*=-1.0;
		}
		else
		{
			m.getRPY(MeasuredHeading, MeasuredRoll, MeasuredTilt);
			MeasuredTilt -= TiltOffset;
		}

    }

    void BobbleBalanceController::commandCB(const bobble_controllers::ControlCommands::ConstPtr &cmd) {
        StartupCmd = cmd->StartupCmd;
        IdleCmd = cmd->IdleCmd;
        DesiredVelocity = VelocityCmdScale * cmd->DesiredVelocity;
        DesiredTurnRate = TurnCmdScale * cmd->DesiredTurnRate;
    }

    void BobbleBalanceController::update(const ros::Time &time, const ros::Duration &duration) {
        Tilt = MeasuredTiltFilter.filter(MeasuredTilt);
        TiltDot = MeasuredTiltDotFilter.filter(MeasuredTiltDot);
        Heading = MeasuredHeadingFilter.filter(MeasuredHeading);
        TurnRate = MeasuredTurnRateFilter.filter(TurnRate);

        // Get odometry information.
        MeasuredLeftMotorPosition = joints_[1].getPosition();
        MeasuredRightMotorPosition = joints_[0].getPosition();
        MeasuredLeftMotorVelocity = joints_[1].getVelocity();
        MeasuredRightMotorVelocity = joints_[0].getVelocity();

        // Filter wheel velocities and apply a wheel velocity adjustment in order to remove
        // a perceived wheel motion due to pendulum rotation
        LeftWheelVelocity = LeftWheelVelocityFilter.filter(MeasuredLeftMotorVelocity) * WheelVelocityAdjustment + TiltDot;
        RightWheelVelocity = RightWheelVelocityFilter.filter(MeasuredRightMotorVelocity) * WheelVelocityAdjustment + TiltDot;

        // Compute estimate forward velocity and turn rate.
        ForwardVelocity = (RightWheelVelocity + LeftWheelVelocity)/2;

        // TODO Apply filters to desired values. Filter stick inputs?

        /////////////////////////////////////////////////////////////////////////////////////////
        /// Perform the desired control depending on BobbleBot controller state
        /////////////////////////////////////////////////////////////////////////////////////////
        if (ActiveControlMode == ControlModes::IDLE) {
            // Sim only q reset
            if (InSim) {
                q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
            }
            TiltEffort = 0.0;
            HeadingEffort = 0.0;
            if (StartupCmd) {
                ActiveControlMode = ControlModes::STARTUP;
            }
        } else if (ActiveControlMode == ControlModes::DIAGNOSTIC)
		{
			if (abs(Tilt) >= StartingTiltSafetyLimitDegrees * (M_PI / 180.0)) {
				TiltEffort = DesiredVelocity;
                HeadingEffort = 0.0;
			} else {
				TiltEffort = 0.0;
                HeadingEffort = 0.0;
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
            DesiredTilt = VelocityControlPID.getOutput(DesiredVelocity, ForwardVelocity);
            DesiredTilt = DesiredTilt*-1.0;
            TiltEffort = TiltControlPID.getOutput(DesiredTilt, Tilt);
            HeadingEffort = TurningControlPID.getOutput(TurnRate, DesiredTurnRate);
            if (IdleCmd) {
                ActiveControlMode = ControlModes::IDLE;
            }
        } else if (ActiveControlMode == ControlModes::DRIVE) {
            DesiredTilt = VelocityControlPID.getOutput(DesiredVelocity, ForwardVelocity);
            TiltEffort = TiltControlPID.getOutput(DesiredTilt, Tilt);
            HeadingEffort = TurningControlPID.getOutput(TurnRate, DesiredTurnRate);
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
        if(InSim) {
            joints_[0].setCommand(RightMotorEffortCmd * MotorEffortToTorqueSimFactor);
            joints_[1].setCommand(LeftMotorEffortCmd * MotorEffortToTorqueSimFactor);
        }else {
            joints_[0].setCommand(RightMotorEffortCmd);
            joints_[1].setCommand(LeftMotorEffortCmd);
        }

        /////////////////////////////////////////////////////////////////////////////////////////
        /// Report our status. TODO Use RT safe approach for this.
        /////////////////////////////////////////////////////////////////////////////////////////
        write_controller_status_msg();
    }

    void BobbleBalanceController::write_controller_status_msg() {
        executive::BobbleBotStatus sim_status_msg;
        sim_status_msg.ControlMode = ActiveControlMode;
        sim_status_msg.DeltaT = 0.0;
        sim_status_msg.Tilt = Tilt * (180.0 / M_PI);
        sim_status_msg.TiltRate = TiltDot * (180.0 / M_PI);
        sim_status_msg.Heading = Heading * (180.0 / M_PI);
        sim_status_msg.TurnRate = TurnRate * (180.0 / M_PI);
        sim_status_msg.ForwardVelocity = ForwardVelocity;
        sim_status_msg.DesiredVelocity = DesiredVelocity;
        sim_status_msg.DesiredTilt = DesiredTilt * (180.0 / M_PI);
        sim_status_msg.DesiredTurnRate = DesiredTurnRate * (180.0 / M_PI);
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

    void BobbleBalanceController::unpackFlag(std::string parameterName, bool &referenceToFlag,
                                             bool defaultValue) {
        if (!node_.getParam(parameterName, referenceToFlag)) {
            referenceToFlag = defaultValue;
            ROS_ERROR("%s not set for (namespace: %s). Setting to false.",
                      parameterName.c_str(),
                      node_.getNamespace().c_str());
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
