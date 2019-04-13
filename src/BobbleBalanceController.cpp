/******************************************************************************
 * Document this if it actually works
 * MMM
*******************************************************************************/

#include <sys/mman.h>

#include <bobble_controllers/BobbleBalanceController.h>
#include <pluginlib/class_list_macros.h>

namespace bobble_controllers {

    void BobbleBalanceController::stateCommandCallback(const topic_tools::ShapeShifter::ConstPtr &msg) {
        bobble_controllers::ControlCommands::ConstPtr cmd = msg->instantiate<bobble_controllers::ControlCommands>();
        controlBoolsNoRT["StartupCmd"] = cmd->StartupCmd;
        controlBoolsNoRT["IdleCmd"] = cmd->IdleCmd;
        controlBoolsNoRT["DiagnosticCmd"] = cmd->DiagnosticCmd;
    }


    void BobbleBalanceController::velocityCommandCallback(const topic_tools::ShapeShifter::ConstPtr &msg) {
        geometry_msgs::Twist::ConstPtr cmd = msg->instantiate<geometry_msgs::Twist>();
        controlDoublesNoRT["DesiredVelocity"] = cmd->linear.x;
        controlDoublesNoRT["DesiredTurnRate"] = cmd->angular.z;
    }

    BobbleBalanceController::BobbleBalanceController(void)
            :
            SOControllerBase(),
            VelocityControlPID(0.0, 0.0, 0.0),
            TiltControlPID(0.0, 0.0, 0.0),
            TurningControlPID(0.0, 0.0, 0.0) {
    }


    bool BobbleBalanceController::initRequest(hardware_interface::RobotHW* robot_hw,
                             ros::NodeHandle&             root_nh,
                             ros::NodeHandle&             controller_nh,
                             ClaimedResources&            claimed_resources)
    {
        // check if construction finished cleanly
        if (state_ != CONSTRUCTED){
            ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
            return false;
        }

        // get a pointer to the effort interface
        hardware_interface::EffortJointInterface* effort_hw = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (!effort_hw)
        {
            ROS_ERROR("This controller requires a hardware interface of type hardware_interface::EffortJointInterface.");
            return false;
        }

        node_ = controller_nh;

        unpackFlag("InSim", InSim, true);
        unpackParameter("StartingTiltSafetyLimitDegrees", StartingTiltSafetyLimitDegrees, 4.0);
        unpackParameter("MaxTiltSafetyLimitDegrees", MaxTiltSafetyLimitDegrees, 20.0);
        unpackParameter("MotorEffortMax", MotorEffortMax, 0.4);
        unpackParameter("MotorEffortToTorqueSimFactor", MotorEffortToTorqueSimFactor, 0.832);
        unpackParameter("WheelVelocityAdjustment", WheelVelocityAdjustment, 0.0);
        unpackParameter("MadgwickFilterGain", MadgwickFilterGain, 0.01);
        unpackParameter("MeasuredTiltFilterGain", MeasuredTiltFilterGain, 0.0);
        unpackParameter("MeasuredTiltDotFilterGain", MeasuredTiltDotFilterGain, 0.0);
        unpackParameter("MeasuredHeadingFilterGain", MeasuredHeadingFilterGain, 0.0);
        unpackParameter("MeasuredTurnRateFilterGain", MeasuredTurnRateFilterGain, 0.0);
        unpackParameter("LeftWheelVelocityFilterGain", LeftWheelVelocityFilterGain, 0.0);
        unpackParameter("RightWheelVelocityFilterGain", RightWheelVelocityFilterGain, 0.0);
        unpackParameter("DesiredForwardVelocityFilterGain", DesiredForwardVelocityFilterGain, 0.0);
        unpackParameter("DesiredTurnRateFilterGain", DesiredTurnRateFilterGain, 0.0);
        unpackParameter("WheelRadiusMeters", WheelRadiusMeters, 0.05);
        unpackParameter("VelocityCmdScale", VelocityCmdScale, 1.0);
        unpackParameter("MaxVelocityCmd", MaxVelocityCmd, 0.5);
        unpackParameter("MaxTurnRateCmd", MaxTurnRateCmd, 0.5);
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
        unpackParameter("TurningOutputFilter", TurningOutputFilter, 0.0);
        unpackParameter("TiltOffset", TiltOffset, 0.0);
        unpackParameter("TiltDotOffset", TiltDotOffset, 0.0);
        unpackParameter("RollDotOffset", RollDotOffset, 0.0);
        unpackParameter("YawDotOffset", YawDotOffset, 0.0);
        std::string defaultImuName = "bno055";
        unpackParameter("ImuName", ImuName, defaultImuName);

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

            hardware_interface::JointHandle j = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle((std::string) name_value);
            joints_.push_back(j);
        }

        // Setup publishers and subscribers
        pub_status = new realtime_tools::RealtimePublisher<bobble_controllers::BobbleBotStatus>(root_nh, "bobble_balance_controller/bb_controller_status", 1);
        // Only do IMU subscription in sim.
        if(InSim){
            sub_imu_sensor_ = node_.subscribe("/imu_bosch/data_raw", 1, &BobbleBalanceController::imuCB, this);
        } else {
            // get a pointer to the imu interface
            hardware_interface::ImuSensorInterface* imu_hw = robot_hw->get<hardware_interface::ImuSensorInterface>();
            if (!imu_hw)
            {
                ROS_ERROR("This controller requires a hardware interface of type hardware_interface::ImuSensorInterface.");
                return false;
            }
            imuData = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(ImuName);
        }

        /// Set up data to be transferred in subscriber thread
        /// Bools to transfer
        controlBoolNames.push_back("StartupCmd");
        controlBoolNames.push_back("IdleCmd");
        controlBoolNames.push_back("DiagnosticCmd");
        /// Doubles for Transfer
        controlDoubleNames.push_back("DesiredVelocity");
        controlDoubleNames.push_back("DesiredTurnRate");
        populateControlCommandNames();

        /// Set up subscribers
        callbackFunctionPtr_t scCallback = boost::bind(&BobbleBalanceController::stateCommandCallback, this, _1);
        callbackFunctionPtr_t vcCallback = boost::bind(&BobbleBalanceController::velocityCommandCallback, this, _1);
        subscribers["/bobble/bobble_balance_controller/bb_cmd"] = scCallback;
        subscribers["/bobble/bobble_balance_controller/cmd_vel"] = vcCallback;

        subscriberFrequency = 20.0;
	    runSubscriberThread = true;
        subscriberThread = new std::thread(&BobbleBalanceController::subscriberFunction, this);

        // Setup Measured State Filters
        MeasuredTiltFilter.setGain(MeasuredTiltFilterGain);
        MeasuredTiltDotFilter.setGain(MeasuredTiltDotFilterGain);
        MeasuredHeadingFilter.setGain(MeasuredHeadingFilterGain);
        MeasuredTurnRateFilter.setGain(MeasuredTurnRateFilterGain);
        LeftWheelVelocityFilter.setGain(LeftWheelVelocityFilterGain);
        RightWheelVelocityFilter.setGain(RightWheelVelocityFilterGain);
        DesiredForwardVelocityFilter.setGain(DesiredForwardVelocityFilterGain);
        DesiredTurnRateFilter.setGain(DesiredTurnRateFilterGain);

        // Setup PID Controllers
        VelocityControlPID.setPID(VelocityControlKp, VelocityControlKi, 0.0);
        VelocityControlPID.setOutputFilter(VelocityControlAlphaFilter);
        VelocityControlPID.setMaxIOutput(VelocityControlMaxIntegralOutput);
        VelocityControlPID.setOutputLimits(-VelocityControlOutputLimitDegrees * (M_PI / 180.0),
                                           VelocityControlOutputLimitDegrees * (M_PI / 180.0));
        VelocityControlPID.setDirection(true);

        TiltControlPID.setPID(TiltControlKp, 0.0, TiltControlKd);
        TiltControlPID.setExternalDerivativeError(&TiltDot);
        TiltControlPID.setOutputFilter(TiltControlAlphaFilter);
        TiltControlPID.setOutputLimits(-MotorEffortMax, MotorEffortMax);
        TiltControlPID.setDirection(true);
        TiltControlPID.setSetpointRange(20.0 * (M_PI / 180.0));

        TurningControlPID.setPID(TurningControlKp, TurningControlKi, TurningControlKd);
        TurningControlPID.setOutputFilter(TurningOutputFilter);
        TurningControlPID.setMaxIOutput(1.0);
        TurningControlPID.setOutputLimits(-MotorEffortMax / 2.0, MotorEffortMax / 2.0);
        TurningControlPID.setDirection(false);
        TurningControlPID.setSetpointRange(45.0 * (M_PI / 180.0));

        state_ = INITIALIZED;
        return true;
    }


    void BobbleBalanceController::starting(const ros::Time &time) {
        ActiveControlMode = ControlModes::IDLE;
        StartupCmd = false;
        DiagnosticCmd = false;
        DesiredVelocity = 0.0;
        DesiredVelocityRaw = 0.0;
        DesiredTilt = 0.0;
        DesiredTurnRate = 0.0;
        DesiredTurnRateRaw = 0.0;
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

    void BobbleBalanceController::populateImuData()
    {
        MeasuredRollDot = imuData.getAngularVelocity()[0] + RollDotOffset;
        MeasuredTiltDot = imuData.getAngularVelocity()[1] + TiltDotOffset;
        MeasuredTurnRate = imuData.getAngularVelocity()[2] + YawDotOffset;
        MadgwickAHRSupdateIMU(MadgwickFilterGain, MeasuredRollDot, MeasuredTiltDot, MeasuredTurnRate,
                              imuData.getLinearAcceleration()[0], imuData.getLinearAcceleration()[1], imuData.getLinearAcceleration()[2]);
        // Construct a DCM matrix from the quaternion
        tf::Quaternion q(q0, q1, q2, q3);
        tf::Matrix3x3 m(q);
        m.getRPY(MeasuredHeading, MeasuredTilt, MeasuredRoll);
        MeasuredTilt += TiltOffset;
    }

    void BobbleBalanceController::imuCB(const sensor_msgs::Imu::ConstPtr &imuData) {
        // NOTE: IMU model in sim uses slightly different reference frames so this
        // function will have sim specific code. Eventually the hardware will not
        // rely on this IMU call back at all and therefore this entire function
        // will be a sim interface only.
        MeasuredTiltDot = -imuData->angular_velocity.y;
        MeasuredTurnRate = imuData->angular_velocity.z;
        // Call Madgwick orientation filter.
        MadgwickAHRSupdateIMU(MadgwickFilterGain, imuData->angular_velocity.x, imuData->angular_velocity.y, imuData->angular_velocity.z,
							  imuData->linear_acceleration.x, imuData->linear_acceleration.y,
							  imuData->linear_acceleration.z);
        // Construct a DCM matrix from the quaternion
        tf::Quaternion q(q0, q1, q2, q3);
		tf::Matrix3x3 m(q);
        m.getRPY(MeasuredHeading, MeasuredTilt, MeasuredRoll);
        MeasuredTilt *= -1.0;
    }

    void BobbleBalanceController::applyFilters()
    {
        Tilt = MeasuredTiltFilter.filter(MeasuredTilt);
        TiltDot = MeasuredTiltDotFilter.filter(MeasuredTiltDot);
        Heading = MeasuredHeadingFilter.filter(MeasuredHeading);
        TurnRate = MeasuredTurnRateFilter.filter(MeasuredTurnRate);

        // Filter wheel velocities and apply a wheel velocity adjustment in order to remove
        // a perceived wheel motion due to pendulum rotation
        LeftWheelVelocity = LeftWheelVelocityFilter.filter(MeasuredLeftMotorVelocity) * WheelVelocityAdjustment;
        RightWheelVelocity = RightWheelVelocityFilter.filter(MeasuredRightMotorVelocity) * WheelVelocityAdjustment;

        // Compute estimate forward velocity and turn rate.
        ForwardVelocity = WheelRadiusMeters*(RightWheelVelocity + LeftWheelVelocity)/2.0;

        // Filter Command Inputs
        DesiredVelocity = DesiredForwardVelocityFilter.filter(controlDoublesRT["DesiredVelocity"]*VelocityCmdScale);
        DesiredTurnRate = DesiredTurnRateFilter.filter(controlDoublesRT["DesiredTurnRate"]*TurnCmdScale);

        /// Limit Velocity Command
        DesiredVelocity = limit(DesiredVelocity, MaxVelocityCmd);
        DesiredTurnRate = limit(DesiredTurnRate, MaxTurnRateCmd);
    }

    void BobbleBalanceController::populateOdometry()
    {
        // Get odometry information.
        MeasuredLeftMotorPosition = joints_[1].getPosition();
        MeasuredRightMotorPosition = joints_[0].getPosition();
        MeasuredLeftMotorVelocity = joints_[1].getVelocity();
        MeasuredRightMotorVelocity = joints_[0].getVelocity();
    }

    void BobbleBalanceController::applySafety()
    {
        // No effort when tilt angle is way out of whack.
        // You're going down. Don't fight it... just accept it.
        if (abs(Tilt) >= MaxTiltSafetyLimitDegrees * (M_PI / 180.0)) {
            RightMotorEffortCmd = 0.0;
            LeftMotorEffortCmd = 0.0;
        }
    }

    void BobbleBalanceController::update(const ros::Time &time, const ros::Duration &duration) {
        /// Populate Command Variables
        populateControlCommands();
        /// If not in simulation, populate the IMU data from sensor handles
        if(!InSim){
           populateImuData();
        }
        /// Populate Odometry from the wheels
        populateOdometry();
        /// Apply Filtering
        applyFilters();

        /////////////////////////////////////////////////////////////////////////////////////////
        /// Perform the desired control depending on BobbleBot controller state
        /////////////////////////////////////////////////////////////////////////////////////////
        if (ActiveControlMode == ControlModes::IDLE) {
            // Sim only q reset
            if (InSim) {
                q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
            }
            VelocityControlPID.reset();
            TiltControlPID.reset();
            TurningControlPID.reset();
            DesiredTilt = 0.0;
            TiltEffort = 0.0;
            HeadingEffort = 0.0;
            if (controlBoolsRT["StartupCmd"]) {
                ActiveControlMode = ControlModes::STARTUP;
            }
            if (controlBoolsRT["DiagnosticCmd"]) {
                ActiveControlMode = ControlModes::DIAGNOSTIC;
            }
        } else if (ActiveControlMode == ControlModes::DIAGNOSTIC)
		{
			TiltEffort = DesiredVelocity;
			HeadingEffort = DesiredTurnRate;
            if (controlBoolsRT["IdleCmd"]) {
                ActiveControlMode = ControlModes::IDLE;
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
	        DesiredTilt *= -1.0;
            TiltEffort = TiltControlPID.getOutput(DesiredTilt, Tilt);
            HeadingEffort = TurningControlPID.getOutput(DesiredTurnRate, TurnRate);
            if (controlBoolsRT["IdleCmd"]) {
                ActiveControlMode = ControlModes::IDLE;
            }
        } else {
            TiltEffort = 0.0;
            HeadingEffort = 0.0;
        }

        /////////////////////////////////////////////////////////////////////////////////////////
        /// Combine heading and tilt efforts to achieve velocity, tilt, and turning control.
        /////////////////////////////////////////////////////////////////////////////////////////
        RightMotorEffortCmd = TiltEffort - HeadingEffort;
        LeftMotorEffortCmd = TiltEffort + HeadingEffort;

        /////////////////////////////////////////////////////////////////////////////////////////
        /// Apply safety checks
        /////////////////////////////////////////////////////////////////////////////////////////
        applySafety();
        // Apply motor effort limits
        RightMotorEffortCmd = limit(RightMotorEffortCmd, MotorEffortMax);
        LeftMotorEffortCmd = limit(LeftMotorEffortCmd, MotorEffortMax);

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
        /// Report our status out using RT safe publisher
        /////////////////////////////////////////////////////////////////////////////////////////
        publish_status_message();
    }

    void BobbleBalanceController::publish_status_message() {
        if(pub_status->trylock()) {
            pub_status->msg_.ControlMode = ActiveControlMode;
            pub_status->msg_.MeasuredTiltDot = MeasuredTiltDot * (180.0 / M_PI);
            pub_status->msg_.MeasuredTurnRate = MeasuredTurnRate * (180.0 / M_PI);
            pub_status->msg_.Tilt = Tilt * (180.0 / M_PI);
            pub_status->msg_.TiltRate = TiltDot * (180.0 / M_PI);
            pub_status->msg_.Heading = Heading * (180.0 / M_PI);
            pub_status->msg_.TurnRate = TurnRate * (180.0 / M_PI);
            pub_status->msg_.ForwardVelocity = ForwardVelocity;
            pub_status->msg_.DesiredVelocity = DesiredVelocity;
	        pub_status->msg_.DesiredTilt = DesiredTilt * (180.0 / M_PI);
            pub_status->msg_.DesiredTurnRate = DesiredTurnRate * (180.0 / M_PI);
            pub_status->msg_.LeftMotorPosition = MeasuredLeftMotorPosition * (180.0 / M_PI);
            pub_status->msg_.LeftMotorVelocity = MeasuredLeftMotorVelocity * (180.0 / M_PI);
            pub_status->msg_.RightMotorPosition = MeasuredRightMotorPosition * (180.0 / M_PI);
            pub_status->msg_.RightMotorVelocity = MeasuredRightMotorVelocity * (180.0 / M_PI);
            pub_status->msg_.TiltEffort = TiltEffort;
            pub_status->msg_.HeadingEffort = HeadingEffort;
            pub_status->msg_.LeftMotorEffortCmd = LeftMotorEffortCmd;
            pub_status->msg_.RightMotorEffortCmd = RightMotorEffortCmd;
            pub_status->unlockAndPublish();
        }
    }

}

PLUGINLIB_EXPORT_CLASS(bobble_controllers::BobbleBalanceController, controller_interface::ControllerBase)

