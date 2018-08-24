/******************************************************************************
 * Document this if it actually works
 * MMM
*******************************************************************************/

#include <sys/mman.h>

#include <bobble_controllers/BobbleBalanceController.h>
#include <pluginlib/class_list_macros.h>

namespace bobble_controllers
{	
	BobbleBalanceController::BobbleBalanceController(void)
	{
	}
	
	BobbleBalanceController::~BobbleBalanceController(void)
	{
		sub_command_.shutdown();
	}

	void BobbleBalanceController::unpackParameter(std::string parameterName, double &referenceToParameter, double defaultValue)
	{
		if(!node_.getParam(parameterName, referenceToParameter))
		{
			referenceToParameter = defaultValue;
			ROS_ERROR("%s not set for (namespace: %s) using %f.",
					 parameterName.c_str(),
					 node_.getNamespace().c_str(),
			         defaultValue);
		}
	}

	void BobbleBalanceController::packGains(double PitchGain, double PitchDotGain, double RightWheelGain, double RightWheelDotGain, double LeftWheelGain, double LeftWheelDotGain)
	{
		ControlGains(0, 0) = ControlGains(0, 1) = PitchGain;
		ControlGains(1, 0) = ControlGains(1, 1) = PitchDotGain;
		ControlGains(0, 2) = RightWheelGain;
		ControlGains(0, 3) = RightWheelDotGain;
		ControlGains(1, 4) = LeftWheelGain;
		ControlGains(1, 5) = LeftWheelDotGain;
		ControlGains(1, 2) = ControlGains(1, 3) = ControlGains(0, 4) = ControlGains(0, 5) = 0.0;
	}


	void BobbleBalanceController::packState(double Pitch, double PitchDot, double RightWheelPosition, double RightWheelVelocity, double LeftWheelPosition, double LeftWheelVelocity)
	{
	    EstimatedState(0) = Pitch;
		EstimatedState(1) = PitchDot;
		EstimatedState(2) = RightWheelPosition;
		EstimatedState(3) = RightWheelVelocity;
		EstimatedState(4) = LeftWheelPosition;
		EstimatedState(5) = LeftWheelVelocity;
	}

	void BobbleBalanceController::packDesired(double PitchDesired, double PitchDotDesired, double RightWheelPositionDesired, double RightWheelVelocityDesired, double LeftWheelPositionDesired, double LeftWheelVelocityDesired)
	{
		DesiredState(0) = PitchDesired;
		DesiredState(1) = PitchDotDesired;
		DesiredState(2) = RightWheelPositionDesired;
		DesiredState(3) = RightWheelVelocityDesired;
		DesiredState(4) = LeftWheelPositionDesired;
		DesiredState(5) = LeftWheelVelocityDesired;
	}

	bool BobbleBalanceController::init(hardware_interface::EffortJointInterface *robot,ros::NodeHandle &n)
	{
		node_=n;
		robot_=robot;

		XmlRpc::XmlRpcValue joint_names;
		if(!node_.getParam("joints",joint_names))
		{
			ROS_ERROR("No 'joints' in controller. (namespace: %s)",
			        node_.getNamespace().c_str());
			return false;
		}
		
		if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
		{
			ROS_ERROR("'joints' is not a struct. (namespace: %s)",
			        node_.getNamespace().c_str());
			return false;
		}
		
		for(int i=0; i < joint_names.size();i++)
		{
			XmlRpc::XmlRpcValue &name_value=joint_names[i];
			if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
			{
				ROS_ERROR("joints are not strings. (namespace: %s)",
				        node_.getNamespace().c_str());
				return false;
			}
			
			hardware_interface::JointHandle j=robot->getHandle((std::string)name_value);
			joints_.push_back(j);
		}

		unpackParameter("MotorEffortMax", MotorEffortMax, 1.0);
		unpackParameter("PitchGain", PitchGain, 0.0);
		unpackParameter("PitchDotGain", PitchDotGain, 0.0);
		unpackParameter("YawGain", YawGain, 0.0);
		unpackParameter("YawDotGain", YawDotGain, 0.0);
		unpackParameter("WheelGain", WheelGain, 0.0);
		unpackParameter("WheelDotGain", WheelDotGain, 0.0);
		unpackParameter("WheelIntegralGain", WheelIntegralGain, 0.0);
		unpackParameter("WheelIntegralSaturation", WheelIntegralSaturation, 0.0);
		unpackParameter("PendulumStateAlpha", PendulumStateAlpha, 0.0);
		unpackParameter("WheelStateAlpha", WheelStateAlpha, 0.0);
		unpackParameter("EffortPendulumAlpha", EffortPendulumAlpha, 0.0);
		unpackParameter("EffortWheelAlpha", EffortWheelAlpha, 0.0);

		packGains(PitchGain, PitchDotGain, WheelGain, WheelDotGain, WheelGain, WheelDotGain);

        // Setup publishers and subscribers
		pub_bobble_status = n.advertise<executive::BobbleBotStatus>("bb_controller_status", 1);
		sub_imu_sensor_ = node_.subscribe("/imu_bosch/data",1, &BobbleBalanceController::imuCB, this);
		sub_command_=node_.subscribe("/bobble/bobble_balance_controller/bb_cmd",1, &BobbleBalanceController::commandCB, this);

		return true;
	}

	double BobbleBalanceController::limitEffort(double effort_cmd, double min_effort, double max_effort)
	{
	    if (effort_cmd <= min_effort) {
			return min_effort;
		}else if (effort_cmd>= max_effort){
	    	return max_effort;
	    }
		return effort_cmd;
	}

	void BobbleBalanceController::starting(const ros::Time& time)
	{
		ActiveControlMode = ControlModes::IDLE;
		DesiredPitch = 0.0;
		DesiredYaw = 0.0;
		Roll = 0.0;
		Pitch = 0.0;
		Yaw = 0.0;
		RollDot = 0.0;
		PitchDot = 0.0;
		YawDot = 0.0;
		EstimatedState.setZero();
		DesiredState.setZero();

        struct sched_param param;
        // set the priority high, but not so high it overrides the comm
        param.sched_priority=95;
        if(sched_setscheduler(0,SCHED_FIFO,&param) == -1)
        {
                ROS_WARN("Failed to set real-time scheduler.");
                return;
        }
        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
                ROS_WARN("Failed to lock memory.");
	}
	
	void BobbleBalanceController::update(const ros::Time& time, const ros::Duration& duration)
	{
	    // Load the BobbleBot state from sensed values
		packState(Pitch, PitchDot, joints_[0].getPosition(), joints_[0].getVelocity(), joints_[1].getPosition(), joints_[1].getVelocity());

		if (ActiveControlMode == ControlModes::DRIVE)
		{
			packDesired(DesiredPitch, 0.0, 0.0, 0.0, 0.0, 0.0);
		}
		else if (ActiveControlMode == ControlModes::BALANCE)
		{
			packDesired(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		}
	    ErrorState = EstimatedState - DesiredState;
	    Effort = ControlGains * ErrorState;

	    // Overwrite effort if we're in idle or diagnostic modes
		if (ActiveControlMode == ControlModes::IDLE)
		{
			Effort(0) = 0.0;
			Effort(1) = 0.0;
		}else if (ActiveControlMode == ControlModes::DIAGNOSTIC){
			Effort(0) = DesiredPitch;
			Effort(1) = DesiredYaw;
		}
		Effort(0) = limitEffort(Effort(0), -MotorEffortMax, MotorEffortMax);
		Effort(1) = limitEffort(Effort(1), -MotorEffortMax, MotorEffortMax);

		// Send the effort commands
        joints_[0].setCommand(Effort(0));
		//joints_[1].setCommand(Effort(0));

        // Write out status message
        write_controller_status_msg();
	}

	void BobbleBalanceController::write_controller_status_msg() {
		executive::BobbleBotStatus sim_status_msg;
		sim_status_msg.ControlMode = ActiveControlMode;
		sim_status_msg.DeltaT = 0.0;
		sim_status_msg.Roll = Roll;
		sim_status_msg.Pitch = EstimatedState(0);
		sim_status_msg.Yaw = Yaw;
		sim_status_msg.RollRate = RollDot;
		sim_status_msg.PitchRate = EstimatedState(1);
		sim_status_msg.YawRate = YawDot;
		sim_status_msg.DesiredPitch = DesiredPitch;
		sim_status_msg.DesiredYaw = DesiredYaw;
		sim_status_msg.LeftMotorEffortCmd = Effort(1);
		sim_status_msg.LeftMotorPosition = EstimatedState(4);
		sim_status_msg.LeftMotorVelocity = EstimatedState(5);
		sim_status_msg.RightMotorEffortCmd = Effort(0);
		sim_status_msg.RightMotorPosition = EstimatedState(2);
		sim_status_msg.RightMotorVelocity = EstimatedState(3);
		pub_bobble_status.publish(sim_status_msg);
	}


	void BobbleBalanceController::imuCB(const sensor_msgs::Imu::ConstPtr &imuData) {
		tf::Quaternion q(imuData->orientation.x, imuData->orientation.y, imuData->orientation.z,
						 imuData->orientation.w);
		tf::Matrix3x3 m(q);
		RollDot = imuData->angular_velocity.x;
		PitchDot = imuData->angular_velocity.y;
		YawDot = imuData->angular_velocity.z;
		m.getRPY(Roll, Pitch, Yaw);
	}

	void BobbleBalanceController::commandCB(const bobble_controllers::ControlCommands::ConstPtr &cmd)
	{
		ActiveControlMode = cmd->ControlMode;
		DesiredPitch = cmd->DesiredPitch;
		DesiredYaw = cmd->DesiredYaw;
	}

}

PLUGINLIB_EXPORT_CLASS(bobble_controllers::BobbleBalanceController, controller_interface::ControllerBase)
