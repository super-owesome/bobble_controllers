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
			ROS_WARN("%s not set for (namespace: %s) using %f.",
					 parameterName.c_str(),
					 node_.getNamespace().c_str(),
			         defaultValue);
		}
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
			
			hardware_interface::JointHandle j=robot->
			        getHandle((std::string)name_value);
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
		unpackParameter("RollOffset", RollOffset, 0.0);
		unpackParameter("PitchOffset", PitchOffset, 0.0);
		unpackParameter("YawOffset", YawOffset, 0.0);

        // Setup publishers and subscribers
		pub_bobble_status = n.advertise<executive::BobbleBotStatus>("bb_controller_status", 1);
		sub_imu_sensor_ = node_.subscribe("/imu_bosch/data",1,
		        &BobbleBalanceController::imuCB, this);
		sub_command_=node_.subscribe("/bobble/bobble_balance_controller/bb_cmd",1,
		        &BobbleBalanceController::commandCB, this);

		return true;
	}
	
	void BobbleBalanceController::starting(const ros::Time& time)
	{
		ActiveControlMode = ControlModes::IDLE;
		DesiredPitch = 0.0;
		DesiredYaw = 0.0;
		Pitch = 0.0;
		RollDot = 0.0;
		PitchDot = 0.0;
		YawDot = 0.0;
		LeftMotorEffortCmd = 0.0;
		LeftWheelPosition = 0.0;
		LeftWheelVelocity = 0.0;
		RightMotorEffortCmd = 0.0;
		RightWheelPosition = 0.0;
		RightWheelVelocity = 0.0;
        LeftWheelErrorAccumulated = 0.0;
		RightWheelErrorAccumulated = 0.0;
        DesiredLeftWheelPosition = 0.0;
        DesiredRightWheelPosition = 0.0;
		//WheelGains.setZero();
		//PendulumGains.setZero();
		//EstimatedPendulumState.setZero();
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
        LeftWheelPosition = joints_[0].getPosition();
		LeftWheelVelocity = joints_[0].getVelocity();
		RightWheelPosition = joints_[1].getPosition();
		RightWheelVelocity = joints_[1].getVelocity();
		double roll_error, yaw_error;
		if (ActiveControlMode == ControlModes::IDLE)
		{
			LeftMotorEffortCmd = 0.0;
			RightMotorEffortCmd = 0.0;
		}
		else if (ActiveControlMode == ControlModes::DRIVE)
		{
			LeftMotorEffortCmd = DesiredPitch;
			RightMotorEffortCmd = -DesiredPitch;
		}
		else if (ActiveControlMode == ControlModes::BALANCE)
		{
			roll_error = Roll - RollOffset;
			yaw_error = Yaw - YawOffset;
			LeftMotorEffortCmd = PitchGain * roll_error + PitchDotGain * RollDot - WheelDotGain * LeftWheelVelocity;
			RightMotorEffortCmd = PitchGain * roll_error + PitchDotGain * RollDot - WheelDotGain * RightWheelVelocity;
			//effort = 1.0 * roll_error;// + 0.25 * PitchDot + 0.025 * RightWheelVelocity;
		}
		// PD control
	    // Send effort commands
	    joints_[0].setCommand(LeftMotorEffortCmd);
	    joints_[1].setCommand(RightMotorEffortCmd);
        // Write out status message
        write_controller_status_msg();
	}

	void BobbleBalanceController::write_controller_status_msg() {
		executive::BobbleBotStatus sim_status_msg;
		sim_status_msg.ControlMode = ActiveControlMode;
		sim_status_msg.DeltaT = 0.0;
		sim_status_msg.Roll = Roll - RollOffset;
		sim_status_msg.Pitch = Pitch - PitchOffset;
		sim_status_msg.Yaw = Yaw - YawOffset;
		sim_status_msg.RollRate = RollDot;
		sim_status_msg.PitchRate = PitchDot;
		sim_status_msg.YawRate = YawDot;
		sim_status_msg.DesiredPitch = DesiredPitch;
		sim_status_msg.DesiredYaw = DesiredYaw;
		sim_status_msg.LeftMotorEffortCmd = LeftMotorEffortCmd;
		sim_status_msg.LeftMotorPosition = LeftWheelPosition;
		sim_status_msg.LeftMotorVelocity = LeftWheelVelocity;
		sim_status_msg.RightMotorEffortCmd = RightMotorEffortCmd;
		sim_status_msg.RightMotorPosition = RightWheelPosition;
		sim_status_msg.RightMotorVelocity = RightWheelVelocity;
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
