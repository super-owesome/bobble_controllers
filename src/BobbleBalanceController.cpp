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
		if(!node_.getParam("MotorEffortMax",MotorEffortMax))
		{
		    MotorEffortMax = 1.0;
			ROS_WARN("MotorEffortMax not set for (namespace: %s) using 1.0.",
			        node_.getNamespace().c_str());
		}
		if(!node_.getParam("PitchGain",PitchGain))
		{
		    PitchGain = 0.0;
			ROS_WARN("PitchGain not set for (namespace: %s) using 0.0.",
			        node_.getNamespace().c_str());
		}
		if(!node_.getParam("PitchDotGain",PitchDotGain))
		{
		    PitchDotGain = 0.0;
			ROS_WARN("PitchDotGain not set for (namespace: %s) using 0.0.",
			        node_.getNamespace().c_str());
		}
		if(!node_.getParam("YawGain",YawGain))
		{
		    YawGain = 0.0;
			ROS_WARN("YawGain not set for (namespace: %s) using 0.0.",
			        node_.getNamespace().c_str());
		}
		if(!node_.getParam("YawDotGain",YawDotGain))
		{
		    YawDotGain = 0.0;
			ROS_WARN("YawDotGain not set for (namespace: %s) using 0.0.",
			        node_.getNamespace().c_str());
		}
		if(!node_.getParam("WheelGain",WheelGain))
		{
		    WheelGain = 0.0;
			ROS_WARN("WheelGain not set for (namespace: %s) using 0.0.",
			        node_.getNamespace().c_str());
		}
		if(!node_.getParam("WheelDotGain",WheelDotGain))
		{
		    WheelDotGain = 0.0;
			ROS_WARN("WheelDotGain not set for (namespace: %s) using 0.0.",
			        node_.getNamespace().c_str());
		}
		if(!node_.getParam("WheelIntegralGain",WheelIntegralGain))
		{
		    WheelIntegralGain = 0.0;
			ROS_WARN("WheelIntegralGain not set for (namespace: %s) using 0.0.",
			        node_.getNamespace().c_str());
		}
		if(!node_.getParam("WheelIntegralSaturation",WheelIntegralSaturation))
		{
		    WheelIntegralSaturation = 0.0;
			ROS_WARN("WheelIntegralSaturation not set for (namespace: %s) using 0.0.",
			        node_.getNamespace().c_str());
		}
		if(!node_.getParam("PendulumStateAlpha",PendulumStateAlpha))
		{
		    PendulumStateAlpha = 0.0;
			ROS_WARN("PendulumStateAlpha not set for (namespace: %s) using 0.0.",
			        node_.getNamespace().c_str());
		}
		if(!node_.getParam("WheelStateAlpha",WheelStateAlpha))
		{
		    WheelStateAlpha = 0.0;
			ROS_WARN("WheelStateAlpha not set for (namespace: %s) using 0.0.",
			        node_.getNamespace().c_str());
		}
		if(!node_.getParam("EffortPendulumAlpha",EffortPendulumAlpha))
		{
		    EffortPendulumAlpha = 0.0;
			ROS_WARN("EffortPendulumAlpha not set for (namespace: %s) using 0.0.",
			        node_.getNamespace().c_str());
		}
		if(!node_.getParam("EffortWheelAlpha",EffortWheelAlpha))
		{
		    EffortWheelAlpha = 0.0;
			ROS_WARN("EffortWheelAlpha not set for (namespace: %s) using 0.0.",
			        node_.getNamespace().c_str());
		}

        // Setup publishers and subscribers
		pub_bobble_status = n.advertise<executive::BobbleBotStatus>("bb_controller_status", 1);
		sub_imu_sensor_ = node_.subscribe("/imu_bosch/data",1,
		        &BobbleBalanceController::imuCB, this);
		sub_command_=node_.subscribe("/bb_cmd",1,
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
        param.sched_priority=sched_get_priority_max(SCHED_FIFO);
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
		double pitch_error, yaw_error, effort;
		if (ActiveControlMode == ControlModes::IDLE)
		{
			effort = 0.0;
		}
		else if (ActiveControlMode == ControlModes::DRIVE)
		{
		    effort = DesiredPitch;
		}
		else if (ActiveControlMode == ControlModes::BALANCE)
		{
			pitch_error = Pitch;
			yaw_error = Yaw;
			effort = 1.0 * pitch_error + 0.25 * PitchDot + 0.025 * RightWheelVelocity;
		}
		// PD control
	    // Send effort commands
	    LeftMotorEffortCmd = effort;
	    RightMotorEffortCmd = effort;
	    if (ActiveControlMode != ControlModes::IDLE)
	    {
	        joints_[0].setCommand(LeftMotorEffortCmd);
	        joints_[1].setCommand(RightMotorEffortCmd);
	    }
        // Write out status message
        write_controller_status_msg();
	}

	void BobbleBalanceController::write_controller_status_msg() {
		executive::BobbleBotStatus sim_status_msg;
		sim_status_msg.ControlMode = ActiveControlMode;
		sim_status_msg.DeltaT = 0.0;
		sim_status_msg.Roll = Roll;
		sim_status_msg.Pitch = Pitch;
		sim_status_msg.Yaw = Yaw;
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
