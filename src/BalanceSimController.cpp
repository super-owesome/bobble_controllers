#include <sys/mman.h>
#include <bobble_controllers/BalanceSimController.h>
#include <pluginlib/class_list_macros.h>

namespace bobble_controllers {

    void BalanceSimController::loadConfig() {
        unpackParameter("MotorEffortToTorqueSimFactor", config.MotorEffortToTorqueSimFactor, 0.832);
        BalanceBaseController::loadConfig();
    }

    bool BalanceSimController::initRequest(hardware_interface::RobotHW* robot_hw,
                             ros::NodeHandle&             root_nh,
                             ros::NodeHandle&             controller_nh,
                             ClaimedResources&            claimed_resources)
    {
        /// check if construction finished cleanly
        if (state_ != CONSTRUCTED){
            ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
            return false;
        }
        node_ = controller_nh;
        loadConfig();
        BalanceBaseController::setupFilters();
        BalanceBaseController::setupControllers();
        pub_bobble_status_ = new realtime_tools::RealtimePublisher<bobble_controllers::BobbleBotStatus>(root_nh,
                                                    "bobble_balance_controller/bb_controller_status", 1);
	    run_thread_ = true;
        subscriber_thread_ = new std::thread(&BalanceBaseController::runSubscriber, this);
        /// get a pointer to the effort interface
        hardware_interface::EffortJointInterface* effort_hw = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (!effort_hw)
        {
            ROS_ERROR("This controller requires a hardware interface of type hardware_interface::EffortJointInterface.");
            return false;
        }
        /// Load the joint names from the controller config file
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
        sub_imu_sensor_ = node_.subscribe("/imu_bosch/data_raw", 1, &BalanceSimController::imuCB, this);
        state_ = INITIALIZED;
        return true;
    }

    void BalanceSimController::starting(const ros::Time &time) {
        BalanceBaseController::reset();
        // Reset Madgwick Q on start is a sim only thing. The sim
        // resets the orientation on transition from Idle to Balance
        q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
    }

    void BalanceSimController::update(const ros::Time &time, const ros::Duration &duration) {
        /// Reset the quaternion every time we go to idle in sim
        if (state.ActiveControlMode == bobble_controllers::ControlModes::IDLE) {
            q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
        }
        BalanceBaseController::update();
    }

    void BalanceSimController::imuCB(const sensor_msgs::Imu::ConstPtr &imuData) {
        state.MeasuredTiltDot = -imuData->angular_velocity.y;
        state.MeasuredTurnRate = imuData->angular_velocity.z;
        MadgwickAHRSupdateIMU(config.MadgwickFilterGain, imuData->angular_velocity.x,
                              imuData->angular_velocity.y, imuData->angular_velocity.z,
							  imuData->linear_acceleration.x, imuData->linear_acceleration.y,
							  imuData->linear_acceleration.z);
        tf::Quaternion q(q0, q1, q2, q3);
		tf::Matrix3x3 m(q);
        m.getRPY(state.MeasuredHeading, state.MeasuredTilt, state.MeasuredRoll);
        state.MeasuredTilt *= -1.0;
    }

    void BalanceSimController::estimateState()
    {
        state.MeasuredLeftMotorPosition = joints_[1].getPosition();
        state.MeasuredRightMotorPosition = joints_[0].getPosition();
        state.MeasuredLeftMotorVelocity = joints_[1].getVelocity();
        state.MeasuredRightMotorVelocity = joints_[0].getVelocity();
    }

    void BalanceSimController::sendMotorCommands()
    {
        joints_[0].setCommand(outputs.RightMotorEffortCmd * config.MotorEffortToTorqueSimFactor);
        joints_[1].setCommand(outputs.LeftMotorEffortCmd  * config.MotorEffortToTorqueSimFactor);
    }

}

PLUGINLIB_EXPORT_CLASS(bobble_controllers::BalanceSimController, controller_interface::ControllerBase
)
