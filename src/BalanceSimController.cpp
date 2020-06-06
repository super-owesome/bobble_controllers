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
        state.MeasuredTiltDot = imuData->angular_velocity.y;
        state.MeasuredTurnRate = imuData->angular_velocity.z;
        MadgwickAHRSupdateIMU(config.MadgwickFilterGain, imuData->angular_velocity.x,
                              imuData->angular_velocity.y, imuData->angular_velocity.z,
							  imuData->linear_acceleration.x, imuData->linear_acceleration.y,
							  imuData->linear_acceleration.z);
        tf::Quaternion q(q0, q1, q2, q3);

        /// Broadcast the quaternion
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "bobble_chassis_link";
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = q0;
        transformStamped.transform.rotation.y = q1;
        transformStamped.transform.rotation.z = q2;
        transformStamped.transform.rotation.w = q3;

        tf2_ros::TransformBroadcaster tfb;

        tfb.sendTransform(transformStamped);

		tf::Matrix3x3 m(q);
        m.getRPY(state.MeasuredHeading, state.MeasuredTilt, state.MeasuredRoll);
        /// These need to be negated to match bb2 frames - roll isn't used
        state.MeasuredTilt *= -1.0;
        state.MeasuredHeading *= -1.0;
        /// This is making the heading go between 0 and 2 PI instead of -PI and PI
        state.MeasuredHeading += M_PI;

        /// Wrap or unwrap the heading so that there are no discontinuities
        if(state.MeasuredHeading - state.PreviousMeasuredHeading > M_PI)
        {
            state.NumberOfWraps -= 1;
        }
        else if(state.MeasuredHeading - state.PreviousMeasuredHeading < -M_PI)
        {
            state.NumberOfWraps += 1;
        }

        /// Wrapped heading is current heading plus 2 pi times the number of wraps
        state.WrappedMeasuredHeading = state.MeasuredHeading + 2.0 * M_PI * state.NumberOfWraps;
        state.PreviousMeasuredHeading = state.MeasuredHeading;
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
