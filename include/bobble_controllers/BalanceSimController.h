/******************************************************************************
 * Gazebo ROS interface for Bobble-Bot's balance controller.
 *
*******************************************************************************/

#ifndef BOBBLE_CONTROLLERS_BALANCE_SIM_CONTROLLER_H
#define BOBBLE_CONTROLLERS_BALANCE_SIM_CONTROLLER_H

#include <cstddef>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <thread>
#include <mutex>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <bobble_controllers/BalanceBaseController.h>
#include <tf/transform_datatypes.h>

namespace bobble_controllers {

    class BalanceSimController : public bobble_controllers::BalanceBaseController,
                                 public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        BalanceSimController(){};
        ~BalanceSimController(){};

        void starting(const ros::Time &time);
        void update(const ros::Time &time, const ros::Duration &duration);

    protected:
        virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                                 ros::NodeHandle&             root_nh,
                                 ros::NodeHandle&             controller_nh,
                                 ClaimedResources&            claimed_resources) override;


        virtual void estimateState();
        virtual void sendMotorCommands();

    private:
        hardware_interface::RobotHW *robot_;
        std::vector <hardware_interface::JointHandle> joints_;
        ros::Subscriber sub_imu_sensor_;
        void imuCB(const sensor_msgs::Imu::ConstPtr &imuData);
    };
}
#endif
