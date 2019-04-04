//
// Created by james on 4/3/19.
//

#ifndef SRC_SINGLEWHEELCONTROLLER_H
#define SRC_SINGLEWHEELCONTROLLER_H

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
#include <controller_interface/controller.h>
#include <sensor_msgs/JointState.h>
#include <bobble_controllers/ControlCommands.h>
#include <bobble_controllers/PidControl.h>
#include <bobble_controllers/Filter.h>
#include <bobble_controllers/BobbleBotStatus.h>
#include <tf/transform_datatypes.h>

namespace bobble_controllers {

    class SingleWheelController : public controller_interface::
    Controller<hardware_interface::EffortJointInterface> {
        ros::NodeHandle node_;
        hardware_interface::robothw *robot_;
        std::vector <hardware_interface::JointHandle> joints_;
        realtime_tools::RealtimePublisher<bobble_controllers::WheelStatus>* pub_wheel_status;
        ros::Subscriber sub_command_;

        enum ControlModes {
            IDLE,
            VOLTAGE,
            VELOCITY,
            POSITION
        };

        typedef struct {
            bool IdleCmd;
            bool VoltageCmd;
            bool VelocityCmd;
            bool PositionCmd;
            double DesiredVoltage;
            double DesiredVelocity;
            double DesiredPosition;
        } CommandStruct;

        std::mutex control_command_mutex;

        std::thread* subscriberThread;

        //! There is a struct for use and a temporary struct
        //! This is so the non-realtime subscriber can pass data into the class through a mutex
        //! to keep the control thread realtime
        CommandStruct commandStruct;
        CommandStruct commandStructTmp;

        bool runThread;
        void runSubscriber();
        void transitionCallback(const bobble_controllers::WheelControlCommands::ConstPtr &cmd);
        void commandCallback(const bobble_controllers::WheelCommands::ConstPtr &cmd);

        typedef struct {
            double Kp;
            double Kd;
            double Ki;
        } GainStruct;
        /// Controller config
        bool InSim; // Temporary hack. Need to re-orient IMU model.
        double MaxVoltageCmd;
        double MaxVelocityCmd;
        double MaxPositionCmd;
        GainStruct VelocityGains;
        GainStruct PositionGains;

        /// Measured state
        double MeasuredMotorPosition;
        double MeasuredMotorVelocity;

        /// PID Controllers
        PidControl VelocityControlPID;
        PidControl PositionControlPID;

        /// Controller outputs
        double MotorEffortCmd;

        void starting(const ros::Time &time);

        void update(const ros::Time &time, const ros::Duration &duration);

        void write_controller_status_msg();

        void unpackParameter(std::string parameterName, double &referenceToParameter, double defaultValue);
        void unpackParameter(std::string parameterName, std::string &referenceToParameter, std::string defaultValue);

        void unpackFlag(std::string parameterName, bool &referenceToFlag, bool defaultValue);

        double limit(double cmd, double max);

        void populateCommands();

    protected:
        virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                                 ros::NodeHandle&             root_nh,
                                 ros::NodeHandle&             controller_nh,
                                 ClaimedResources&            claimed_resources) override;
    public:
        SingleWheelController(void);

        ~SingleWheelController(void);

    };
}
#endif //SRC_SINGLEWHEELCONTROLLER_H
