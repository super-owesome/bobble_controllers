//
// Created by mike on 6/23/18.
//

#ifndef SRC_BOBBLEBOTHW_H
#define SRC_BOBBLEBOTHW_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <chupacabra_comm/LinuxTransporter.h>
#include <chupacabra_comm/ChupCanLinuxDriver.h>

class BobbleBotHw : public hardware_interface::RobotHW
{
public:
    BobbleBotHw();
    ~BobbleBotHw();

    void init();
    void read();
    void write();

    std::string left_motor_joint_name;
    std::string right_motor_joint_name;

    ChupCanLinuxDriver left_motor_chup_can_interface;
    ChupCanLinuxDriver right_motor_chup_can_interface;

private:
    hardware_interface::EffortJointInterface jnt_effort_interface;
    hardware_interface::ImuSensorInterface imu_interface;
    LinuxTransporter   left_motor_chup_can_transporter;
    LinuxTransporter   right_motor_chup_can_transporter;
    double LeftMotorPosition;
    double LeftMotorVelocity;
    double LeftMotorTorque;
    double LeftMotorCmdVoltage;
    double RightMotorPosition;
    double RightMotorVelocity;
    double RightMotorTorque;
    double RightMotorCmdVoltage;
    double LinearAcceleration[3];
    double AngularVelocity[3];

};

#endif //SRC_BOBBLEBOTHW_H
