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
#include "adxl345_ros_node/adxl345_i2c.h"
#include "itg3200_ros_node/itg3200_i2c.h"

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

    std::string imu_name;
    std::string imu_link;

    ChupCanLinuxDriver left_motor_chup_can_interface;
    ChupCanLinuxDriver right_motor_chup_can_interface;

    adxl345_ros_node::ADXL345_I2C adxl345_i2c;
    itg3200_ros_node::ITG3200_I2C itg3200_i2c;

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

};

#endif //SRC_BOBBLEBOTHW_H
