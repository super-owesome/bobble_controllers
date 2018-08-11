//
// Created by mike on 6/23/18.
//

#ifndef SRC_BOBBLEBOTHW_H
#define SRC_BOBBLEBOTHW_H

#include <hardware_interface/robot_hw.h>
#include <bobble_controllers/ChupJointCommandInterface.h>
#include <bobble_controllers/ChupJointStateInterface.h>
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
    hardware_interface::ChupEffortJointInterface jnt_effort_interface;
    LinuxTransporter   left_motor_chup_can_transporter;
    CommData           left_motor_chup_joint_data;
    LinuxTransporter   right_motor_chup_can_transporter;
    CommData           right_motor_chup_joint_data;

};

#endif //SRC_BOBBLEBOTHW_H
